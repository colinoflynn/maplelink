#include "net_config.h"

#include <stdio.h>
#include <string.h>

#include "build_info.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define NETCFG_MAGIC 0x4D4C4E43u  // MLNC
#define NETCFG_VERSION 1u
#define NETCFG_DEFAULT_IP 0xC0A80701u      // 192.168.7.1
#define NETCFG_DEFAULT_DHCP_START 0xC0A80702u
#define NETCFG_DEFAULT_DHCP_END 0xC0A80720u

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved0;
  uint32_t fw_tag;
  uint32_t device_ip;
  uint32_t dhcp_start;
  uint32_t dhcp_end;
  uint32_t reserved1;
  uint32_t crc32;
} netcfg_flash_t;

static net_config_t g_cfg = {
    .device_ip = NETCFG_DEFAULT_IP,
    .dhcp_start = NETCFG_DEFAULT_DHCP_START,
    .dhcp_end = NETCFG_DEFAULT_DHCP_END,
};

static uint32_t fnv1a32(const uint8_t *p, size_t n) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < n; i++) {
    h ^= p[i];
    h *= 16777619u;
  }
  return h;
}

static uint32_t fw_tag_now(void) {
  char s[64];
  snprintf(s, sizeof(s), "%s %s", BROWSERIO_BUILD_DATE, BROWSERIO_BUILD_TIME);
  return fnv1a32((const uint8_t *)s, strlen(s));
}

static uint32_t cfg_crc(const netcfg_flash_t *f) {
  return fnv1a32((const uint8_t *)f, offsetof(netcfg_flash_t, crc32));
}

static inline const netcfg_flash_t *flash_cfg_ptr(void) {
  return (const netcfg_flash_t *)(XIP_BASE + (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE));
}

static bool same_subnet24(uint32_t a, uint32_t b) {
  return (a & 0xFFFFFF00u) == (b & 0xFFFFFF00u);
}

static bool host_octet_valid(uint32_t ip) {
  uint32_t h = ip & 0xFFu;
  return h >= 1u && h <= 254u;
}

static bool validate_cfg(const net_config_t *cfg, char *err, size_t err_sz) {
  if (!cfg) return false;
  if (!host_octet_valid(cfg->device_ip)) {
    snprintf(err, err_sz, "device IP host must be 1..254");
    return false;
  }
  if (!host_octet_valid(cfg->dhcp_start) || !host_octet_valid(cfg->dhcp_end)) {
    snprintf(err, err_sz, "DHCP range host must be 1..254");
    return false;
  }
  if (!same_subnet24(cfg->device_ip, cfg->dhcp_start) || !same_subnet24(cfg->device_ip, cfg->dhcp_end)) {
    snprintf(err, err_sz, "DHCP range must be in same /24 as device IP");
    return false;
  }
  if (cfg->dhcp_start > cfg->dhcp_end) {
    snprintf(err, err_sz, "DHCP start must be <= end");
    return false;
  }
  if (cfg->device_ip >= cfg->dhcp_start && cfg->device_ip <= cfg->dhcp_end) {
    snprintf(err, err_sz, "device IP cannot be inside DHCP range");
    return false;
  }
  return true;
}

static bool save_cfg(const net_config_t *cfg, char *err, size_t err_sz) {
  netcfg_flash_t f = {
      .magic = NETCFG_MAGIC,
      .version = NETCFG_VERSION,
      .reserved0 = 0,
      .fw_tag = fw_tag_now(),
      .device_ip = cfg->device_ip,
      .dhcp_start = cfg->dhcp_start,
      .dhcp_end = cfg->dhcp_end,
      .reserved1 = 0,
      .crc32 = 0,
  };
  uint8_t page[FLASH_PAGE_SIZE];
  uint32_t irq;
  const uint32_t off = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
  (void)err;
  (void)err_sz;

  f.crc32 = cfg_crc(&f);
  memset(page, 0xFF, sizeof(page));
  memcpy(page, &f, sizeof(f));
  irq = save_and_disable_interrupts();
  flash_range_erase(off, FLASH_SECTOR_SIZE);
  flash_range_program(off, page, FLASH_PAGE_SIZE);
  restore_interrupts(irq);
  return true;
}

void net_config_init(void) {
  char err[96];
  const netcfg_flash_t *f = flash_cfg_ptr();
  bool ok = false;

  if (f->magic == NETCFG_MAGIC && f->version == NETCFG_VERSION && f->fw_tag == fw_tag_now() && f->crc32 == cfg_crc(f)) {
    net_config_t c = {.device_ip = f->device_ip, .dhcp_start = f->dhcp_start, .dhcp_end = f->dhcp_end};
    if (validate_cfg(&c, err, sizeof(err))) {
      g_cfg = c;
      ok = true;
    }
  }

  if (!ok) {
    g_cfg.device_ip = NETCFG_DEFAULT_IP;
    g_cfg.dhcp_start = NETCFG_DEFAULT_DHCP_START;
    g_cfg.dhcp_end = NETCFG_DEFAULT_DHCP_END;
  }
}

void net_config_get(net_config_t *out) {
  if (!out) return;
  *out = g_cfg;
}

bool net_config_set_and_save(const net_config_t *cfg, char *err, size_t err_sz) {
  if (!validate_cfg(cfg, err, err_sz)) return false;
  g_cfg = *cfg;
  return save_cfg(cfg, err, err_sz);
}

bool net_parse_ipv4(const char *s, uint32_t *out_host_order) {
  unsigned a, b, c, d;
  if (!s || !out_host_order) return false;
  if (sscanf(s, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return false;
  if (a > 255u || b > 255u || c > 255u || d > 255u) return false;
  *out_host_order = ((a & 0xFFu) << 24) | ((b & 0xFFu) << 16) | ((c & 0xFFu) << 8) | (d & 0xFFu);
  return true;
}

void net_format_ipv4(uint32_t ip_host_order, char *out, size_t out_sz) {
  if (!out || out_sz == 0) return;
  snprintf(out, out_sz, "%u.%u.%u.%u", (unsigned)((ip_host_order >> 24) & 0xFFu), (unsigned)((ip_host_order >> 16) & 0xFFu),
           (unsigned)((ip_host_order >> 8) & 0xFFu), (unsigned)(ip_host_order & 0xFFu));
}

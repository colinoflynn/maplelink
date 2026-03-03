#include "proto_spi.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_transport.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "spi_sniff_byte_rx.pio.h"

#define SPI_DEV spi0
#define SPI_PORT_NAME "spi0"

// UART0 uses GPIO0/1 in proto_uart.c. Keep SPI away from those.
#define SPI_MISO_PIN 16
#define SPI_CS_PIN 17
#define SPI_SCK_PIN 18
#define SPI_MOSI_PIN 19

#define SPI_MIN_SPEED_HZ 10000u
#define SPI_MAX_SPEED_HZ 16000000u
#define SPI_MIN_IDPOLL_MS 50u
#define SPI_MAX_IDPOLL_MS 5000u
#define SPI_MAX_DUMP_CHUNK 4096u
#define SPI_MAX_SNIFF_BYTES 512u
#define SPI_DUMP_WS_DATA_BYTES 256u
#define SPI_DUMP_WS_INTERVAL_MS 2u
#define SPI_DUMP_FF_SCAN_MAX_BYTES 1024u
#define SPI_DUMP_VERIFY_MAX_BYTES 128u
#define SPI_DUMP_TX_FAIL_LIMIT 40u
#define SPI_RAW_MAX_BYTES 256u
#define WS_BIN_MAGIC 0xB0u
#define WS_CH_SPI_DUMP_DATA 0x02u
#define WS_CH_SPI_DUMP_FF 0x03u

typedef struct {
  bool tristate_default;
  bool pullups_enabled;
  bool detect_enabled;
  bool sniffer_enabled;
  bool idpoll_enabled;
  bool idpoll_monitor_between;
  bool dump_enabled;
  uint32_t speed_hz;
  uint32_t idpoll_interval_ms;
  uint8_t dump_addr_bytes;
  uint8_t dump_read_cmd;
  bool dump_double_read;
  uint8_t dump_verify_retries;
  bool dump_ff_opt;
  uint32_t dump_start_addr;
  bool dump_to_file;
  uint32_t dump_total_bytes;
  uint32_t dump_chunk_bytes;
  uint32_t dump_sent_bytes;
  uint32_t dump_verify_ok_count;
  uint32_t dump_tx_fail_streak;
  uint32_t last_detect_ms;
  uint32_t last_idpoll_ms;
  uint32_t last_dump_ms;
  uint32_t tr_mosi;
  uint32_t tr_miso;
  uint32_t tr_cs;
  uint32_t tr_clk;
  uint32_t base_tr_mosi;
  uint32_t base_tr_miso;
  uint32_t base_tr_cs;
  uint32_t base_tr_clk;
  uint8_t lv_mosi;
  uint8_t lv_miso;
  uint8_t lv_cs;
  uint8_t lv_clk;
  bool spi_active;
  bool sniff_pio_ready;
  bool sniff_dma_ready;
  bool sniff_dma_active;
  PIO sniff_pio;
  uint sniff_pio_off;
  uint sniff_sm_mosi;
  uint sniff_sm_miso;
  int sniff_dma_ch_mosi;
  int sniff_dma_ch_miso;
  bool sniff_frame_active;
  uint8_t sniff_prev_cs;
  uint16_t sniff_len;
  uint16_t sniff_dropped_frame;
  uint32_t sniff_dropped_total;
  uint8_t sniff_tx[SPI_MAX_SNIFF_BYTES];
  uint8_t sniff_rx[SPI_MAX_SNIFF_BYTES];
  uint8_t dump_buf[SPI_MAX_DUMP_CHUNK];
  uint8_t dump_chk[SPI_MAX_DUMP_CHUNK];
} spi_state_t;

static spi_state_t g_spi = {
    .tristate_default = true,
    .pullups_enabled = true,
    .detect_enabled = false,
    .sniffer_enabled = false,
    .idpoll_enabled = false,
    .idpoll_monitor_between = false,
    .dump_enabled = false,
    .speed_hz = 1000000,
    .idpoll_interval_ms = 250,
    .dump_addr_bytes = 3,
    .dump_read_cmd = 0x03,
    .dump_double_read = false,
    .dump_verify_retries = 0,
    .dump_ff_opt = true,
    .dump_start_addr = 0,
    .dump_to_file = true,
    .dump_total_bytes = 0,
    .dump_chunk_bytes = 256,
    .dump_sent_bytes = 0,
};

static bool starts_with(const char *s, const char *prefix) {
  size_t n = strlen(prefix);
  return strncmp(s, prefix, n) == 0;
}

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }
static void spi_dbg(uint8_t level, const char *msg) { (void)app_debug_log(level, "spi", msg); }
static void wr_le16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFFu); p[1] = (uint8_t)((v >> 8) & 0xFFu); }
static void wr_le32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}
static bool send_dump_data_bin(uint32_t offset, const uint8_t *data, uint16_t count) {
  uint8_t fr[8u + SPI_DUMP_WS_DATA_BYTES];
  if (count == 0 || count > SPI_DUMP_WS_DATA_BYTES) return false;
  fr[0] = WS_BIN_MAGIC;
  fr[1] = WS_CH_SPI_DUMP_DATA;
  wr_le32(&fr[2], offset);
  wr_le16(&fr[6], count);
  memcpy(&fr[8], data, count);
  return app_send_binary(fr, (uint16_t)(8u + count));
}
static bool send_dump_ff_bin(uint32_t offset, uint16_t count) {
  uint8_t fr[8];
  if (count == 0) return false;
  fr[0] = WS_BIN_MAGIC;
  fr[1] = WS_CH_SPI_DUMP_FF;
  wr_le32(&fr[2], offset);
  wr_le16(&fr[6], count);
  return app_send_binary(fr, (uint16_t)sizeof(fr));
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void sniff_pio_stop(void);
static void sniff_dma_stop(void);

static void pin_input(uint pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_disable_pulls(pin);
  if (g_spi.pullups_enabled) gpio_pull_up(pin);
}

static void spi_apply_safe_io(void) {
  // All lines as inputs unless an operation takes ownership.
  pin_input(SPI_MOSI_PIN);
  pin_input(SPI_MISO_PIN);
  pin_input(SPI_CS_PIN);
  pin_input(SPI_SCK_PIN);
}

static void spi_activate_master(void) {
  spi_init(SPI_DEV, g_spi.speed_hz);
  spi_set_format(SPI_DEV, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
  gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_init(SPI_CS_PIN);
  gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
  gpio_put(SPI_CS_PIN, 1);
  g_spi.spi_active = true;
}

static void spi_release_master(void) {
  if (!g_spi.spi_active) return;
  spi_deinit(SPI_DEV);
  g_spi.spi_active = false;
  if (g_spi.tristate_default) spi_apply_safe_io();
}

static void spi_enter_passive_mode(void) {
  spi_release_master();
  spi_apply_safe_io();
}

static bool spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len) {
  if (len == 0) return true;
  if (!g_spi.spi_active) spi_activate_master();
  gpio_put(SPI_CS_PIN, 0);
  int got = spi_write_read_blocking(SPI_DEV, tx, rx, (size_t)len);
  gpio_put(SPI_CS_PIN, 1);
  if ((size_t)got != len) return false;
  return true;
}

static bool json_extract_u32(const char *json, const char *key, uint32_t *out) {
  char pattern[40];
  const char *p;
  unsigned v;

  snprintf(pattern, sizeof(pattern), "\"%s\":", key);
  p = strstr(json, pattern);
  if (!p) return false;
  p += strlen(pattern);
  if (sscanf(p, "%u", &v) != 1) return false;
  *out = v;
  return true;
}

static bool json_extract_bool(const char *json, const char *key, bool *out) {
  char pattern[40];
  const char *p;

  snprintf(pattern, sizeof(pattern), "\"%s\":", key);
  p = strstr(json, pattern);
  if (!p) return false;
  p += strlen(pattern);

  if (strncmp(p, "true", 4) == 0) {
    *out = true;
    return true;
  }
  if (strncmp(p, "false", 5) == 0) {
    *out = false;
    return true;
  }
  return false;
}

static bool json_extract_string(const char *json, const char *key, char *out, size_t out_sz) {
  char pattern[40];
  const char *p;
  const char *e;
  size_t n;

  if (!json || !key || !out || out_sz < 2u) return false;
  snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
  p = strstr(json, pattern);
  if (!p) return false;
  p += strlen(pattern);
  e = strchr(p, '"');
  if (!e) return false;
  n = (size_t)(e - p);
  if (n == 0u || n >= out_sz) return false;
  memcpy(out, p, n);
  out[n] = 0;
  return true;
}

static int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return (int)(c - '0');
  if (c >= 'a' && c <= 'f') return (int)(c - 'a' + 10);
  if (c >= 'A' && c <= 'F') return (int)(c - 'A' + 10);
  return -1;
}

static bool parse_hex_flexible(const char *in, uint8_t *out, uint32_t out_cap, uint32_t *out_len) {
  int hi = -1;
  uint32_t n = 0;
  if (!in || !out || !out_len) return false;
  for (const char *p = in; *p; p++) {
    int v = hex_nibble(*p);
    if (v >= 0) {
      if (hi < 0) hi = v;
      else {
        if (n >= out_cap) return false;
        out[n++] = (uint8_t)((hi << 4) | v);
        hi = -1;
      }
    } else if (*p == 'x' || *p == 'X') {
      // optional 0x marker: if we saw only leading '0', drop it
      if (hi == 0) hi = -1;
    } else if (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == ',') {
      // separators
    } else {
      return false;
    }
  }
  if (hi >= 0) return false;
  *out_len = n;
  return true;
}

static void bytes_to_hex(const uint8_t *in, size_t n, char *out, size_t out_len) {
  size_t used = 0;
  if (out_len == 0) return;
  out[0] = '\0';
  for (size_t i = 0; i < n; i++) {
    int w = snprintf(out + used, out_len - used, i == 0 ? "%02X" : " %02X", in[i]);
    if (w <= 0 || (size_t)w >= out_len - used) break;
    used += (size_t)w;
  }
}

static bool is_all_ff(const uint8_t *buf, uint32_t n) {
  for (uint32_t i = 0; i < n; i++) {
    if (buf[i] != 0xFFu) return false;
  }
  return true;
}

static bool read_flash_region(uint32_t addr, uint8_t read_cmd, uint8_t addr_bytes, uint8_t *dst, uint32_t len) {
  uint8_t hdr[5];
  if (len == 0 || len > SPI_MAX_DUMP_CHUNK) return false;

  memset(hdr, 0, sizeof(hdr));
  hdr[0] = read_cmd;
  for (uint8_t i = 0; i < addr_bytes; i++) {
    hdr[1 + i] = (uint8_t)(addr >> (8u * (addr_bytes - 1u - i)));
  }

  if (!g_spi.spi_active) spi_activate_master();
  gpio_put(SPI_CS_PIN, 0);
  if (spi_write_blocking(SPI_DEV, hdr, (size_t)(addr_bytes + 1u)) != (int)(addr_bytes + 1u)) {
    gpio_put(SPI_CS_PIN, 1);
    return false;
  }
  if (spi_read_blocking(SPI_DEV, 0x00u, dst, (size_t)len) != (int)len) {
    gpio_put(SPI_CS_PIN, 1);
    return false;
  }
  gpio_put(SPI_CS_PIN, 1);
  return true;
}

static bool read_sfdp_region(uint32_t addr, uint8_t *dst, uint32_t len) {
  uint8_t hdr[5];
  if (len == 0 || len > 256u) return false;
  hdr[0] = 0x5Au;
  hdr[1] = (uint8_t)((addr >> 16) & 0xFFu);
  hdr[2] = (uint8_t)((addr >> 8) & 0xFFu);
  hdr[3] = (uint8_t)(addr & 0xFFu);
  hdr[4] = 0x00u;  // dummy
  if (!g_spi.spi_active) spi_activate_master();
  gpio_put(SPI_CS_PIN, 0);
  if (spi_write_blocking(SPI_DEV, hdr, 5u) != 5) {
    gpio_put(SPI_CS_PIN, 1);
    return false;
  }
  if (spi_read_blocking(SPI_DEV, 0x00u, dst, len) != (int)len) {
    gpio_put(SPI_CS_PIN, 1);
    return false;
  }
  gpio_put(SPI_CS_PIN, 1);
  return true;
}

static void handle_status_read(const char *json) {
  (void)json;
  uint8_t tx1[2] = {0x05u, 0x00u};
  uint8_t rx1[2] = {0};
  uint8_t tx2[2] = {0x35u, 0x00u};
  uint8_t rx2[2] = {0};
  uint8_t tx3[2] = {0x15u, 0x00u};
  uint8_t rx3[2] = {0};
  bool ok1;
  bool ok2;
  bool ok3;
  uint8_t sr1;
  uint8_t sr2;
  uint8_t sr3;
  char out[480];

  if (g_spi.dump_enabled || g_spi.sniffer_enabled) {
    (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_BUSY\",\"msg\":\"status read blocked while dump/sniffer active\"}");
    return;
  }

  ok1 = spi_xfer(tx1, rx1, sizeof(tx1));
  ok2 = spi_xfer(tx2, rx2, sizeof(tx2));
  ok3 = spi_xfer(tx3, rx3, sizeof(tx3));
  sr1 = rx1[1];
  sr2 = rx2[1];
  sr3 = rx3[1];

  if (!ok1) {
    (void)app_send_text("{\"type\":\"spi.status.result\",\"ok\":false,\"msg\":\"read failed\"}");
    if (g_spi.tristate_default) spi_release_master();
    return;
  }

  snprintf(out, sizeof(out),
           "{\"type\":\"spi.status.result\",\"ok\":true,"
           "\"sr1\":%u,\"sr2\":%u,\"sr3\":%u,\"sr2_ok\":%s,\"sr3_ok\":%s,"
           "\"wip\":%s,\"wel\":%s,\"bp\":%u,\"tb\":%s,\"sec\":%s,\"srp0\":%s,\"qe\":%s}",
           (unsigned)sr1, (unsigned)sr2, (unsigned)sr3,
           ok2 ? "true" : "false", ok3 ? "true" : "false",
           (sr1 & 0x01u) ? "true" : "false",
           (sr1 & 0x02u) ? "true" : "false",
           (unsigned)((sr1 >> 2u) & 0x07u),
           (sr1 & 0x20u) ? "true" : "false",
           (sr1 & 0x40u) ? "true" : "false",
           (sr1 & 0x80u) ? "true" : "false",
           (sr2 & 0x02u) ? "true" : "false");
  (void)app_send_text(out);
  if (g_spi.tristate_default) spi_release_master();
}

static void handle_raw_txrx(const char *json) {
  uint8_t tx[SPI_RAW_MAX_BYTES];
  uint8_t rx[SPI_RAW_MAX_BYTES];
  char tx_hex_in[3 * SPI_RAW_MAX_BYTES + 8];
  char tx_hex_out[3 * SPI_RAW_MAX_BYTES];
  char rx_hex_out[3 * SPI_RAW_MAX_BYTES];
  char out[3 * SPI_RAW_MAX_BYTES * 2 + 160];
  uint32_t tx_len = 0;
  uint32_t ff_tail = 0;
  uint32_t speed_hz = 0;
  uint32_t total;
  bool ok;

  if (g_spi.dump_enabled || g_spi.idpoll_enabled || g_spi.sniffer_enabled) {
    (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_BUSY\",\"msg\":\"raw txrx blocked while dump/idpoll/sniffer active\"}");
    return;
  }

  if (!json_extract_string(json, "tx_hex", tx_hex_in, sizeof(tx_hex_in))) {
    (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_BAD_TX\",\"msg\":\"missing tx_hex\"}");
    return;
  }
  if (!parse_hex_flexible(tx_hex_in, tx, SPI_RAW_MAX_BYTES, &tx_len) || tx_len == 0u) {
    (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_BAD_TX\",\"msg\":\"invalid hex data\"}");
    return;
  }
  (void)json_extract_u32(json, "ff_tail", &ff_tail);
  if (json_extract_u32(json, "speed_hz", &speed_hz)) {
    g_spi.speed_hz = clamp_u32(speed_hz, SPI_MIN_SPEED_HZ, SPI_MAX_SPEED_HZ);
  }

  total = tx_len + ff_tail;
  if (total == 0u || total > SPI_RAW_MAX_BYTES) {
    (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_TX_TOO_LONG\",\"msg\":\"raw tx exceeds max transaction size\"}");
    return;
  }
  for (uint32_t i = tx_len; i < total; i++) tx[i] = 0xFFu;

  ok = spi_xfer(tx, rx, total);
  if (!ok) {
    (void)app_send_text("{\"type\":\"spi.raw.result\",\"ok\":false,\"msg\":\"transfer failed\"}");
    if (g_spi.tristate_default) spi_release_master();
    return;
  }

  bytes_to_hex(tx, total, tx_hex_out, sizeof(tx_hex_out));
  bytes_to_hex(rx, total, rx_hex_out, sizeof(rx_hex_out));
  snprintf(out, sizeof(out),
           "{\"type\":\"spi.raw.result\",\"ok\":true,\"tx_len\":%lu,\"ff_tail\":%lu,\"tx_hex\":\"%s\",\"rx_hex\":\"%s\"}",
           (unsigned long)total, (unsigned long)ff_tail, tx_hex_out, rx_hex_out);
  (void)app_send_text(out);
  if (g_spi.tristate_default) spi_release_master();
}

static uint32_t rd_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8u) | ((uint32_t)p[2] << 16u) | ((uint32_t)p[3] << 24u);
}

static void handle_sfdp_read(const char *json) {
  uint32_t req_len = 256u;
  uint8_t buf[256];
  uint8_t bfpt_buf[128];
  char raw_hex[3 * 64];
  char sig_txt[5];
  char sig_hex[12];
  char out[1024];
  uint32_t nph = 0;
  uint32_t rev_major = 0;
  uint32_t rev_minor = 0;
  bool bfpt_found = false;
  uint32_t bfpt_ptr = 0;
  uint32_t bfpt_len_dw = 0;
  uint32_t bfpt_major = 0;
  uint32_t bfpt_minor = 0;
  uint64_t density_bits = 0;
  uint64_t density_bytes = 0;
  uint32_t density_raw = 0;
  bool density_ok = false;
  const char *addr_mode = "unknown";
  uint32_t read_cmd_3b = 0x03u;
  bool read4b_cmd_known = false;
  uint32_t read_cmd_4b = 0u;

  if (json_extract_u32(json, "length", &req_len)) {
    if (req_len < 16u) req_len = 16u;
    if (req_len > 256u) req_len = 256u;
  }

  if (g_spi.dump_enabled || g_spi.sniffer_enabled) {
    (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_BUSY\",\"msg\":\"sfdp read blocked while dump/sniffer active\"}");
    return;
  }

  if (!read_sfdp_region(0u, buf, req_len)) {
    (void)app_send_text("{\"type\":\"spi.sfdp.result\",\"ok\":false,\"msg\":\"read failed\"}");
    if (g_spi.tristate_default) spi_release_master();
    return;
  }

  bytes_to_hex(buf, req_len > 64u ? 64u : req_len, raw_hex, sizeof(raw_hex));
  // Keep signature JSON-safe and UTF-8 safe even for 0xFF/0x00 bytes.
  for (uint32_t i = 0; i < 4u; i++) {
    uint8_t c = buf[i];
    sig_txt[i] = (c >= 32u && c <= 126u) ? (char)c : '.';
  }
  sig_txt[4] = 0;
  snprintf(sig_hex, sizeof(sig_hex), "%02X %02X %02X %02X", buf[0], buf[1], buf[2], buf[3]);
  if (req_len >= 8u && buf[0] == 'S' && buf[1] == 'F' && buf[2] == 'D' && buf[3] == 'P') {
    rev_minor = buf[4];
    rev_major = buf[5];
    nph = (uint32_t)buf[6] + 1u;

    // Parameter headers start at byte 8, each 8 bytes.
    for (uint32_t i = 0; i < nph; i++) {
      uint32_t off = 8u + (i * 8u);
      uint8_t id_lsb;
      uint8_t minr;
      uint8_t majr;
      uint8_t len_dw;
      uint32_t ptp;
      uint8_t id_msb;
      if (off + 8u > req_len) break;
      id_lsb = buf[off + 0u];
      minr = buf[off + 1u];
      majr = buf[off + 2u];
      len_dw = buf[off + 3u];
      ptp = (uint32_t)buf[off + 4u] | ((uint32_t)buf[off + 5u] << 8u) | ((uint32_t)buf[off + 6u] << 16u);
      id_msb = buf[off + 7u];
      // BFPT parameter ID is 0xFF00 (msb:0xFF, lsb:0x00).
      if (id_lsb == 0x00u && id_msb == 0xFFu) {
        bfpt_found = true;
        bfpt_ptr = ptp;
        bfpt_len_dw = len_dw;
        bfpt_major = majr;
        bfpt_minor = minr;
      }
      // 4-Byte Address Instruction Table (4BAIT), when present, can report
      // explicit 4-byte opcodes (JESD216 parameter ID 0xFF84).
      if (id_lsb == 0x84u && id_msb == 0xFFu && len_dw >= 1u && (ptp + 4u) <= req_len) {
        uint8_t cmd = buf[ptp + 0u];
        if (cmd != 0x00u && cmd != 0xFFu) {
          read_cmd_4b = cmd;
          read4b_cmd_known = true;
        }
      }
    }

    if (bfpt_found) {
      uint32_t bfpt_bytes = bfpt_len_dw * 4u;
      if (bfpt_bytes < 8u) bfpt_bytes = 8u;
      if (bfpt_bytes > sizeof(bfpt_buf)) bfpt_bytes = sizeof(bfpt_buf);
      if (read_sfdp_region(bfpt_ptr, bfpt_buf, bfpt_bytes)) {
        uint32_t dword1 = rd_le32(&bfpt_buf[0]);
        uint32_t addr_bits = (dword1 >> 16u) & 0x3u;
        // JESD216 BFPT DWORD1 bits [17:16]
        if (addr_bits == 0u) addr_mode = "3-byte only";
        else if (addr_bits == 1u) addr_mode = "3-byte or 4-byte";
        else if (addr_bits == 2u) addr_mode = "4-byte only";
        else addr_mode = "reserved";

        // BFPT DWORD2 contains density encoding.
        density_raw = rd_le32(&bfpt_buf[4]);
        if ((density_raw & 0x80000000u) == 0u) {
          density_bits = (uint64_t)(density_raw & 0x7FFFFFFFu) + 1ull;
          density_bytes = density_bits / 8ull;
          density_ok = true;
        }
      }
    }
  }

  snprintf(out, sizeof(out),
           "{\"type\":\"spi.sfdp.result\",\"ok\":true,\"len\":%lu,\"sig\":\"%s\",\"sig_hex\":\"%s\","
           "\"rev_major\":%lu,\"rev_minor\":%lu,\"nph\":%lu,"
           "\"bfpt_found\":%s,\"bfpt_ptr\":%lu,\"bfpt_len_dw\":%lu,\"bfpt_rev_major\":%lu,\"bfpt_rev_minor\":%lu,"
           "\"density_ok\":%s,\"density_bits\":%lu,\"density_bytes\":%lu,\"density_raw\":%lu,"
           "\"addr_mode\":\"%s\",\"read_cmd_3b\":%lu,\"read4b_cmd_known\":%s,\"read_cmd_4b\":%lu,"
           "\"raw_hex\":\"%s\"}",
           (unsigned long)req_len, sig_txt, sig_hex, (unsigned long)rev_major,
           (unsigned long)rev_minor, (unsigned long)nph, bfpt_found ? "true" : "false",
           (unsigned long)bfpt_ptr, (unsigned long)bfpt_len_dw, (unsigned long)bfpt_major,
            (unsigned long)bfpt_minor, density_ok ? "true" : "false",
           (unsigned long)density_bits, (unsigned long)density_bytes, (unsigned long)density_raw,
           addr_mode, (unsigned long)read_cmd_3b, read4b_cmd_known ? "true" : "false",
           (unsigned long)read_cmd_4b, raw_hex);
  (void)app_send_text(out);
  if (g_spi.tristate_default) spi_release_master();
}

static void send_spi_state(void) {
  char out[768];
  snprintf(out, sizeof(out),
           "{\"type\":\"spi.config\",\"port\":\"" SPI_PORT_NAME "\",\"status\":\"hw\","
           "\"pins\":{\"mosi\":%u,\"miso\":%u,\"cs\":%u,\"clk\":%u},"
           "\"tristate_default\":%s,\"pullups_enabled\":%s,"
           "\"speed_hz\":%lu,\"idpoll_interval_ms\":%lu,"
           "\"detect_enabled\":%s,\"sniffer_enabled\":%s,\"idpoll_enabled\":%s,\"idpoll_monitor_between\":%s,\"dump_enabled\":%s,"
           "\"dump\":{\"addr_bytes\":%u,\"read_cmd\":%u,\"double_read\":%s,\"verify_retries\":%u,"
           "\"ff_opt\":%s,\"start_addr\":%lu,\"to_file\":%s}}",
           (unsigned)SPI_MOSI_PIN, (unsigned)SPI_MISO_PIN, (unsigned)SPI_CS_PIN, (unsigned)SPI_SCK_PIN,
           g_spi.tristate_default ? "true" : "false", g_spi.pullups_enabled ? "true" : "false",
           (unsigned long)g_spi.speed_hz,
           (unsigned long)g_spi.idpoll_interval_ms, g_spi.detect_enabled ? "true" : "false",
           g_spi.sniffer_enabled ? "true" : "false", g_spi.idpoll_enabled ? "true" : "false",
           g_spi.idpoll_monitor_between ? "true" : "false",
           g_spi.dump_enabled ? "true" : "false", g_spi.dump_addr_bytes, g_spi.dump_read_cmd,
           g_spi.dump_double_read ? "true" : "false", g_spi.dump_verify_retries,
           g_spi.dump_ff_opt ? "true" : "false", (unsigned long)g_spi.dump_start_addr,
           g_spi.dump_to_file ? "true" : "false");
  (void)app_send_text(out);
}

static void send_spi_status(const char *state, const char *detail) {
  char out[256];
  snprintf(out, sizeof(out), "{\"type\":\"spi.dump.status\",\"port\":\"" SPI_PORT_NAME "\",\"state\":\"%s\",\"detail\":\"%s\"}",
           state, detail ? detail : "");
  (void)app_send_text(out);
}

static void ensure_safe_exclusion(void) {
  if (g_spi.dump_enabled || g_spi.idpoll_enabled) {
    g_spi.sniffer_enabled = false;
    sniff_pio_stop();
  }
}

static void poll_pin_levels_and_transitions(void) {
  uint8_t v;

  v = (uint8_t)gpio_get(SPI_MOSI_PIN);
  if (v != g_spi.lv_mosi) g_spi.tr_mosi++;
  g_spi.lv_mosi = v;

  v = (uint8_t)gpio_get(SPI_MISO_PIN);
  if (v != g_spi.lv_miso) g_spi.tr_miso++;
  g_spi.lv_miso = v;

  v = (uint8_t)gpio_get(SPI_CS_PIN);
  if (v != g_spi.lv_cs) g_spi.tr_cs++;
  g_spi.lv_cs = v;

  v = (uint8_t)gpio_get(SPI_SCK_PIN);
  if (v != g_spi.lv_clk) g_spi.tr_clk++;
  g_spi.lv_clk = v;
}

static void send_pin_snapshot(void) {
  char out[384];
  snprintf(out, sizeof(out),
           "{\"type\":\"spi.pins\",\"mosi\":%u,\"miso\":%u,\"cs\":%u,\"clk\":%u,"
           "\"transitions\":{\"mosi\":%lu,\"miso\":%lu,\"cs\":%lu,\"clk\":%lu}}",
           (unsigned)g_spi.lv_mosi, (unsigned)g_spi.lv_miso, (unsigned)g_spi.lv_cs, (unsigned)g_spi.lv_clk,
           (unsigned long)g_spi.tr_mosi, (unsigned long)g_spi.tr_miso, (unsigned long)g_spi.tr_cs,
           (unsigned long)g_spi.tr_clk);
  (void)app_send_text(out);
}

static void snapshot_pin_levels_no_count(void) {
  g_spi.lv_mosi = (uint8_t)gpio_get(SPI_MOSI_PIN);
  g_spi.lv_miso = (uint8_t)gpio_get(SPI_MISO_PIN);
  g_spi.lv_cs = (uint8_t)gpio_get(SPI_CS_PIN);
  g_spi.lv_clk = (uint8_t)gpio_get(SPI_SCK_PIN);
}

static void set_idpoll_baseline(void) {
  g_spi.base_tr_mosi = g_spi.tr_mosi;
  g_spi.base_tr_miso = g_spi.tr_miso;
  g_spi.base_tr_cs = g_spi.tr_cs;
  g_spi.base_tr_clk = g_spi.tr_clk;
}

static void sniff_emit_frame(void) {
  char tx_hex[3 * 64];
  char rx_hex[3 * 64];
  char out[760];
  uint16_t shown = g_spi.sniff_len > 64 ? 64 : g_spi.sniff_len;
  bytes_to_hex(g_spi.sniff_tx, shown, tx_hex, sizeof(tx_hex));
  bytes_to_hex(g_spi.sniff_rx, shown, rx_hex, sizeof(rx_hex));
  snprintf(out, sizeof(out),
           "{\"type\":\"spi.sniffer.frame\",\"cs_frame\":1,\"tx_hex\":\"%s\",\"rx_hex\":\"%s\","
           "\"bytes_captured\":%u,\"bytes_total\":%u,\"dropped\":%u,\"overflow\":%s}",
           tx_hex, rx_hex, (unsigned)g_spi.sniff_len, (unsigned)(g_spi.sniff_len + g_spi.sniff_dropped_frame),
           (unsigned)g_spi.sniff_dropped_frame, g_spi.sniff_dropped_frame ? "true" : "false");
  (void)app_send_text(out);
}

static bool sniff_pio_init(void) {
  PIO pio_try[2] = {pio1, pio0};
  size_t i;

  if (g_spi.sniff_pio_ready) return true;

  for (i = 0; i < (sizeof(pio_try) / sizeof(pio_try[0])); i++) {
    PIO pio = pio_try[i];
    int sm_mosi;
    int sm_miso;
    pio_sm_config c;

    if (!pio_can_add_program(pio, &spi_sniff_byte_rx_program)) continue;

    sm_mosi = pio_claim_unused_sm(pio, false);
    if (sm_mosi < 0) continue;
    sm_miso = pio_claim_unused_sm(pio, false);
    if (sm_miso < 0) {
      pio_sm_unclaim(pio, (uint)sm_mosi);
      continue;
    }

    g_spi.sniff_pio = pio;
    g_spi.sniff_sm_mosi = (uint)sm_mosi;
    g_spi.sniff_sm_miso = (uint)sm_miso;
    g_spi.sniff_pio_off = pio_add_program(pio, &spi_sniff_byte_rx_program);

    pio_gpio_init(pio, SPI_MOSI_PIN);
    pio_gpio_init(pio, SPI_MISO_PIN);
    pio_gpio_init(pio, SPI_CS_PIN);
    pio_gpio_init(pio, SPI_SCK_PIN);

    c = spi_sniff_byte_rx_program_get_default_config(g_spi.sniff_pio_off);
    sm_config_set_jmp_pin(&c, SPI_CS_PIN);
    sm_config_set_in_shift(&c, false, true, 8);
    // Sniffer is RX-only; join FIFO for deeper buffering.
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&c, 1.0f);

    sm_config_set_in_pins(&c, SPI_MOSI_PIN);
    pio_sm_set_consecutive_pindirs(pio, g_spi.sniff_sm_mosi, SPI_MOSI_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, g_spi.sniff_sm_mosi, SPI_CS_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, g_spi.sniff_sm_mosi, SPI_SCK_PIN, 1, false);
    pio_sm_init(pio, g_spi.sniff_sm_mosi, g_spi.sniff_pio_off, &c);

    sm_config_set_in_pins(&c, SPI_MISO_PIN);
    pio_sm_set_consecutive_pindirs(pio, g_spi.sniff_sm_miso, SPI_MISO_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, g_spi.sniff_sm_miso, SPI_CS_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, g_spi.sniff_sm_miso, SPI_SCK_PIN, 1, false);
    pio_sm_init(pio, g_spi.sniff_sm_miso, g_spi.sniff_pio_off, &c);

    pio_sm_set_enabled(pio, g_spi.sniff_sm_mosi, false);
    pio_sm_set_enabled(pio, g_spi.sniff_sm_miso, false);
    pio_sm_clear_fifos(pio, g_spi.sniff_sm_mosi);
    pio_sm_clear_fifos(pio, g_spi.sniff_sm_miso);
    pio_sm_restart(pio, g_spi.sniff_sm_mosi);
    pio_sm_restart(pio, g_spi.sniff_sm_miso);

    g_spi.sniff_pio_ready = true;
    if (app_debug_level() >= 2u) {
      char m[96];
      snprintf(m, sizeof(m), "sniffer pio init ok (%s sm=%u/%u off=%u)",
               (pio == pio0) ? "pio0" : "pio1",
               (unsigned)g_spi.sniff_sm_mosi, (unsigned)g_spi.sniff_sm_miso,
               (unsigned)g_spi.sniff_pio_off);
      spi_dbg(2, m);
    }
    return true;
  }

  g_spi.sniff_pio_ready = false;
  if (app_debug_level() >= 1u) spi_dbg(1, "sniffer pio init failed");
  return false;
}

static bool sniff_dma_init(void) {
  if (g_spi.sniff_dma_ready) return true;
  g_spi.sniff_dma_ch_mosi = dma_claim_unused_channel(false);
  if (g_spi.sniff_dma_ch_mosi < 0) return false;
  g_spi.sniff_dma_ch_miso = dma_claim_unused_channel(false);
  if (g_spi.sniff_dma_ch_miso < 0) {
    dma_channel_unclaim((uint)g_spi.sniff_dma_ch_mosi);
    g_spi.sniff_dma_ch_mosi = -1;
    return false;
  }
  g_spi.sniff_dma_ready = true;
  if (app_debug_level() >= 2u) {
    char m[96];
    snprintf(m, sizeof(m), "sniffer dma init ok (ch=%d/%d)",
             g_spi.sniff_dma_ch_mosi, g_spi.sniff_dma_ch_miso);
    spi_dbg(2, m);
  }
  return true;
}

static void sniff_dma_start_frame(void) {
  dma_channel_config c_mosi;
  dma_channel_config c_miso;

  if (!g_spi.sniff_dma_ready) return;

  dma_channel_abort((uint)g_spi.sniff_dma_ch_mosi);
  dma_channel_abort((uint)g_spi.sniff_dma_ch_miso);

  c_mosi = dma_channel_get_default_config((uint)g_spi.sniff_dma_ch_mosi);
  channel_config_set_transfer_data_size(&c_mosi, DMA_SIZE_8);
  channel_config_set_read_increment(&c_mosi, false);
  channel_config_set_write_increment(&c_mosi, true);
  channel_config_set_dreq(&c_mosi, pio_get_dreq(g_spi.sniff_pio, g_spi.sniff_sm_mosi, false));
  dma_channel_configure((uint)g_spi.sniff_dma_ch_mosi, &c_mosi,
                        g_spi.sniff_tx, &g_spi.sniff_pio->rxf[g_spi.sniff_sm_mosi],
                        SPI_MAX_SNIFF_BYTES, false);

  c_miso = dma_channel_get_default_config((uint)g_spi.sniff_dma_ch_miso);
  channel_config_set_transfer_data_size(&c_miso, DMA_SIZE_8);
  channel_config_set_read_increment(&c_miso, false);
  channel_config_set_write_increment(&c_miso, true);
  channel_config_set_dreq(&c_miso, pio_get_dreq(g_spi.sniff_pio, g_spi.sniff_sm_miso, false));
  dma_channel_configure((uint)g_spi.sniff_dma_ch_miso, &c_miso,
                        g_spi.sniff_rx, &g_spi.sniff_pio->rxf[g_spi.sniff_sm_miso],
                        SPI_MAX_SNIFF_BYTES, false);

  dma_start_channel_mask((1u << (uint)g_spi.sniff_dma_ch_mosi) |
                         (1u << (uint)g_spi.sniff_dma_ch_miso));
  g_spi.sniff_dma_active = true;
}

static void sniff_dma_stop(void) {
  if (!g_spi.sniff_dma_ready) return;
  dma_channel_abort((uint)g_spi.sniff_dma_ch_mosi);
  dma_channel_abort((uint)g_spi.sniff_dma_ch_miso);
  g_spi.sniff_dma_active = false;
}

static void sniff_dma_finish_frame(void) {
  uint32_t got_mosi;
  uint32_t got_miso;
  uint32_t got_min;
  uintptr_t wr_mosi;
  uintptr_t wr_miso;
  uintptr_t base_mosi;
  uintptr_t base_miso;

  if (!g_spi.sniff_dma_ready || !g_spi.sniff_dma_active) return;

  dma_channel_abort((uint)g_spi.sniff_dma_ch_mosi);
  dma_channel_abort((uint)g_spi.sniff_dma_ch_miso);
  wr_mosi = (uintptr_t)dma_hw->ch[g_spi.sniff_dma_ch_mosi].write_addr;
  wr_miso = (uintptr_t)dma_hw->ch[g_spi.sniff_dma_ch_miso].write_addr;
  base_mosi = (uintptr_t)g_spi.sniff_tx;
  base_miso = (uintptr_t)g_spi.sniff_rx;
  got_mosi = (wr_mosi > base_mosi) ? (uint32_t)(wr_mosi - base_mosi) : 0u;
  got_miso = (wr_miso > base_miso) ? (uint32_t)(wr_miso - base_miso) : 0u;
  if (got_mosi > SPI_MAX_SNIFF_BYTES) got_mosi = SPI_MAX_SNIFF_BYTES;
  if (got_miso > SPI_MAX_SNIFF_BYTES) got_miso = SPI_MAX_SNIFF_BYTES;
  got_min = got_mosi < got_miso ? got_mosi : got_miso;
  g_spi.sniff_len = (uint16_t)got_min;

  if (got_mosi != got_miso) {
    g_spi.sniff_dropped_frame++;
    g_spi.sniff_dropped_total++;
  }
  if (got_min >= SPI_MAX_SNIFF_BYTES) {
    g_spi.sniff_dropped_frame++;
    g_spi.sniff_dropped_total++;
  }
  g_spi.sniff_dma_active = false;
}

static void sniff_pio_arm(void) {
  uint32_t mask;
  if (!g_spi.sniff_pio_ready) return;
  mask = (1u << g_spi.sniff_sm_mosi) | (1u << g_spi.sniff_sm_miso);
  pio_set_sm_mask_enabled(g_spi.sniff_pio, mask, false);
  pio_sm_clear_fifos(g_spi.sniff_pio, g_spi.sniff_sm_mosi);
  pio_sm_clear_fifos(g_spi.sniff_pio, g_spi.sniff_sm_miso);
  pio_sm_restart(g_spi.sniff_pio, g_spi.sniff_sm_mosi);
  pio_sm_restart(g_spi.sniff_pio, g_spi.sniff_sm_miso);
  // Start both SMs on the same cycle so MOSI/MISO sampling stays aligned.
  pio_enable_sm_mask_in_sync(g_spi.sniff_pio, mask);
}

static void sniff_pio_stop(void) {
  sniff_dma_stop();
  if (!g_spi.sniff_pio_ready) return;
  pio_sm_set_enabled(g_spi.sniff_pio, g_spi.sniff_sm_mosi, false);
  pio_sm_set_enabled(g_spi.sniff_pio, g_spi.sniff_sm_miso, false);
  pio_sm_clear_fifos(g_spi.sniff_pio, g_spi.sniff_sm_mosi);
  pio_sm_clear_fifos(g_spi.sniff_pio, g_spi.sniff_sm_miso);
  pio_sm_restart(g_spi.sniff_pio, g_spi.sniff_sm_mosi);
  pio_sm_restart(g_spi.sniff_pio, g_spi.sniff_sm_miso);
}

static void sniff_pio_drain_pairs(void) {
  while (!pio_sm_is_rx_fifo_empty(g_spi.sniff_pio, g_spi.sniff_sm_mosi) &&
         !pio_sm_is_rx_fifo_empty(g_spi.sniff_pio, g_spi.sniff_sm_miso)) {
    uint8_t tx = (uint8_t)pio_sm_get(g_spi.sniff_pio, g_spi.sniff_sm_mosi);
    uint8_t rx = (uint8_t)pio_sm_get(g_spi.sniff_pio, g_spi.sniff_sm_miso);
    if (g_spi.sniff_len < SPI_MAX_SNIFF_BYTES) {
      g_spi.sniff_tx[g_spi.sniff_len] = tx;
      g_spi.sniff_rx[g_spi.sniff_len] = rx;
      g_spi.sniff_len++;
    } else {
      g_spi.sniff_dropped_frame++;
      g_spi.sniff_dropped_total++;
    }
  }
}

static void sniff_pio_discard_unpaired(void) {
  while (!pio_sm_is_rx_fifo_empty(g_spi.sniff_pio, g_spi.sniff_sm_mosi)) {
    (void)pio_sm_get(g_spi.sniff_pio, g_spi.sniff_sm_mosi);
    g_spi.sniff_dropped_frame++;
    g_spi.sniff_dropped_total++;
  }
  while (!pio_sm_is_rx_fifo_empty(g_spi.sniff_pio, g_spi.sniff_sm_miso)) {
    (void)pio_sm_get(g_spi.sniff_pio, g_spi.sniff_sm_miso);
    g_spi.sniff_dropped_frame++;
    g_spi.sniff_dropped_total++;
  }
}

static void sniff_sample(void) {
  uint8_t cs = (uint8_t)gpio_get(SPI_CS_PIN);
  uint64_t spin_until_us;

  if (cs == 0u && !g_spi.sniff_frame_active) {
    g_spi.sniff_frame_active = true;
    g_spi.sniff_len = 0;
    g_spi.sniff_dropped_frame = 0;
    if (g_spi.sniff_dma_ready) sniff_dma_start_frame();
  }

  if (g_spi.sniff_frame_active && !g_spi.sniff_dma_ready) {
    // In sniffer mode we can spend short bursts draining FIFOs to avoid
    // overflow during high-speed transfers.
    spin_until_us = time_us_64() + 1000u;
    do {
      sniff_pio_drain_pairs();
      cs = (uint8_t)gpio_get(SPI_CS_PIN);
      if (cs != 0u) break;
      tight_loop_contents();
    } while (time_us_64() < spin_until_us);
  }

  if (g_spi.sniff_frame_active && cs == 1u) {
    if (g_spi.sniff_dma_ready) {
      sniff_dma_finish_frame();
    } else {
      sniff_pio_drain_pairs();
      sniff_pio_discard_unpaired();
    }
    sniff_emit_frame();
    g_spi.sniff_frame_active = false;
    // Re-arm while CS is high so next frame starts cleanly from bit 0.
    sniff_pio_arm();
  }

  g_spi.sniff_prev_cs = cs;
}

static void handle_set_io(const char *json) {
  bool b;
  if (json_extract_bool(json, "tristate_default", &b)) g_spi.tristate_default = b;
  if (json_extract_bool(json, "pullups_enabled", &b)) g_spi.pullups_enabled = b;

  if (g_spi.tristate_default) spi_apply_safe_io();
  spi_dbg(2, "set_io applied");
  send_spi_state();
}

static void handle_set_speed(const char *json) {
  uint32_t v;
  if (json_extract_u32(json, "speed_hz", &v)) g_spi.speed_hz = clamp_u32(v, SPI_MIN_SPEED_HZ, SPI_MAX_SPEED_HZ);
  if (json_extract_u32(json, "idpoll_interval_ms", &v)) g_spi.idpoll_interval_ms = clamp_u32(v, SPI_MIN_IDPOLL_MS, SPI_MAX_IDPOLL_MS);
  if (g_spi.spi_active) {
    spi_set_baudrate(SPI_DEV, g_spi.speed_hz);
  }
  spi_dbg(2, "speed settings applied");
  send_spi_state();
}

static void handle_dump_start(const char *json, bool to_file) {
  uint32_t v;
  uint64_t max_bytes;
  uint64_t span;
  bool b;

  if (json_extract_u32(json, "addr_bytes", &v)) g_spi.dump_addr_bytes = (uint8_t)v;
  if (json_extract_u32(json, "read_cmd", &v)) g_spi.dump_read_cmd = (uint8_t)v;
  if (json_extract_u32(json, "length_bytes", &v)) g_spi.dump_total_bytes = v;
  if (json_extract_u32(json, "chunk_bytes", &v)) g_spi.dump_chunk_bytes = v;
  if (json_extract_u32(json, "speed_hz", &v)) g_spi.speed_hz = clamp_u32(v, SPI_MIN_SPEED_HZ, SPI_MAX_SPEED_HZ);
  if (json_extract_u32(json, "start_addr", &v)) g_spi.dump_start_addr = v;
  if (json_extract_bool(json, "double_read", &b)) g_spi.dump_double_read = b;
  if (json_extract_u32(json, "verify_retries", &v)) g_spi.dump_verify_retries = (uint8_t)(v > 20u ? 20u : v);
  if (json_extract_bool(json, "ff_opt", &b)) g_spi.dump_ff_opt = b;

  if (g_spi.dump_addr_bytes < 3u) g_spi.dump_addr_bytes = 3u;
  if (g_spi.dump_addr_bytes > 4u) g_spi.dump_addr_bytes = 4u;
  if (g_spi.dump_total_bytes == 0u) g_spi.dump_total_bytes = 4096u;
  if (g_spi.dump_chunk_bytes == 0u) g_spi.dump_chunk_bytes = 256u;
  if (g_spi.dump_chunk_bytes > SPI_MAX_DUMP_CHUNK) g_spi.dump_chunk_bytes = SPI_MAX_DUMP_CHUNK;
  max_bytes = 1ull << (8u * g_spi.dump_addr_bytes);
  span = max_bytes - (uint64_t)g_spi.dump_start_addr;
  if ((uint64_t)g_spi.dump_total_bytes > span) {
    (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_ADDR_RANGE\",\"msg\":\"length exceeds selected address-byte range\"}");
    send_spi_status("error", "address range exceeded");
    send_spi_state();
    return;
  }

  g_spi.dump_sent_bytes = 0;
  g_spi.dump_verify_ok_count = 0;
  g_spi.dump_tx_fail_streak = 0;
  g_spi.last_dump_ms = now_ms();
  g_spi.dump_enabled = true;
  g_spi.dump_to_file = to_file;
  g_spi.idpoll_enabled = false;
  ensure_safe_exclusion();
  if (app_debug_level() >= 1u) {
    char m[160];
    snprintf(m, sizeof(m),
             "dump start accepted: mode=%s total=%lu chunk=%lu start=0x%08lX",
             to_file ? "dump" : "display",
             (unsigned long)g_spi.dump_total_bytes,
             (unsigned long)g_spi.dump_chunk_bytes,
             (unsigned long)g_spi.dump_start_addr);
    spi_dbg(1, m);
  }
  send_spi_status("running", to_file ? "dump started" : "display started");
  send_spi_state();
}

void proto_spi_init(void) {
  uint32_t now = now_ms();
  g_spi.sniff_dma_ch_mosi = -1;
  g_spi.sniff_dma_ch_miso = -1;
  g_spi.last_detect_ms = now;
  g_spi.last_idpoll_ms = now;
  g_spi.last_dump_ms = now;
  g_spi.lv_mosi = 0;
  g_spi.lv_miso = 0;
  g_spi.lv_cs = 1;
  g_spi.lv_clk = 0;
  g_spi.sniff_prev_cs = 1;
  spi_apply_safe_io();
}

void proto_spi_on_client_open(ws_conn_t *conn) { (void)conn; }

void proto_spi_on_client_close(ws_conn_t *conn) {
  (void)conn;
  g_spi.sniffer_enabled = false;
  g_spi.idpoll_enabled = false;
  g_spi.dump_enabled = false;
  sniff_pio_stop();
  spi_release_master();
}

bool proto_spi_handle_text(const char *type, const char *json) {
  if (!starts_with(type, "spi.")) return false;

  if (strcmp(type, "spi.get_config") == 0) {
    spi_dbg(3, "get_config");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.set_io") == 0) {
    handle_set_io(json);
    return true;
  }

  if (strcmp(type, "spi.set_speed") == 0) {
    handle_set_speed(json);
    return true;
  }

  if (strcmp(type, "spi.detect.start") == 0) {
    g_spi.detect_enabled = true;
    spi_enter_passive_mode();
    spi_dbg(1, "detect started");
    send_spi_status("detect", "enabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.detect.stop") == 0) {
    g_spi.detect_enabled = false;
    spi_dbg(1, "detect stopped");
    send_spi_status("detect", "disabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.sniffer.start") == 0) {
    if (g_spi.dump_enabled || g_spi.idpoll_enabled) {
      (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_BUSY\",\"msg\":\"sniffer blocked while idpoll/dump active\"}");
      spi_dbg(1, "sniffer start blocked (busy)");
      return true;
    }
    if (!sniff_pio_init()) {
      (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_SNIFFER_PIO_UNAVAILABLE\",\"msg\":\"sniffer PIO unavailable\"}");
      spi_dbg(1, "sniffer start failed (pio unavailable)");
      return true;
    }
    if (!sniff_dma_init() && app_debug_level() >= 1u) {
      spi_dbg(1, "sniffer dma unavailable, using cpu fifo drain");
    }
    g_spi.sniffer_enabled = true;
    g_spi.sniff_frame_active = false;
    g_spi.sniff_prev_cs = (uint8_t)gpio_get(SPI_CS_PIN);
    spi_enter_passive_mode();
    sniff_pio_arm();
    spi_dbg(1, "sniffer started");
    send_spi_status("sniffer", "enabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.sniffer.stop") == 0) {
    g_spi.sniffer_enabled = false;
    g_spi.sniff_frame_active = false;
    sniff_pio_stop();
    spi_dbg(1, "sniffer stopped");
    send_spi_status("sniffer", "disabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.idpoll.start") == 0) {
    uint32_t v;
    bool b;
    if (json_extract_u32(json, "interval_ms", &v)) g_spi.idpoll_interval_ms = clamp_u32(v, SPI_MIN_IDPOLL_MS, SPI_MAX_IDPOLL_MS);
    if (json_extract_u32(json, "speed_hz", &v)) g_spi.speed_hz = clamp_u32(v, SPI_MIN_SPEED_HZ, SPI_MAX_SPEED_HZ);
    if (json_extract_bool(json, "monitor_between", &b)) g_spi.idpoll_monitor_between = b;
    g_spi.idpoll_enabled = true;
    g_spi.dump_enabled = false;
    ensure_safe_exclusion();
    snapshot_pin_levels_no_count();
    set_idpoll_baseline();
    spi_dbg(1, "idpoll started");
    send_spi_status("idpoll", "enabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.idpoll.stop") == 0) {
    g_spi.idpoll_enabled = false;
    g_spi.detect_enabled = false;
    spi_release_master();
    spi_dbg(1, "idpoll stopped");
    send_spi_status("idpoll", "disabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.dump.start") == 0) {
    handle_dump_start(json, true);
    return true;
  }

  if (strcmp(type, "spi.display.start") == 0) {
    handle_dump_start(json, false);
    return true;
  }

  if (strcmp(type, "spi.dump.stop") == 0) {
    g_spi.dump_enabled = false;
    spi_release_master();
    spi_dbg(1, "dump stopped by user");
    send_spi_status("stopped", "dump stopped");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.display.stop") == 0) {
    g_spi.dump_enabled = false;
    spi_release_master();
    spi_dbg(1, "display stopped by user");
    send_spi_status("stopped", "display stopped");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.sfdp.read") == 0) {
    handle_sfdp_read(json);
    return true;
  }
  if (strcmp(type, "spi.status.read") == 0) {
    handle_status_read(json);
    return true;
  }
  if (strcmp(type, "spi.raw.txrx") == 0) {
    handle_raw_txrx(json);
    return true;
  }

  return app_send_text("{\"type\":\"error\",\"code\":\"SPI_UNSUPPORTED\",\"msg\":\"spi command not implemented\"}");
}

void proto_spi_poll(void) {
  uint32_t now = now_ms();

  if (g_spi.detect_enabled || (g_spi.idpoll_enabled && g_spi.idpoll_monitor_between)) {
    // Passive modes always release SPI peripheral ownership.
    if (g_spi.spi_active) spi_enter_passive_mode();
    poll_pin_levels_and_transitions();
  }

  if (g_spi.sniffer_enabled) {
    sniff_sample();
  }

  if ((g_spi.detect_enabled || (g_spi.idpoll_enabled && g_spi.idpoll_monitor_between)) && (now - g_spi.last_detect_ms) >= 120u) {
    g_spi.last_detect_ms = now;
    send_pin_snapshot();
  }

  if (g_spi.idpoll_enabled && (now - g_spi.last_idpoll_ms) >= g_spi.idpoll_interval_ms) {
    uint8_t tx[4] = {0x9Fu, 0x00u, 0x00u, 0x00u};
    uint8_t rx[4] = {0};
    char out[160];
    uint32_t d_mosi = g_spi.tr_mosi - g_spi.base_tr_mosi;
    uint32_t d_miso = g_spi.tr_miso - g_spi.base_tr_miso;
    uint32_t d_cs = g_spi.tr_cs - g_spi.base_tr_cs;
    uint32_t d_clk = g_spi.tr_clk - g_spi.base_tr_clk;
    uint32_t total = d_mosi + d_miso + d_cs + d_clk;
    g_spi.last_idpoll_ms = now;

    if (g_spi.idpoll_monitor_between) {
      char win[320];
      snprintf(win, sizeof(win),
               "{\"type\":\"spi.idpoll.window\",\"between_ms\":%lu,\"clear\":%s,"
               "\"transitions\":{\"mosi\":%lu,\"miso\":%lu,\"cs\":%lu,\"clk\":%lu},\"total\":%lu}",
               (unsigned long)g_spi.idpoll_interval_ms, total == 0 ? "true" : "false",
               (unsigned long)d_mosi, (unsigned long)d_miso, (unsigned long)d_cs,
               (unsigned long)d_clk, (unsigned long)total);
      (void)app_send_text(win);
    }

    if (spi_xfer(tx, rx, sizeof(tx))) {
      uint8_t id[3] = {rx[1], rx[2], rx[3]};
      char id_hex[32];
      bytes_to_hex(id, sizeof(id), id_hex, sizeof(id_hex));
      snprintf(out, sizeof(out), "{\"type\":\"spi.id.result\",\"id_hex\":\"%s\",\"ok\":true}", id_hex);
      (void)app_send_text(out);
    } else {
      (void)app_send_text("{\"type\":\"spi.id.result\",\"ok\":false}");
      spi_dbg(1, "idpoll transfer failed");
    }
    // Per request, tri-state between repeated ID commands.
    spi_release_master();
    snapshot_pin_levels_no_count();
    set_idpoll_baseline();
  }

  if (g_spi.dump_enabled) {
    if ((now - g_spi.last_dump_ms) < SPI_DUMP_WS_INTERVAL_MS) return;
    g_spi.last_dump_ms = now;

    uint32_t left = (g_spi.dump_total_bytes > g_spi.dump_sent_bytes) ? (g_spi.dump_total_bytes - g_spi.dump_sent_bytes) : 0u;
    if (left == 0u) {
      g_spi.dump_enabled = false;
      spi_release_master();
      spi_dbg(1, "dump complete");
      send_spi_status("complete", g_spi.dump_to_file ? "dump finished" : "display finished");
      send_spi_state();
      return;
    }

    uint32_t n = left > g_spi.dump_chunk_bytes ? g_spi.dump_chunk_bytes : left;
    uint32_t read_n = n;
    uint32_t n_send;
    bool ff_run = false;
    bool ok;

    // Read a larger window when FF optimization is enabled so we can collapse
    // long erased regions into a single ff_run event.
    if (g_spi.dump_ff_opt) {
      if (read_n > SPI_DUMP_FF_SCAN_MAX_BYTES) read_n = SPI_DUMP_FF_SCAN_MAX_BYTES;
      ok = read_flash_region(g_spi.dump_start_addr + g_spi.dump_sent_bytes, g_spi.dump_read_cmd, g_spi.dump_addr_bytes, g_spi.dump_buf, read_n);
      if (!ok) {
        g_spi.dump_enabled = false;
        spi_release_master();
        (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_READ_FAILED\",\"msg\":\"dump read failed\"}");
        spi_dbg(1, "dump read failed");
        send_spi_status("error", "dump read failed");
        send_spi_state();
        return;
      }
      n = read_n;
      ff_run = is_all_ff(g_spi.dump_buf, n);
      if (!ff_run && n > SPI_DUMP_WS_DATA_BYTES) n = SPI_DUMP_WS_DATA_BYTES;
    } else {
      if (n > SPI_DUMP_WS_DATA_BYTES) n = SPI_DUMP_WS_DATA_BYTES;
      ok = read_flash_region(g_spi.dump_start_addr + g_spi.dump_sent_bytes, g_spi.dump_read_cmd, g_spi.dump_addr_bytes, g_spi.dump_buf, n);
      if (!ok) {
        g_spi.dump_enabled = false;
        spi_release_master();
        (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_READ_FAILED\",\"msg\":\"dump read failed\"}");
        spi_dbg(1, "dump read failed");
        send_spi_status("error", "dump read failed");
        send_spi_state();
        return;
      }
    }
    n_send = n;
    if (g_spi.dump_double_read && n_send > SPI_DUMP_VERIFY_MAX_BYTES) n_send = SPI_DUMP_VERIFY_MAX_BYTES;

    if (g_spi.dump_double_read) {
      uint32_t vlen = n_send;
      uint32_t attempt = 0;
      bool match = false;
      bool last_ok2 = false;
      uint32_t mismatch_idx = 0u;
      uint8_t exp_b = 0u;
      uint8_t got_b = 0u;
      if (vlen > SPI_DUMP_VERIFY_MAX_BYTES) vlen = SPI_DUMP_VERIFY_MAX_BYTES;
      for (attempt = 0; attempt <= g_spi.dump_verify_retries; attempt++) {
        bool ok2 = read_flash_region(g_spi.dump_start_addr + g_spi.dump_sent_bytes, g_spi.dump_read_cmd, g_spi.dump_addr_bytes, g_spi.dump_chk, vlen);
        last_ok2 = ok2;
        match = ok2 && (memcmp(g_spi.dump_buf, g_spi.dump_chk, vlen) == 0);
        if (ok2 && !match) {
          for (mismatch_idx = 0u; mismatch_idx < vlen; mismatch_idx++) {
            if (g_spi.dump_buf[mismatch_idx] != g_spi.dump_chk[mismatch_idx]) {
              exp_b = g_spi.dump_buf[mismatch_idx];
              got_b = g_spi.dump_chk[mismatch_idx];
              break;
            }
          }
        }
        if (match) break;
      }
      if (match) {
        g_spi.dump_verify_ok_count++;
        if ((g_spi.dump_verify_ok_count % 64u) == 0u) {
          (void)app_send_text("{\"type\":\"spi.dump.verify\",\"ok\":true}");
        }
      } else {
        char vmsg[320];
        uint32_t abs_addr = g_spi.dump_start_addr + g_spi.dump_sent_bytes + mismatch_idx;
        if (!last_ok2) {
          snprintf(vmsg, sizeof(vmsg),
                   "{\"type\":\"spi.dump.verify\",\"ok\":false,\"reason\":\"read2_failed\",\"offset\":%lu,\"length\":%lu,\"attempts\":%lu}",
                   (unsigned long)(g_spi.dump_sent_bytes), (unsigned long)vlen,
                   (unsigned long)(attempt + 1u));
        } else {
          snprintf(vmsg, sizeof(vmsg),
                   "{\"type\":\"spi.dump.verify\",\"ok\":false,\"reason\":\"mismatch\",\"offset\":%lu,\"address\":%lu,"
                   "\"index\":%lu,\"expected\":%u,\"actual\":%u,\"length\":%lu,\"attempts\":%lu}",
                   (unsigned long)(g_spi.dump_sent_bytes), (unsigned long)abs_addr,
                   (unsigned long)mismatch_idx, (unsigned)exp_b, (unsigned)got_b,
                   (unsigned long)vlen, (unsigned long)(attempt + 1u));
        }
        (void)app_send_text(vmsg);
      }
      if (!match) {
        char detail[192];
        g_spi.dump_enabled = false;
        spi_release_master();
        spi_dbg(1, "dump verify mismatch");
        if (!last_ok2) {
          snprintf(detail, sizeof(detail), "double-read failed (2nd read failed, attempts=%lu)",
                   (unsigned long)(attempt + 1u));
        } else {
          uint32_t abs_addr = g_spi.dump_start_addr + g_spi.dump_sent_bytes + mismatch_idx;
          snprintf(detail, sizeof(detail),
                   "double-read mismatch @0x%08lX idx=%lu exp=%02X got=%02X attempts=%lu",
                   (unsigned long)abs_addr, (unsigned long)mismatch_idx, (unsigned)exp_b,
                   (unsigned)got_b, (unsigned long)(attempt + 1u));
        }
        send_spi_status("error", detail);
        send_spi_state();
        return;
      }
    }

    if (g_spi.dump_ff_opt && ff_run) {
      if (n_send > 0xFFFFu) n_send = 0xFFFFu;
      if (!send_dump_ff_bin(g_spi.dump_start_addr + g_spi.dump_sent_bytes, (uint16_t)n_send)) {
        g_spi.dump_tx_fail_streak++;
        if (g_spi.dump_tx_fail_streak > SPI_DUMP_TX_FAIL_LIMIT) {
          g_spi.dump_enabled = false;
          spi_release_master();
          (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_TX_BUSY\",\"msg\":\"stream stalled (tx busy)\"}");
          send_spi_status("error", "stream stalled");
          send_spi_state();
        }
        return;
      }
    } else {
      if (n_send > SPI_DUMP_WS_DATA_BYTES) n_send = SPI_DUMP_WS_DATA_BYTES;
      if (!send_dump_data_bin(g_spi.dump_start_addr + g_spi.dump_sent_bytes, g_spi.dump_buf, (uint16_t)n_send)) {
        g_spi.dump_tx_fail_streak++;
        if (g_spi.dump_tx_fail_streak > SPI_DUMP_TX_FAIL_LIMIT) {
          g_spi.dump_enabled = false;
          spi_release_master();
          (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_TX_BUSY\",\"msg\":\"stream stalled (tx busy)\"}");
          send_spi_status("error", "stream stalled");
          send_spi_state();
        }
        return;
      }
    }

    g_spi.dump_tx_fail_streak = 0;
    g_spi.dump_sent_bytes += n_send;
    if (app_debug_level() >= 3u) {
      char m[96];
      snprintf(m, sizeof(m), "dump chunk sent: %lu/%lu", (unsigned long)g_spi.dump_sent_bytes,
               (unsigned long)g_spi.dump_total_bytes);
      spi_dbg(3, m);
    }
    if (g_spi.tristate_default) spi_release_master();
  }
}

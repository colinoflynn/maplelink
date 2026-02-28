#include "proto_spi.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_transport.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define SPI_DEV spi0
#define SPI_PORT_NAME "spi0"

// UART0 uses GPIO0/1 in proto_uart.c. Keep SPI away from those.
#define SPI_MISO_PIN 16
#define SPI_CS_PIN 17
#define SPI_SCK_PIN 18
#define SPI_MOSI_PIN 19
#define SPI_RST_PIN 20
#define SPI_HOLD_PIN 21

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
  bool hold_present;
  bool rst_present;
  uint32_t speed_hz;
  uint32_t idpoll_interval_ms;
  uint8_t dump_addr_bytes;
  uint8_t dump_read_cmd;
  bool dump_double_read;
  bool dump_ff_opt;
  uint32_t dump_total_bytes;
  uint32_t dump_chunk_bytes;
  uint32_t dump_sent_bytes;
  uint32_t dump_verify_ok_count;
  uint32_t last_detect_ms;
  uint32_t last_idpoll_ms;
  uint32_t last_dump_ms;
  uint32_t tr_mosi;
  uint32_t tr_miso;
  uint32_t tr_cs;
  uint32_t tr_clk;
  uint32_t tr_rst;
  uint32_t tr_hold;
  uint32_t base_tr_mosi;
  uint32_t base_tr_miso;
  uint32_t base_tr_cs;
  uint32_t base_tr_clk;
  uint32_t base_tr_rst;
  uint32_t base_tr_hold;
  uint8_t lv_mosi;
  uint8_t lv_miso;
  uint8_t lv_cs;
  uint8_t lv_clk;
  uint8_t lv_rst;
  uint8_t lv_hold;
  bool spi_active;
  bool sniff_frame_active;
  uint8_t sniff_cur_tx;
  uint8_t sniff_cur_rx;
  uint8_t sniff_bit_count;
  uint8_t sniff_prev_clk;
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
    .hold_present = false,
    .rst_present = false,
    .speed_hz = 1000000,
    .idpoll_interval_ms = 250,
    .dump_addr_bytes = 3,
    .dump_read_cmd = 0x03,
    .dump_double_read = false,
    .dump_ff_opt = true,
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

static void pin_input(uint pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_disable_pulls(pin);
  if (g_spi.pullups_enabled) gpio_pull_up(pin);
}

static void pin_input_opt(uint pin, bool present) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  if (!present) {
    gpio_disable_pulls(pin);
    return;
  }
  gpio_disable_pulls(pin);
  if (g_spi.pullups_enabled) gpio_pull_up(pin);
}

static void spi_apply_safe_io(void) {
  // All lines as inputs unless an operation takes ownership.
  pin_input(SPI_MOSI_PIN);
  pin_input(SPI_MISO_PIN);
  pin_input(SPI_CS_PIN);
  pin_input(SPI_SCK_PIN);
  pin_input_opt(SPI_RST_PIN, g_spi.rst_present);
  pin_input_opt(SPI_HOLD_PIN, g_spi.hold_present);
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

static void send_spi_state(void) {
  char out[768];
  snprintf(out, sizeof(out),
           "{\"type\":\"spi.config\",\"port\":\"" SPI_PORT_NAME "\",\"status\":\"hw\","
           "\"pins\":{\"mosi\":%u,\"miso\":%u,\"cs\":%u,\"clk\":%u,\"rst\":%u,\"hold\":%u},"
           "\"tristate_default\":%s,\"pullups_enabled\":%s,\"hold_present\":%s,\"rst_present\":%s,"
           "\"speed_hz\":%lu,\"idpoll_interval_ms\":%lu,"
           "\"detect_enabled\":%s,\"sniffer_enabled\":%s,\"idpoll_enabled\":%s,\"idpoll_monitor_between\":%s,\"dump_enabled\":%s,"
           "\"dump\":{\"addr_bytes\":%u,\"read_cmd\":%u,\"double_read\":%s,\"ff_opt\":%s}}",
           (unsigned)SPI_MOSI_PIN, (unsigned)SPI_MISO_PIN, (unsigned)SPI_CS_PIN, (unsigned)SPI_SCK_PIN,
           (unsigned)SPI_RST_PIN, (unsigned)SPI_HOLD_PIN, g_spi.tristate_default ? "true" : "false",
           g_spi.pullups_enabled ? "true" : "false", g_spi.hold_present ? "true" : "false",
           g_spi.rst_present ? "true" : "false", (unsigned long)g_spi.speed_hz,
           (unsigned long)g_spi.idpoll_interval_ms, g_spi.detect_enabled ? "true" : "false",
           g_spi.sniffer_enabled ? "true" : "false", g_spi.idpoll_enabled ? "true" : "false",
           g_spi.idpoll_monitor_between ? "true" : "false",
           g_spi.dump_enabled ? "true" : "false", g_spi.dump_addr_bytes, g_spi.dump_read_cmd,
           g_spi.dump_double_read ? "true" : "false", g_spi.dump_ff_opt ? "true" : "false");
  (void)app_send_text(out);
}

static void send_spi_status(const char *state, const char *detail) {
  char out[256];
  snprintf(out, sizeof(out), "{\"type\":\"spi.dump.status\",\"port\":\"" SPI_PORT_NAME "\",\"state\":\"%s\",\"detail\":\"%s\"}",
           state, detail ? detail : "");
  (void)app_send_text(out);
}

static void ensure_safe_exclusion(void) {
  if (g_spi.dump_enabled || g_spi.idpoll_enabled) g_spi.sniffer_enabled = false;
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

  if (g_spi.rst_present) {
    v = (uint8_t)gpio_get(SPI_RST_PIN);
    if (v != g_spi.lv_rst) g_spi.tr_rst++;
    g_spi.lv_rst = v;
  }

  if (g_spi.hold_present) {
    v = (uint8_t)gpio_get(SPI_HOLD_PIN);
    if (v != g_spi.lv_hold) g_spi.tr_hold++;
    g_spi.lv_hold = v;
  }
}

static void send_pin_snapshot(void) {
  char out[384];
  snprintf(out, sizeof(out),
           "{\"type\":\"spi.pins\",\"mosi\":%u,\"miso\":%u,\"cs\":%u,\"clk\":%u,\"rst\":%s,\"hold\":%s,"
           "\"transitions\":{\"mosi\":%lu,\"miso\":%lu,\"cs\":%lu,\"clk\":%lu,\"rst\":%lu,\"hold\":%lu}}",
           (unsigned)g_spi.lv_mosi, (unsigned)g_spi.lv_miso, (unsigned)g_spi.lv_cs, (unsigned)g_spi.lv_clk,
           g_spi.rst_present ? (g_spi.lv_rst ? "1" : "0") : "null",
           g_spi.hold_present ? (g_spi.lv_hold ? "1" : "0") : "null",
           (unsigned long)g_spi.tr_mosi, (unsigned long)g_spi.tr_miso, (unsigned long)g_spi.tr_cs,
           (unsigned long)g_spi.tr_clk, (unsigned long)g_spi.tr_rst, (unsigned long)g_spi.tr_hold);
  (void)app_send_text(out);
}

static void snapshot_pin_levels_no_count(void) {
  g_spi.lv_mosi = (uint8_t)gpio_get(SPI_MOSI_PIN);
  g_spi.lv_miso = (uint8_t)gpio_get(SPI_MISO_PIN);
  g_spi.lv_cs = (uint8_t)gpio_get(SPI_CS_PIN);
  g_spi.lv_clk = (uint8_t)gpio_get(SPI_SCK_PIN);
  if (g_spi.rst_present) g_spi.lv_rst = (uint8_t)gpio_get(SPI_RST_PIN);
  if (g_spi.hold_present) g_spi.lv_hold = (uint8_t)gpio_get(SPI_HOLD_PIN);
}

static void set_idpoll_baseline(void) {
  g_spi.base_tr_mosi = g_spi.tr_mosi;
  g_spi.base_tr_miso = g_spi.tr_miso;
  g_spi.base_tr_cs = g_spi.tr_cs;
  g_spi.base_tr_clk = g_spi.tr_clk;
  g_spi.base_tr_rst = g_spi.tr_rst;
  g_spi.base_tr_hold = g_spi.tr_hold;
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

static void sniff_sample(void) {
  // Passive SPI mode-0 sniffer: samples MOSI/MISO on rising CLK while CS is low.
  uint8_t cs = (uint8_t)gpio_get(SPI_CS_PIN);
  uint8_t clk = (uint8_t)gpio_get(SPI_SCK_PIN);

  if (cs == 0u && !g_spi.sniff_frame_active) {
    g_spi.sniff_frame_active = true;
    g_spi.sniff_len = 0;
    g_spi.sniff_dropped_frame = 0;
    g_spi.sniff_cur_tx = 0;
    g_spi.sniff_cur_rx = 0;
    g_spi.sniff_bit_count = 0;
  }

  if (g_spi.sniff_frame_active && cs == 1u) {
    if (g_spi.sniff_bit_count != 0u) {
      if (g_spi.sniff_len < SPI_MAX_SNIFF_BYTES) {
        g_spi.sniff_tx[g_spi.sniff_len] = (uint8_t)(g_spi.sniff_cur_tx << (8u - g_spi.sniff_bit_count));
        g_spi.sniff_rx[g_spi.sniff_len] = (uint8_t)(g_spi.sniff_cur_rx << (8u - g_spi.sniff_bit_count));
        g_spi.sniff_len++;
      } else {
        g_spi.sniff_dropped_frame++;
        g_spi.sniff_dropped_total++;
      }
    }
    sniff_emit_frame();
    g_spi.sniff_frame_active = false;
    return;
  }

  if (!g_spi.sniff_frame_active) {
    g_spi.sniff_prev_clk = clk;
    return;
  }

  if (g_spi.sniff_prev_clk == 0u && clk == 1u) {
    g_spi.sniff_cur_tx = (uint8_t)((g_spi.sniff_cur_tx << 1) | (gpio_get(SPI_MOSI_PIN) ? 1u : 0u));
    g_spi.sniff_cur_rx = (uint8_t)((g_spi.sniff_cur_rx << 1) | (gpio_get(SPI_MISO_PIN) ? 1u : 0u));
    g_spi.sniff_bit_count++;
    if (g_spi.sniff_bit_count >= 8u) {
      if (g_spi.sniff_len < SPI_MAX_SNIFF_BYTES) {
        g_spi.sniff_tx[g_spi.sniff_len] = g_spi.sniff_cur_tx;
        g_spi.sniff_rx[g_spi.sniff_len] = g_spi.sniff_cur_rx;
        g_spi.sniff_len++;
      } else {
        g_spi.sniff_dropped_frame++;
        g_spi.sniff_dropped_total++;
      }
      g_spi.sniff_cur_tx = 0;
      g_spi.sniff_cur_rx = 0;
      g_spi.sniff_bit_count = 0;
    }
  }
  g_spi.sniff_prev_clk = clk;
}

static void handle_set_io(const char *json) {
  bool b;
  if (json_extract_bool(json, "tristate_default", &b)) g_spi.tristate_default = b;
  if (json_extract_bool(json, "pullups_enabled", &b)) g_spi.pullups_enabled = b;
  if (json_extract_bool(json, "hold_present", &b)) g_spi.hold_present = b;
  if (json_extract_bool(json, "rst_present", &b)) g_spi.rst_present = b;

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

static void handle_dump_start(const char *json) {
  uint32_t v;
  bool b;

  if (json_extract_u32(json, "addr_bytes", &v)) g_spi.dump_addr_bytes = (uint8_t)v;
  if (json_extract_u32(json, "read_cmd", &v)) g_spi.dump_read_cmd = (uint8_t)v;
  if (json_extract_u32(json, "length_bytes", &v)) g_spi.dump_total_bytes = v;
  if (json_extract_u32(json, "chunk_bytes", &v)) g_spi.dump_chunk_bytes = v;
  if (json_extract_u32(json, "speed_hz", &v)) g_spi.speed_hz = clamp_u32(v, SPI_MIN_SPEED_HZ, SPI_MAX_SPEED_HZ);
  if (json_extract_bool(json, "double_read", &b)) g_spi.dump_double_read = b;
  if (json_extract_bool(json, "ff_opt", &b)) g_spi.dump_ff_opt = b;

  if (g_spi.dump_addr_bytes < 2u) g_spi.dump_addr_bytes = 2u;
  if (g_spi.dump_addr_bytes > 4u) g_spi.dump_addr_bytes = 4u;
  if (g_spi.dump_total_bytes == 0u) g_spi.dump_total_bytes = 4096u;
  if (g_spi.dump_chunk_bytes == 0u) g_spi.dump_chunk_bytes = 256u;
  if (g_spi.dump_chunk_bytes > SPI_MAX_DUMP_CHUNK) g_spi.dump_chunk_bytes = SPI_MAX_DUMP_CHUNK;

  g_spi.dump_sent_bytes = 0;
  g_spi.dump_verify_ok_count = 0;
  g_spi.last_dump_ms = now_ms();
  g_spi.dump_enabled = true;
  g_spi.idpoll_enabled = false;
  ensure_safe_exclusion();
  spi_dbg(1, "dump start accepted");
  send_spi_status("running", "dump started");
  send_spi_state();
}

void proto_spi_init(void) {
  uint32_t now = now_ms();
  g_spi.last_detect_ms = now;
  g_spi.last_idpoll_ms = now;
  g_spi.last_dump_ms = now;
  g_spi.lv_mosi = 0;
  g_spi.lv_miso = 0;
  g_spi.lv_cs = 1;
  g_spi.lv_clk = 0;
  g_spi.lv_rst = 1;
  g_spi.lv_hold = 1;
  g_spi.sniff_prev_clk = 0;
  spi_apply_safe_io();
}

void proto_spi_on_client_open(ws_conn_t *conn) { (void)conn; }

void proto_spi_on_client_close(ws_conn_t *conn) {
  (void)conn;
  g_spi.sniffer_enabled = false;
  g_spi.idpoll_enabled = false;
  g_spi.dump_enabled = false;
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
    g_spi.sniffer_enabled = true;
    g_spi.sniff_frame_active = false;
    g_spi.sniff_prev_clk = (uint8_t)gpio_get(SPI_SCK_PIN);
    spi_enter_passive_mode();
    spi_dbg(1, "sniffer started");
    send_spi_status("sniffer", "enabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.sniffer.stop") == 0) {
    g_spi.sniffer_enabled = false;
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
    spi_release_master();
    spi_dbg(1, "idpoll stopped");
    send_spi_status("idpoll", "disabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.dump.start") == 0) {
    handle_dump_start(json);
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

  return app_send_text("{\"type\":\"error\",\"code\":\"SPI_UNSUPPORTED\",\"msg\":\"spi command not implemented\"}");
}

void proto_spi_poll(void) {
  uint32_t now = now_ms();

  if (g_spi.detect_enabled || g_spi.sniffer_enabled || (g_spi.idpoll_enabled && g_spi.idpoll_monitor_between)) {
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
    uint32_t d_rst = g_spi.tr_rst - g_spi.base_tr_rst;
    uint32_t d_hold = g_spi.tr_hold - g_spi.base_tr_hold;
    uint32_t total = d_mosi + d_miso + d_cs + d_clk + d_rst + d_hold;
    g_spi.last_idpoll_ms = now;

    if (g_spi.idpoll_monitor_between) {
      char win[320];
      snprintf(win, sizeof(win),
               "{\"type\":\"spi.idpoll.window\",\"between_ms\":%lu,\"clear\":%s,"
               "\"transitions\":{\"mosi\":%lu,\"miso\":%lu,\"cs\":%lu,\"clk\":%lu,\"rst\":%lu,\"hold\":%lu},\"total\":%lu}",
               (unsigned long)g_spi.idpoll_interval_ms, total == 0 ? "true" : "false",
               (unsigned long)d_mosi, (unsigned long)d_miso, (unsigned long)d_cs,
               (unsigned long)d_clk, (unsigned long)d_rst, (unsigned long)d_hold, (unsigned long)total);
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
      send_spi_status("complete", "dump finished");
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
      ok = read_flash_region(g_spi.dump_sent_bytes, g_spi.dump_read_cmd, g_spi.dump_addr_bytes, g_spi.dump_buf, read_n);
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
      ok = read_flash_region(g_spi.dump_sent_bytes, g_spi.dump_read_cmd, g_spi.dump_addr_bytes, g_spi.dump_buf, n);
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
      if (vlen > SPI_DUMP_VERIFY_MAX_BYTES) vlen = SPI_DUMP_VERIFY_MAX_BYTES;
      bool ok2 = read_flash_region(g_spi.dump_sent_bytes, g_spi.dump_read_cmd, g_spi.dump_addr_bytes, g_spi.dump_chk, vlen);
      bool match = ok2 && (memcmp(g_spi.dump_buf, g_spi.dump_chk, vlen) == 0);
      if (match) {
        g_spi.dump_verify_ok_count++;
        if ((g_spi.dump_verify_ok_count % 64u) == 0u) {
          (void)app_send_text("{\"type\":\"spi.dump.verify\",\"ok\":true}");
        }
      } else {
        (void)app_send_text("{\"type\":\"spi.dump.verify\",\"ok\":false}");
      }
      if (!match) {
        g_spi.dump_enabled = false;
        spi_release_master();
        spi_dbg(1, "dump verify mismatch");
        send_spi_status("error", "double-read mismatch");
        send_spi_state();
        return;
      }
    }

    if (g_spi.dump_ff_opt && ff_run) {
      if (n_send > 0xFFFFu) n_send = 0xFFFFu;
      if (!send_dump_ff_bin(g_spi.dump_sent_bytes, (uint16_t)n_send)) return;
    } else {
      if (n_send > SPI_DUMP_WS_DATA_BYTES) n_send = SPI_DUMP_WS_DATA_BYTES;
      if (!send_dump_data_bin(g_spi.dump_sent_bytes, g_spi.dump_buf, (uint16_t)n_send)) return;
    }

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

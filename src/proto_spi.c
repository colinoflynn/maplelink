#include "proto_spi.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_transport.h"
#include "pico/stdlib.h"

typedef struct {
  bool tristate_default;
  bool pullups_enabled;
  bool detect_enabled;
  bool sniffer_enabled;
  bool idpoll_enabled;
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
  uint32_t last_detect_ms;
  uint32_t last_idpoll_ms;
  uint32_t last_sniffer_ms;
  uint32_t last_dump_ms;
  uint32_t tr_mosi;
  uint32_t tr_miso;
  uint32_t tr_cs;
  uint32_t tr_clk;
  uint32_t tr_rst;
  uint32_t tr_hold;
} spi_state_t;

static spi_state_t g_spi = {
  .tristate_default = true,
  .pullups_enabled = true,
  .detect_enabled = false,
  .sniffer_enabled = false,
  .idpoll_enabled = false,
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

static void send_spi_state(void) {
  char out[512];
  snprintf(out, sizeof(out),
           "{\"type\":\"spi.config\",\"port\":\"spi0\",\"status\":\"stub\","
           "\"tristate_default\":%s,\"pullups_enabled\":%s,\"hold_present\":%s,\"rst_present\":%s,"
           "\"speed_hz\":%lu,\"idpoll_interval_ms\":%lu,"
           "\"detect_enabled\":%s,\"sniffer_enabled\":%s,\"idpoll_enabled\":%s,\"dump_enabled\":%s,"
           "\"dump\":{\"addr_bytes\":%u,\"read_cmd\":%u,\"double_read\":%s,\"ff_opt\":%s}}",
           g_spi.tristate_default ? "true" : "false", g_spi.pullups_enabled ? "true" : "false",
           g_spi.hold_present ? "true" : "false", g_spi.rst_present ? "true" : "false",
           (unsigned long)g_spi.speed_hz, (unsigned long)g_spi.idpoll_interval_ms,
           g_spi.detect_enabled ? "true" : "false", g_spi.sniffer_enabled ? "true" : "false",
           g_spi.idpoll_enabled ? "true" : "false", g_spi.dump_enabled ? "true" : "false",
           g_spi.dump_addr_bytes, g_spi.dump_read_cmd, g_spi.dump_double_read ? "true" : "false",
           g_spi.dump_ff_opt ? "true" : "false");
  (void)app_send_text(out);
}

static void send_spi_status(const char *state, const char *detail) {
  char out[256];
  snprintf(out, sizeof(out), "{\"type\":\"spi.dump.status\",\"port\":\"spi0\",\"state\":\"%s\",\"detail\":\"%s\"}",
           state, detail ? detail : "");
  (void)app_send_text(out);
}

static bool extract_u32(const char *json, const char *key, uint32_t *out) {
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

static bool extract_bool(const char *json, const char *key, bool *out) {
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

static void ensure_safe_exclusion(void) {
  if (g_spi.dump_enabled || g_spi.idpoll_enabled) g_spi.sniffer_enabled = false;
}

void proto_spi_init(void) {
  g_spi.last_detect_ms = now_ms();
  g_spi.last_idpoll_ms = g_spi.last_detect_ms;
  g_spi.last_sniffer_ms = g_spi.last_detect_ms;
  g_spi.last_dump_ms = g_spi.last_detect_ms;
}

void proto_spi_on_client_open(ws_conn_t *conn) {
  (void)conn;
}

void proto_spi_on_client_close(ws_conn_t *conn) {
  (void)conn;
}

static void handle_set_io(const char *json) {
  bool b;
  if (extract_bool(json, "tristate_default", &b)) g_spi.tristate_default = b;
  if (extract_bool(json, "pullups_enabled", &b)) g_spi.pullups_enabled = b;
  if (extract_bool(json, "hold_present", &b)) g_spi.hold_present = b;
  if (extract_bool(json, "rst_present", &b)) g_spi.rst_present = b;
  send_spi_state();
}

static void handle_set_speed(const char *json) {
  uint32_t v;
  if (extract_u32(json, "speed_hz", &v)) g_spi.speed_hz = v;
  if (extract_u32(json, "idpoll_interval_ms", &v)) g_spi.idpoll_interval_ms = v;
  send_spi_state();
}

static void handle_dump_start(const char *json) {
  uint32_t v;
  bool b;

  if (extract_u32(json, "addr_bytes", &v)) g_spi.dump_addr_bytes = (uint8_t)v;
  if (extract_u32(json, "read_cmd", &v)) g_spi.dump_read_cmd = (uint8_t)v;
  if (extract_u32(json, "length_bytes", &v)) g_spi.dump_total_bytes = v;
  if (extract_u32(json, "chunk_bytes", &v)) g_spi.dump_chunk_bytes = v;
  if (extract_bool(json, "double_read", &b)) g_spi.dump_double_read = b;
  if (extract_bool(json, "ff_opt", &b)) g_spi.dump_ff_opt = b;

  if (g_spi.dump_total_bytes == 0) g_spi.dump_total_bytes = 4096;
  if (g_spi.dump_chunk_bytes == 0) g_spi.dump_chunk_bytes = 256;
  if (g_spi.dump_chunk_bytes > 4096) g_spi.dump_chunk_bytes = 4096;

  g_spi.dump_sent_bytes = 0;
  g_spi.dump_enabled = true;
  g_spi.idpoll_enabled = false;
  ensure_safe_exclusion();
  send_spi_status("running", "dump started");
  send_spi_state();
}

bool proto_spi_handle_text(const char *type, const char *json) {
  if (!starts_with(type, "spi.")) return false;

  if (strcmp(type, "spi.get_config") == 0) {
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
    send_spi_status("detect", "enabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.detect.stop") == 0) {
    g_spi.detect_enabled = false;
    send_spi_status("detect", "disabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.sniffer.start") == 0) {
    if (g_spi.dump_enabled || g_spi.idpoll_enabled) {
      (void)app_send_text("{\"type\":\"error\",\"code\":\"SPI_BUSY\",\"msg\":\"sniffer blocked while idpoll/dump active\"}");
      return true;
    }
    g_spi.sniffer_enabled = true;
    send_spi_status("sniffer", "enabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.sniffer.stop") == 0) {
    g_spi.sniffer_enabled = false;
    send_spi_status("sniffer", "disabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.idpoll.start") == 0) {
    uint32_t v;
    if (extract_u32(json, "interval_ms", &v)) g_spi.idpoll_interval_ms = v;
    if (extract_u32(json, "speed_hz", &v)) g_spi.speed_hz = v;
    g_spi.idpoll_enabled = true;
    g_spi.dump_enabled = false;
    ensure_safe_exclusion();
    send_spi_status("idpoll", "enabled");
    send_spi_state();
    return true;
  }

  if (strcmp(type, "spi.idpoll.stop") == 0) {
    g_spi.idpoll_enabled = false;
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
    send_spi_status("stopped", "dump stopped");
    send_spi_state();
    return true;
  }

  return app_send_text("{\"type\":\"error\",\"code\":\"SPI_UNSUPPORTED\",\"msg\":\"spi command not implemented\"}");
}

void proto_spi_poll(void) {
  uint32_t now = now_ms();

  if (g_spi.detect_enabled && (now - g_spi.last_detect_ms) >= 150) {
    char out[320];
    g_spi.last_detect_ms = now;
    g_spi.tr_mosi += 3;
    g_spi.tr_miso += 2;
    g_spi.tr_cs += 1;
    g_spi.tr_clk += 4;
    if (g_spi.rst_present) g_spi.tr_rst += 1;
    if (g_spi.hold_present) g_spi.tr_hold += 1;

    snprintf(out, sizeof(out),
             "{\"type\":\"spi.pins\",\"mosi\":%s,\"miso\":%s,\"cs\":%s,\"clk\":%s,\"rst\":%s,\"hold\":%s,"
             "\"transitions\":{\"mosi\":%lu,\"miso\":%lu,\"cs\":%lu,\"clk\":%lu,\"rst\":%lu,\"hold\":%lu}}",
             "1", "0", "1", "0", g_spi.rst_present ? "1" : "null", g_spi.hold_present ? "1" : "null",
             (unsigned long)g_spi.tr_mosi, (unsigned long)g_spi.tr_miso, (unsigned long)g_spi.tr_cs,
             (unsigned long)g_spi.tr_clk, (unsigned long)g_spi.tr_rst, (unsigned long)g_spi.tr_hold);
    (void)app_send_text(out);
  }

  if (g_spi.idpoll_enabled && (now - g_spi.last_idpoll_ms) >= g_spi.idpoll_interval_ms) {
    g_spi.last_idpoll_ms = now;
    (void)app_send_text("{\"type\":\"spi.id.result\",\"id_hex\":\"EF 40 18\",\"ok\":true}");
  }

  if (g_spi.sniffer_enabled && (now - g_spi.last_sniffer_ms) >= 250) {
    g_spi.last_sniffer_ms = now;
    (void)app_send_text("{\"type\":\"spi.sniffer.frame\",\"cs_frame\":1,\"hex\":\"9F EF 40 18\"}");
  }

  if (g_spi.dump_enabled && (now - g_spi.last_dump_ms) >= 60) {
    uint32_t left;
    uint32_t n;
    char out[320];

    g_spi.last_dump_ms = now;
    left = (g_spi.dump_total_bytes > g_spi.dump_sent_bytes) ? (g_spi.dump_total_bytes - g_spi.dump_sent_bytes) : 0;
    if (left == 0) {
      g_spi.dump_enabled = false;
      send_spi_status("complete", "dump finished");
      send_spi_state();
      return;
    }

    n = left > g_spi.dump_chunk_bytes ? g_spi.dump_chunk_bytes : left;

    if (g_spi.dump_ff_opt && (g_spi.dump_sent_bytes % (g_spi.dump_chunk_bytes * 3) == 0)) {
      snprintf(out, sizeof(out),
               "{\"type\":\"spi.dump.chunk\",\"mode\":\"ff_run\",\"offset\":%lu,\"count\":%lu}",
               (unsigned long)g_spi.dump_sent_bytes, (unsigned long)n);
    } else {
      snprintf(out, sizeof(out),
               "{\"type\":\"spi.dump.chunk\",\"mode\":\"data\",\"offset\":%lu,\"hex_preview\":\"00 11 22 33 44 55 66 77 ...\",\"count\":%lu}",
               (unsigned long)g_spi.dump_sent_bytes, (unsigned long)n);
    }
    (void)app_send_text(out);
    g_spi.dump_sent_bytes += n;

    if (g_spi.dump_double_read && (g_spi.dump_sent_bytes % (g_spi.dump_chunk_bytes * 4) == 0)) {
      (void)app_send_text("{\"type\":\"spi.dump.verify\",\"ok\":true}");
    }
  }
}

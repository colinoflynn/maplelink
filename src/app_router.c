#include "app_router.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_transport.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "proto_i2c.h"
#include "proto_emmc.h"
#include "proto_spi.h"
#include "proto_uart.h"

static ws_conn_t *g_client;
static bool g_bootsel_pending;
static uint8_t g_debug_level = 1;

bool app_send_text(const char *text) {
  if (!g_client || !text) return false;
  return ws_conn_send_text(g_client, (const uint8_t *)text, (uint16_t)strlen(text));
}

bool app_send_binary(const uint8_t *data, uint16_t len) {
  if (!g_client || !data || len == 0) return false;
  return ws_conn_send_binary(g_client, data, len);
}

ws_conn_t *app_current_client(void) {
  return g_client;
}

uint8_t app_debug_level(void) {
  return g_debug_level;
}

bool app_debug_log(uint8_t level, const char *scope, const char *msg) {
  char out[320];
  if (!g_client) return false;
  if (level > g_debug_level) return false;
  snprintf(out, sizeof(out), "{\"type\":\"debug.log\",\"level\":%u,\"scope\":\"%s\",\"msg\":\"%s\"}",
           (unsigned)level, scope ? scope : "sys", msg ? msg : "");
  return app_send_text(out);
}

static bool extract_u32_key(const char *json, const char *key, uint32_t *out) {
  char pattern[40];
  const char *p;
  unsigned v;

  if (!json || !key || !out) return false;
  snprintf(pattern, sizeof(pattern), "\"%s\":", key);
  p = strstr(json, pattern);
  if (!p) return false;
  p += strlen(pattern);
  if (sscanf(p, "%u", &v) != 1) return false;
  *out = v;
  return true;
}

static bool send_debug_config(void) {
  char out[96];
  snprintf(out, sizeof(out), "{\"type\":\"system.debug\",\"level\":%u}", (unsigned)g_debug_level);
  return app_send_text(out);
}

static bool extract_type(const char *json, char *out, size_t out_sz) {
  const char *p;
  const char *e;
  size_t n;

  if (!json || !out || out_sz < 2) return false;

  p = strstr(json, "\"type\"");
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (*p != '"') return false;
  p++;

  e = strchr(p, '"');
  if (!e) return false;
  n = (size_t)(e - p);
  if (n == 0 || n >= out_sz) return false;

  memcpy(out, p, n);
  out[n] = 0;
  return true;
}

static bool dispatch_text(const char *type, const char *json) {
  uint32_t v;
  if (strcmp(type, "system.bootsel") == 0) {
    g_bootsel_pending = true;
    (void)app_debug_log(1, "system", "bootsel requested");
    return app_send_text("{\"type\":\"system.bootsel\",\"ok\":true,\"msg\":\"entering usb boot mode\"}");
  }
  if (strcmp(type, "system.get_debug") == 0) {
    return send_debug_config();
  }
  if (strcmp(type, "system.set_debug") == 0) {
    if (extract_u32_key(json, "level", &v)) {
      if (v > 3u) v = 3u;
      g_debug_level = (uint8_t)v;
      (void)app_debug_log(1, "system", "debug level updated");
    }
    return send_debug_config();
  }
  if (strcmp(type, "ping") == 0) {
    return app_send_text("{\"type\":\"pong\"}");
  }
  if (proto_uart_handle_text(type, json)) return true;
  if (proto_spi_handle_text(type, json)) return true;
  if (proto_i2c_handle_text(type, json)) return true;
  if (proto_emmc_handle_text(type, json)) return true;
  return false;
}

static void send_hello(void) {
  char out[224];
  snprintf(out, sizeof(out),
           "{\"type\":\"hello\",\"fw\":\"0.2.0\",\"build_date\":\"%s\",\"build_time\":\"%s\","
           "\"caps\":[\"uart0\",\"spi0\",\"i2c0\",\"emmc0\",\"bootsel\",\"debug\"]}",
           __DATE__, __TIME__);
  (void)app_send_text(out);
}

static void on_open(ws_conn_t *conn) {
  if (g_client && g_client != conn) ws_conn_close(g_client);
  g_client = conn;

  proto_uart_on_client_open(conn);
  proto_spi_on_client_open(conn);
  proto_i2c_on_client_open(conn);
  proto_emmc_on_client_open(conn);
  send_hello();
}

static void on_close(ws_conn_t *conn) {
  proto_uart_on_client_close(conn);
  proto_spi_on_client_close(conn);
  proto_i2c_on_client_close(conn);
  proto_emmc_on_client_close(conn);

  if (g_client == conn) g_client = NULL;
}

static void on_text(ws_conn_t *conn, const uint8_t *data, uint16_t len) {
  char msg[512];
  char type[64];
  (void)conn;

  if (len >= sizeof(msg)) {
    (void)app_send_text("{\"type\":\"error\",\"code\":\"TOO_LARGE\",\"msg\":\"message too large\"}");
    return;
  }

  memcpy(msg, data, len);
  msg[len] = 0;

  if (!extract_type(msg, type, sizeof(type))) {
    (void)app_send_text("{\"type\":\"error\",\"code\":\"BAD_REQUEST\",\"msg\":\"missing type\"}");
    return;
  }

  if (strcmp(type, "hello") == 0) {
    send_hello();
    return;
  }

  if (!dispatch_text(type, msg)) {
    (void)app_send_text("{\"type\":\"error\",\"code\":\"UNKNOWN\",\"msg\":\"unknown command\"}");
  }
}

static void on_binary(ws_conn_t *conn, const uint8_t *data, uint16_t len) {
  (void)conn;
  (void)proto_uart_handle_binary(data, len);
}

const ws_app_handler_t *app_router_handler(void) {
  static bool inited;
  static ws_app_handler_t handler = {
    .on_open = on_open,
    .on_close = on_close,
    .on_text = on_text,
    .on_binary = on_binary,
  };

  if (!inited) {
    proto_uart_init();
    proto_spi_init();
    proto_i2c_init();
    proto_emmc_init();
    inited = true;
  }

  return &handler;
}

void app_router_poll(void) {
  if (g_bootsel_pending) {
    // Small delay gives websocket ACK a chance to flush before rebooting.
    sleep_ms(40);
    reset_usb_boot(0, 0);
    return;
  }
  proto_uart_poll();
  proto_spi_poll();
  proto_i2c_poll();
  proto_emmc_poll();
}

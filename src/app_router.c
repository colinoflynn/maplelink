#include "app_router.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_transport.h"
#include "proto_i2c.h"
#include "proto_spi.h"
#include "proto_uart.h"

static ws_conn_t *g_client;

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
  if (proto_uart_handle_text(type, json)) return true;
  if (proto_spi_handle_text(type, json)) return true;
  if (proto_i2c_handle_text(type, json)) return true;
  return false;
}

static void send_hello(void) {
  (void)app_send_text("{\"type\":\"hello\",\"fw\":\"0.2.0\",\"caps\":[\"uart0\",\"spi0\",\"i2c0\"]}");
}

static void on_open(ws_conn_t *conn) {
  if (g_client && g_client != conn) ws_conn_close(g_client);
  g_client = conn;

  proto_uart_on_client_open(conn);
  proto_spi_on_client_open(conn);
  proto_i2c_on_client_open(conn);
  send_hello();
}

static void on_close(ws_conn_t *conn) {
  proto_uart_on_client_close(conn);
  proto_spi_on_client_close(conn);
  proto_i2c_on_client_close(conn);

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
    inited = true;
  }

  return &handler;
}

void app_router_poll(void) {
  proto_uart_poll();
  proto_spi_poll();
  proto_i2c_poll();
}

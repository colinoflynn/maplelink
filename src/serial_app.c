#include "serial_app.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/uart.h"
#include "pico/stdlib.h"

#define SERIAL_UART_ID uart0
#define SERIAL_TX_PIN 0
#define SERIAL_RX_PIN 1
#define RX_TMP_BUF_LEN 64

typedef struct {
  uint32_t baud;
  uint8_t data_bits;
  uint8_t stop_bits;
  char parity[5];
  char flow[5];
} uart_cfg_t;

static uart_cfg_t g_cfg = {
  .baud = 115200,
  .data_bits = 8,
  .stop_bits = 1,
  .parity = "none",
  .flow = "none",
};

static bool g_uart_ready;
static ws_conn_t *g_client;

static void send_json(const char *s) {
  if (!g_client) return;
  ws_conn_send_text(g_client, (const uint8_t *)s, (uint16_t)strlen(s));
}

static bool extract_u32(const char *json, const char *key, uint32_t *out) {
  char pattern[32];
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

static bool extract_str(const char *json, const char *key, char *out, size_t out_sz) {
  char pattern[32];
  const char *p;
  const char *e;
  size_t n;

  if (out_sz < 2) return false;

  snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
  p = strstr(json, pattern);
  if (!p) return false;
  p += strlen(pattern);
  e = strchr(p, '"');
  if (!e) return false;

  n = (size_t)(e - p);
  if (n == 0 || n >= out_sz) return false;
  memcpy(out, p, n);
  out[n] = 0;
  return true;
}

static bool apply_uart_cfg(const uart_cfg_t *cfg) {
  uart_parity_t parity_mode = UART_PARITY_NONE;

  if (strcmp(cfg->parity, "even") == 0) parity_mode = UART_PARITY_EVEN;
  else if (strcmp(cfg->parity, "odd") == 0) parity_mode = UART_PARITY_ODD;
  else if (strcmp(cfg->parity, "none") != 0) return false;

  if (cfg->data_bits < 7 || cfg->data_bits > 8) return false;
  if (cfg->stop_bits != 1 && cfg->stop_bits != 2) return false;

  uart_deinit(SERIAL_UART_ID);
  uart_init(SERIAL_UART_ID, cfg->baud);
  uart_set_hw_flow(SERIAL_UART_ID, false, false);
  uart_set_format(SERIAL_UART_ID, cfg->data_bits, cfg->stop_bits, parity_mode);
  gpio_set_function(SERIAL_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(SERIAL_RX_PIN, GPIO_FUNC_UART);
  uart_set_fifo_enabled(SERIAL_UART_ID, true);
  g_uart_ready = true;
  return true;
}

static void send_cfg(void) {
  char out[192];
  snprintf(out, sizeof(out),
           "{\"type\":\"uart.config\",\"port\":\"uart0\",\"baud\":%lu,\"data_bits\":%u,"
           "\"parity\":\"%s\",\"stop_bits\":%u,\"flow\":\"%s\"}",
           (unsigned long)g_cfg.baud, g_cfg.data_bits, g_cfg.parity, g_cfg.stop_bits, g_cfg.flow);
  send_json(out);
}

static void on_open(ws_conn_t *conn) {
  if (g_client && g_client != conn) {
    ws_conn_close(g_client);
  }
  g_client = conn;
  send_json("{\"type\":\"hello\",\"fw\":\"0.1.0\",\"caps\":[\"uart0\"]}");
}

static void on_close(ws_conn_t *conn) {
  if (g_client == conn) g_client = NULL;
}

static void on_binary(ws_conn_t *conn, const uint8_t *data, uint16_t len) {
  (void)conn;
  if (!g_uart_ready) return;
  for (uint16_t i = 0; i < len; i++) uart_putc_raw(SERIAL_UART_ID, data[i]);
}

static void handle_term_write(const char *json) {
  char buf[128];
  if (!extract_str(json, "data", buf, sizeof(buf))) {
    send_json("{\"type\":\"error\",\"code\":\"BAD_REQUEST\",\"msg\":\"missing data\"}");
    return;
  }

  if (!g_uart_ready) return;
  for (size_t i = 0; i < strlen(buf); i++) uart_putc_raw(SERIAL_UART_ID, (uint8_t)buf[i]);
}

static void handle_set_cfg(const char *json) {
  uart_cfg_t next = g_cfg;
  uint32_t tmp = 0;
  char parity[8];

  if (extract_u32(json, "baud", &tmp)) next.baud = tmp;
  if (extract_u32(json, "data_bits", &tmp)) next.data_bits = (uint8_t)tmp;
  if (extract_u32(json, "stop_bits", &tmp)) next.stop_bits = (uint8_t)tmp;
  if (extract_str(json, "parity", parity, sizeof(parity))) {
    strncpy(next.parity, parity, sizeof(next.parity) - 1);
    next.parity[sizeof(next.parity) - 1] = 0;
  }

  if (!apply_uart_cfg(&next)) {
    send_json("{\"type\":\"error\",\"code\":\"BAD_CONFIG\",\"msg\":\"unsupported uart config\"}");
    return;
  }

  g_cfg = next;
  send_cfg();
}

static void on_text(ws_conn_t *conn, const uint8_t *data, uint16_t len) {
  char msg[512];
  (void)conn;

  if (len >= sizeof(msg)) {
    send_json("{\"type\":\"error\",\"code\":\"TOO_LARGE\",\"msg\":\"message too large\"}");
    return;
  }

  memcpy(msg, data, len);
  msg[len] = 0;

  if (strstr(msg, "\"type\":\"hello\"")) {
    send_json("{\"type\":\"hello\",\"fw\":\"0.1.0\",\"caps\":[\"uart0\"]}");
    return;
  }

  if (strstr(msg, "\"type\":\"uart.get_config\"")) {
    send_cfg();
    return;
  }

  if (strstr(msg, "\"type\":\"uart.set_config\"")) {
    handle_set_cfg(msg);
    return;
  }

  if (strstr(msg, "\"type\":\"term.write\"")) {
    handle_term_write(msg);
    return;
  }

  send_json("{\"type\":\"error\",\"code\":\"UNKNOWN\",\"msg\":\"unknown command\"}");
}

const ws_app_handler_t *serial_app_handler(void) {
  static ws_app_handler_t handler = {
    .on_open = on_open,
    .on_close = on_close,
    .on_text = on_text,
    .on_binary = on_binary,
  };

  if (!g_uart_ready) apply_uart_cfg(&g_cfg);
  return &handler;
}

void serial_app_poll(void) {
  if (!g_client || !g_uart_ready) return;
  if (!uart_is_readable(SERIAL_UART_ID)) return;

  uint8_t tmp[RX_TMP_BUF_LEN];
  size_t n = 0;

  while (n < RX_TMP_BUF_LEN && uart_is_readable(SERIAL_UART_ID)) {
    tmp[n++] = uart_getc(SERIAL_UART_ID);
  }

  if (n > 0) ws_conn_send_binary(g_client, tmp, (uint16_t)n);
}

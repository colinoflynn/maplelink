#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct ws_conn ws_conn_t;

typedef struct {
  void (*on_open)(ws_conn_t *conn);
  void (*on_close)(ws_conn_t *conn);
  void (*on_text)(ws_conn_t *conn, const uint8_t *data, uint16_t len);
  void (*on_binary)(ws_conn_t *conn, const uint8_t *data, uint16_t len);
} ws_app_handler_t;

bool web_server_start(const ws_app_handler_t *handler);
bool ws_conn_send_text(ws_conn_t *conn, const uint8_t *data, uint16_t len);
bool ws_conn_send_binary(ws_conn_t *conn, const uint8_t *data, uint16_t len);
void ws_conn_close(ws_conn_t *conn);
void ws_conn_close_with_reason(ws_conn_t *conn, uint16_t code, const char *reason);

#endif

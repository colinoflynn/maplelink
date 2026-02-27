#ifndef PROTO_UART_H
#define PROTO_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "web_server.h"

void proto_uart_init(void);
void proto_uart_on_client_open(ws_conn_t *conn);
void proto_uart_on_client_close(ws_conn_t *conn);
bool proto_uart_handle_text(const char *type, const char *json);
bool proto_uart_handle_binary(const uint8_t *data, uint16_t len);
void proto_uart_poll(void);

#endif

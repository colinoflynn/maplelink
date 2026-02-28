#ifndef APP_TRANSPORT_H
#define APP_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

#include "web_server.h"

bool app_send_text(const char *text);
bool app_send_binary(const uint8_t *data, uint16_t len);
ws_conn_t *app_current_client(void);
uint8_t app_debug_level(void);
bool app_debug_log(uint8_t level, const char *scope, const char *msg);

#endif

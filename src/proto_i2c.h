#ifndef PROTO_I2C_H
#define PROTO_I2C_H

#include <stdbool.h>

#include "web_server.h"

void proto_i2c_init(void);
void proto_i2c_on_client_open(ws_conn_t *conn);
void proto_i2c_on_client_close(ws_conn_t *conn);
bool proto_i2c_handle_text(const char *type, const char *json);
void proto_i2c_poll(void);

#endif

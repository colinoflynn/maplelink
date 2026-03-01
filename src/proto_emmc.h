#ifndef PROTO_EMMC_H
#define PROTO_EMMC_H

#include <stdbool.h>

#include "web_server.h"

void proto_emmc_init(void);
void proto_emmc_on_client_open(ws_conn_t *conn);
void proto_emmc_on_client_close(ws_conn_t *conn);
bool proto_emmc_handle_text(const char *type, const char *json);
void proto_emmc_poll(void);

#endif

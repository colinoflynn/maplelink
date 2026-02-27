#ifndef PROTO_SPI_H
#define PROTO_SPI_H

#include <stdbool.h>

#include "web_server.h"

void proto_spi_init(void);
void proto_spi_on_client_open(ws_conn_t *conn);
void proto_spi_on_client_close(ws_conn_t *conn);
bool proto_spi_handle_text(const char *type, const char *json);
void proto_spi_poll(void);

#endif

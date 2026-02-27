#include "proto_spi.h"

#include <stdio.h>
#include <string.h>

#include "app_transport.h"

static bool starts_with(const char *s, const char *prefix) {
  size_t n = strlen(prefix);
  return strncmp(s, prefix, n) == 0;
}

void proto_spi_init(void) {}

void proto_spi_on_client_open(ws_conn_t *conn) {
  (void)conn;
}

void proto_spi_on_client_close(ws_conn_t *conn) {
  (void)conn;
}

bool proto_spi_handle_text(const char *type, const char *json) {
  (void)json;

  if (!starts_with(type, "spi.")) return false;

  if (strcmp(type, "spi.get_config") == 0) {
    return app_send_text("{\"type\":\"spi.config\",\"port\":\"spi0\",\"mode\":0,\"freq\":1000000,"
                         "\"bits\":8,\"cs\":\"manual\",\"status\":\"stub\"}");
  }

  if (strcmp(type, "spi.dump.start") == 0) {
    return app_send_text("{\"type\":\"spi.dump.status\",\"port\":\"spi0\",\"state\":\"not_implemented\"}");
  }

  if (strcmp(type, "spi.dump.stop") == 0) {
    return app_send_text("{\"type\":\"spi.dump.status\",\"port\":\"spi0\",\"state\":\"stopped\"}");
  }

  return app_send_text("{\"type\":\"error\",\"code\":\"SPI_UNSUPPORTED\",\"msg\":\"spi command not implemented\"}");
}

void proto_spi_poll(void) {}

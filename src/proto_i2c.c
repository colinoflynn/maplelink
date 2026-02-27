#include "proto_i2c.h"

#include <string.h>

#include "app_transport.h"

static bool starts_with(const char *s, const char *prefix) {
  size_t n = strlen(prefix);
  return strncmp(s, prefix, n) == 0;
}

void proto_i2c_init(void) {}

void proto_i2c_on_client_open(ws_conn_t *conn) {
  (void)conn;
}

void proto_i2c_on_client_close(ws_conn_t *conn) {
  (void)conn;
}

bool proto_i2c_handle_text(const char *type, const char *json) {
  (void)json;

  if (!starts_with(type, "i2c.")) return false;

  if (strcmp(type, "i2c.get_config") == 0) {
    return app_send_text("{\"type\":\"i2c.config\",\"port\":\"i2c0\",\"freq\":100000,\"status\":\"stub\"}");
  }

  return app_send_text("{\"type\":\"error\",\"code\":\"I2C_UNSUPPORTED\",\"msg\":\"i2c command not implemented\"}");
}

void proto_i2c_poll(void) {}

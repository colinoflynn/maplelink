#include "serial_app.h"

#include "app_router.h"

const ws_app_handler_t *serial_app_handler(void) {
  return app_router_handler();
}

void serial_app_poll(void) {
  app_router_poll();
}

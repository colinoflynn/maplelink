#ifndef APP_ROUTER_H
#define APP_ROUTER_H

#include "web_server.h"

const ws_app_handler_t *app_router_handler(void);
void app_router_poll(void);

#endif

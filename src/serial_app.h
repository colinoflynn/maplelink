#ifndef SERIAL_APP_H
#define SERIAL_APP_H

#include "web_server.h"

const ws_app_handler_t *serial_app_handler(void);
void serial_app_poll(void);

#endif

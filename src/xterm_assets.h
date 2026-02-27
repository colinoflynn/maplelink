#ifndef XTERM_ASSETS_H
#define XTERM_ASSETS_H

#include <stdbool.h>

/* Returns true if an embedded xterm asset is available for `name`. */
bool xterm_asset_get(const char *name, const char **data, int *len, const char **content_type);

#endif

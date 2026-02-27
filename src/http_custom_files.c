#include <string.h>

#include "index_html_data.h"
#include "lwip/apps/fs.h"
#include "xterm_assets.h"

int fs_open_custom(struct fs_file *file, const char *name) {
  const char *asset_data = NULL;
  const char *asset_type = NULL;
  int asset_len = 0;

  if (!file || !name) return 0;

#if defined(BROWSERIO_XTERM_SOURCE_LOCAL) && BROWSERIO_XTERM_SOURCE_LOCAL
  if (xterm_asset_get(name, &asset_data, &asset_len, &asset_type)) {
    (void)asset_type;
    file->data = asset_data;
    file->len = asset_len;
    file->index = file->len;
    file->flags = 0;
#if LWIP_HTTPD_FILE_EXTENSION
    file->pextension = NULL;
#endif
#if HTTPD_PRECALCULATED_CHECKSUM
    file->chksum = NULL;
    file->chksum_count = 0;
#endif
#if LWIP_HTTPD_FILE_STATE
    file->state = NULL;
#endif
    return 1;
  }
#endif

  if (strcmp(name, "/") == 0 || strcmp(name, "/index.html") == 0) {
    file->data = BROWSERIO_INDEX_HTML;
    file->len = BROWSERIO_INDEX_HTML_LEN;
    file->index = file->len;
    file->flags = 0;
#if LWIP_HTTPD_FILE_EXTENSION
    file->pextension = NULL;
#endif
#if HTTPD_PRECALCULATED_CHECKSUM
    file->chksum = NULL;
    file->chksum_count = 0;
#endif
#if LWIP_HTTPD_FILE_STATE
    file->state = NULL;
#endif
    return 1;
  }

  return 0;
}

void fs_close_custom(struct fs_file *file) { LWIP_UNUSED_ARG(file); }

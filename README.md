# pico-browserio

## Web UI workflow

- Edit the web page in `src/web/index.html`.
- During build, CMake runs `tools/embed_html.py` automatically to generate:
  - `build/generated/index_html_data.h`
- Firmware serves the generated HTML via `src/http_custom_files.c`.

### xterm source mode

Use the CMake cache option:

- `BROWSERIO_XTERM_SOURCE=CDN` (default): loads xterm from CDN URLs.
- `BROWSERIO_XTERM_SOURCE=LOCAL`: expects local `/xterm.js` and `/xterm.css` assets via `xterm_asset_get`.

Example:

```bash
cmake -S . -B build -DBROWSERIO_XTERM_SOURCE=LOCAL
cmake --build build
```

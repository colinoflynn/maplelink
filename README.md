# MapleLink: USB Serial + SPI Dumper with a (Offline) Web Interface




## Detailed Features

* Webpage served over USB is the UI, nothing to install on host computers if USB-Ethernet support available (works in Win 11, Linux, Mac probably)
* UART terminal features:
  * ASCII, hex mode, and terminal (vt100) support.
  * Can tx hex blocks or specific text blocks.
  * Can send characters you type only after enter or or as you type.
  * Download logs
* SPI Flash general features:
  * Little ASCII-art of SOIC8 pinout
  * Pin activity monitor you can use to see if system is accessing SPI
  * Basic sniffer (doesn't keep up with long streams)
  * ID-poll loop that looks for SPI access between reading, used to see if you have full control of SPI
  * Minimal SFDP read/decoding, and status register read/decoding
* SPI Flash dumping features:
  * Supports 3 & 4-byte addressing, hopefully detects correctly which to use
  * Option to double-read each block to hopefully detect errors, aborts or retires if this happens
  * Can display on screen or dump to binary file. Preview data as it's dumping. Partial dumps are saved if you abort.
  * Fast enough to not be infuriating (under 2 mins for 8MB flash on my computer)
* SPI RAW features:
  * Send arbitrary SPI commands to a device, including extra padding FF's
  * Lets you set 10 quick-access fields to transmit different commands (can change the number in html file)
* Re-enable R-Pi bootloader for easy development without touching a USB cable
* Front-end written by AI, I have no idea what it does or how it works (as god intended)



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

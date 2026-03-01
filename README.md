# MapleLink: USB Serial + SPI Dumper with a (Offline) Web Interface

Do you want to do some basic hardware hacking, but don't want to deal with installing tools? This project makes it (somewhat) easy to get into hardware
hacking, requiring only a R-Pi Pico and some wires (and probably logic clips).

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
  * Option to double-read each block to hopefully detect errors, aborts or retires if this happens with status
  * Can display on screen or dump to binary file. Preview data as it's dumping. Partial dumps can be saved if you abort.
  * Fast enough to not be infuriating for smaller flash sizes (see table below)
* SPI RAW features:
  * Send arbitrary SPI commands to a device, including extra padding FF's
  * Lets you set 10 quick-access fields to transmit different commands (can change the number in html file)
* Re-enable R-Pi bootloader for easy development without touching a USB cable
* Front-end and most processing code written by AI, I have no idea what it does or how it works (as god intended)

### Speed Examples

Some test on my computer:

| Flash size | SCK   | Double-read on? | Time (MM:SS) |
|------------|-------|-----|------|
| 32 MBit    | 1 MHz | No  | 0:47 |
| 32 MBit    | 1 MHz | Yes | 1:05 |
| 32 MBit    | 5 MHz | No  | 2:13 |
| 32 MBit    | 5 MHz | Yes | 1:06 |
| 512 MBit   | 5 MHz | No  | 8:50 |
| 512 MBit   | 5 MHz | Yes | 17:50 |
| 512 MBit   | 1 MHz | No  | 12:32 |
| 512 MBit   | 1 MHz | Yes | 35:34 |


## Usage Directions

## Developer Notes

Some notes for either the human or clanker working on this thing.

## Building Firmware

Firmware follows normal R-Pi Pico development, with a SDK externally held:

1. Clone the directly somewhere.
2. Clone the [pico-sdk](https://github.com/raspberrypi/pico-sdk) or download a release (if cloning be sure to init/update submodules - this project was developed on 2.2.0)

```
cmake -B build -G "Ninja" -DPICO_SDK_PATH="/path/to/pico-sdk"
cd build
cmake --build .
```

If you need some of the SDK tools, you can get pre-build versions from the [pico-sdk-tools](https://github.com/raspberrypi/pico-sdk-tools) repository, make sure they match your SDK version.

On Windows, you will likely want to download picotool (must be the SAME version as the SDK) and pass the directory with `-Dpicotool_DIR="/dir/to/picotool` for the first `cmake`. You can also pass paths to the compilers directly, in case you are using older compilers (e.g., like you got from the R-Pi Windows SDK) with a current pico-sdk. Here is both of those shown:

```
cmake -B build -G "Ninja" -DPICO_SDK_PATH="C:/dev/pico_sdk-src" -DCMAKE_C_COMPILER="C:/devtools/Pico SDK v1.5.1/gcc-arm-none-eabi/bin/arm-none-eabi-gcc.exe" -DCMAKE_CXX_COMPILER="C:/devtools/Pico SDK v1.5.1/gcc-arm-none-eabi/bin/arm-none-eabi-g++.exe" -Dpicotool_DIR="c:/devtools/picotool"
```

If your build fails when trying to build `picotool` this is because it did not find a matching version, and is attempting to build it. On Windows you may not have a regular C/C++ compiler so this will fail. If you needed to use `pioasm` you would see a similar failure (this project does not require pioasm currently as does not use the PIO blocks).


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

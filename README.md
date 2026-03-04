# MapleLink: USB Serial + SPI & slow eMMC Dumper with a (Offline) Web Interface

Do you want to do some basic hardware hacking, but don't want to deal with installing tools? This project makes it (somewhat) easy to get into hardware
hacking, requiring only a R-Pi Pico and some wires (and probably logic clips). You can even use your phone instead of a computer! Perfect for hardware hacking on the go.

## Detailed Features

* Webpage served over USB is the UI, nothing to install on host computers if USB-Ethernet support available (works in Win 11, Linux, Mac probably, Android)
* UART terminal features:
  * ASCII, hex mode, and terminal (vt100) support.
  * Can tx hex blocks or specific text blocks.
  * Can send characters you type only after enter or or as you type.
  * Hot pink & rainbow colour options.
  * Download logs
* SPI Flash general features:
  * Little ASCII-art of SOIC8 pinout
  * Pin activity monitor you can use to see if system is accessing SPI
  * Basic sniffer (doesn't keep up with long streams)
  * ID-poll loop that looks for SPI access between reading, used to see if you have full control of SPI
  * Minimal SFDP read/decoding, and status register read/decoding
* SPI Flash dumping features:
  * Supports 3 & 4-byte addressing, hopefully detects correctly which to use
  * Option to double-read each block to hopefully detect errors, aborts or retries if this happens with status
  * Can display on screen or dump to binary file. Preview data as it's dumping. Partial dumps can be saved if you abort
  * Fast enough to not be infuriating for smaller flash sizes (see table below)
* SPI RAW features:
  * Send arbitrary SPI commands to a device, including extra padding FF's
  * Lets you set 10 quick-access fields to transmit different commands (can change the number in html file)
* eMMC Features:
  * Reads ID, sizes
  * Scan for empty/non-empty areas to try and avoid wasting time dumping large partitions
  * Infuriatingly slow dumping of user partition that you don't want to use for the full 4Gb flash
  * Generate .gz (browser dependent)
  * Extra-questionable code quality
* Re-enable R-Pi bootloader for easy development without touching a button
* Front-end and much of the firmware code written by AI, I have no idea what it does or how it works (as god intended)
* Change between Dark Mode, Pink Mode, and MS-Dos 6.22 Mode Themes
* Dec/Hex converter on every page
* Randomly disconnects or times out sometimes (surprise feature)

Note vt100 terminal mode requires internet to get the vt100 processing code, this can be moved onto the R-Pi as well but by
default I did not for space reasons (there is a stub to copy it to).

### Speed Examples

Some test on my computer for SPI:

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

### Setup

USB Ethernet is a beautiful idea with an unfortunute history of quasi-standards and different implementations (I even wrote such firmware [a long time ago](https://github.com/contiki-os/contiki/blob/master/cpu/avr/dev/usb/rndis/rndis.c) so I have the scars). Luckily this  better nowadays, but there are still TWO different imeplementations you can try:

* `maplelink-X.X-ncm.uf2`
* `maplelink-X.X-rndis-ecm.uf2`

Try the `ncm` one first - this is a more recent USB ethernet standard.

The `rndis-ecm` tries two other standards: RNDIS is a Microsoft 'standard' (but some other devices work with it?), and ECM is an older standard. I may need to release a third version without RNDIS for certain things, but will wait to see if that is needed.

Drag & drop the `.uf2` file onto a R-Pi pico in bootloader mode.

I didn't need a driver for `ncm` in Windows 11, but I can provide signed drivers if required, open an issue if this is the problem.

### Web

The current firmware is hard-coded to serve a webpage on [http://192.168.7.1](http://192.168.7.1). Your computer should get an IP automatically (`192.168.7.2`). If you can't access the webpage, check in order:

* Check you have a new network interface - it might get a weird name, on Windows it shows up under "Network Interface"
* Check you got an IP assigned from this interface

Local settings like VPN enforcing tunneling may prevent access to the network.

### LED Troubleshooting

The LED on the R-Pi Pico has 3 states:

* Fast blinking (3-5 blinks per second) = power but no USB action yet
* Fast blinking but in blink / blink / off / off pattern = enumeration failed
* Slow blinking (1-2 blinks per second) = enumeration successful, no webpage connection
* Solid = webpage connection

You should get slow blinking until you access the website.

If you are unsure about slow vs. fast blinking, plug the USB into a power adapter instead of a computer. This
will trigger the fast blinking mode.

### Phone Usage

So far tested on Pixel 8A running Android 16:

* Used NCM firmware.
* Plugged into a basic USC-C hub (just 4x USB-A ports) to go USB-C --> USB-A --> Micro-USB A.
* I had to *TURN OFF* both Wi-Fi & Mobile Internet for it to use the USB-Ethernet connection. Try at least turning off Wi-Fi. Airplane mode alone didn't work either.

Search "USB Ethernet Adapter" for your specific phone model for any device-specific information. It seemed people reported varying degrees of success with needing to turn off other data (wifi etc).

According to [https://www.macrumors.com/2023/09/22/iphone-15-usb-c-to-ethernet/](macrumors.com) iPhone 15 and later support USB Ethernet adapters.

You may want to use a R-Pi Pico clone with a USB-C port to make this less painful physically (or get a forbidden USB-C to Micro-USB cable).

### Terminal Usage

1. Open the **Terminal** tab.
2. Set serial parameters:
   - `Baud`, `Data`, `Parity`, `Stop`, `Enter`.
3. Choose send mode:
   - `as you type`: sends each character immediately.
   - `on Enter only`: buffers until Enter.
4. Choose decode mode:
   - `plain`: raw text.
   - `ansi colors`: interprets ANSI color escapes.
   - `vt100 (xterm.js)`: full terminal emulation (requires internet by default).
5. Optional display controls:
   - `Terminal Theme`: `default`, `matrix`, `rainbow`, `pink`, `hot pink`.
   - `hex view`: enables a hex view of incoming data
   - `Hex cols`: defines the hex width
   - `Hex gap ms`: After this many ms of no data, will make a new line in the hex view (to help with packets)
   - `Backlog KB`: limits log size
   - `Disable Autoscroll`: stops terminal from jumping to bottom
   - `local echo`: normal terminal stuff
6. Send data:
   - `Send Text` for plain text, will interperate `\n`, `\r`, `\t`, `\0`, `\b`, `\f`, `\v`, `\\`, `\"`, `\'`, `\xNN` (byte), `\uNNNN` and `\u{...}` (Unicode)
   - `Send Bytes` for hex byte sequences (`01 02 FF` or `0x41,0x42`).
   Both of these save recently sent.
7. Logs:
   - `Download Text Log` exports terminal text.
   - `Download Raw Binary` exports raw RX bytes.

### SPI FLASH Interface

#### Detecting Conflicting Access

1. In **Pin Monitor**, run `Start Pin Detect` to view MOSI/MISO/CS/CLK activity while system is running.
2. Stop the **Pin Monitor**
2. In **ID Poll**:
   - Click `Monitor activity between polls`.
   - Click `Start ID Poll`.
3. Watch:
   - `Between Polls` and `Window Activity` fields.
   - `INTERFERENCE` indicates external activity on the bus.

If it is reading a valid ID and says *CLEAR* between reads, this indicates you might have a good link.

**NOTE**: The frequency display is not accurate as just polls, but gives some idea of if things are busier or less busy, so have left the numeric display in.

#### Basic Dumping

1. Setup link as above.
2. Perform ID Poll & SFDP read to get flash information.
3. This should auto-fill the "Dump Length", and detect if `Addr Bytes` (`3` or `4`) is needed.
4. Set `Double-read verify` if you want a more reliable dump - this will attempt each read twice. Set the number of
   `Automatic Retries` if you want it to also retry on failed (instead of aborting).
5. For on-screen preview only (recommended to check data looks reasonable).
   - Use **Display**: set `Display Length`, click `Start Display`.
5. For capture:
   - In **Dump to file**, set `Filename Prefix` and `Dump Length` (should be auto-filled).
   - Click `Start Dump`.
   - Click `Stop Dump` if needed (partial data is still kept in UI memory).
   - Check the full dump is done (shows as complete), otherwise check if there was a read error.
   - Click `Download Bin`.

#### SPI SFDP / Status / Sniffer

1. In SPI tab, use `Read SFDP` to parse density/address mode/read command hints.
2. Use `Read Status Register` to read/decode SR1/SR2/SR3 and common protection bits.
3. `Start Sniffer` captures SPI transactions - this samples both MOSI & MISO, so it uses PIO instead of the SPI hardware blocks. This is not a well tested area, it won't keep up with long data stretches and is likely to miss data.

#### ⚠ PIN2PWN Mode ⚠

⚠ ⚠ WARNING: HIGH RISK OF PERMANENT AND NON-OBVIOUS PIN DAMAGE TO PICO ⚠ ⚠

PIN2PWN mode will look for a trigger of activity on the SPI lines, then drive them high from the R-Pi Pico. You select one line to serve as the trigger, which can see a specific number of transitions.

If you blow up your pins, you can change them in [src/proto_spi.c](src/proto_spi.c). For example here are some other good pins:

```
#define SPI_MISO_PIN 16
#define SPI_CS_PIN 17
#define SPI_SCK_PIN 18
#define SPI_MOSI_PIN 19
```

I found that some devices would quickly cause the R-Pico pins to be damaged from conflicting. This manifested as unreliable SPI communication after trying PIN2PWN, so it's not obvious but results in frustrating work later.

More ideally you would drive the SPI line with a transistor connected instead - this would result in pulling the pin *down*, but that will still 100% work.

### SPI Raw Packet Mode

1. Open **SPI RAW** tab.
2. Set SPI speed with `Apply Speed`.
3. Use command rows to send arbitrary SPI byte sequences (with optional extra clocks/padding as configured in UI).
4. Read TX/RX in the raw log area.

### eMMC Mode

eMMC mode uses only three pins on the eMMC, which limits data speed greatly:

 * D0
 * CMD
 * CLK

I was able to get around 200 KB/s dumping speed, which means you won't want to dump the full files. The device
implements some searching modes to try and find more interesting (non-empty) areas so you can dump specific areas to decide how much effort you want to put into dumping the full flash.

#### Safety / Pin checks / ID

1. Open **eMMC** tab.
2. In **Safety and Link**:
   - Set `Tri-state by default`, `Pull-ups enabled`.
   - Set `Speed`, `Retry Idle`.
   - Click `Apply I/O Safety` and `Apply Speed`.
3. Use:
   - `Start Pin Detect` for raw pin activity.
   - `CMD Test Waveform`, `CMD1 Sequence Test`, `SD Init Test` for link-level checks.
4. Use `Start ID Poll` to repeatedly read ID/OCR/CID/CSD.

#### Layout

1. Click `Read Layout` to fetch EXT_CSD layout fields.
2. Use `Full Dump Hint` / `sec_count` output to estimate full user-area size.

#### Dump

1. Configure:
   - `Filename Prefix`
   - `Start LBA`
   - `Blocks`
   - `Chunk`
   - `Double-read verify`
   - `Verify retries`
   - `Auto retries`
2. Click `Start Dump`.
3. Monitor:
   - Progress line (bytes + current LBA)
   - Rate/Avg/ETA
   - Error counters (`CRC`, `no-R1`, `resync`, `tx-busy`)
4. Click `Stop Dump` for partial capture if needed.
5. Download:
   - `Download Bin` for raw binary.
   - `Download GZip` for compressed binary (browser-dependent support/performance).
6. eMMC filename format:
   - `PREFIXSTARTLBA_ENDLBA.bin` (or `.bin.gz`)

#### Scan Memory

1. Configure:
   - `Start LBA`
   - `Sample Every n=` (sampling stride is `n+1` blocks)
   - `Total Samples`
   - `Min Span (blocks)` display filter
2. Click `Start Scan`.
3. Table outputs grouped ranges:
   - `Type`: `empty` or `non-empty`
   - `Start LBA`, `End LBA`, `Span`, `Size`
4. `Min Span` only filters what is shown; it does not change collected data.
5. `Clear Results` clears the table view.

#### Find Start

1. Configure:
   - `Start LBA`
   - `Max Search Length (Blocks)`
2. Click `Start Find`.
3. It scans forward and stops at first non-empty block, or reports not found.
4. Status line shows running progress and final result.

### Debug / About

1. Open **DEBUG** tab.
2. Set debug level (`0..3`) with `Apply Debug Level`.
3. Use `Read Debug Level` to confirm runtime level.
4. `Clear Debug Log` clears UI debug output.
5. `Enter BOOTSEL` reboots Pico into USB bootloader mode.
6. About panel includes firmware/build timestamp and project links.
7. In `Network`, you can set `Device IP` and DHCP range (`DHCP Start`/`DHCP End`).
8. Applying network config saves settings in flash and reboots device.
9. Stored network config is invalidated automatically when firmware build changes.

## Developer Notes

Some notes for either the human or clanker working on this thing.

### Building Firmware

Firmware follows normal R-Pi Pico development, with a SDK externally held:

1. Clone the [pico-sdk](https://github.com/raspberrypi/pico-sdk) or download a release (if cloning be sure to init/update submodules - this project was developed on 2.2.0)
2. Clone the repo somewhere for this project or download a zip-file. Then in the root of that folder run:

```
cmake -B build -G "Ninja" -DPICO_SDK_PATH="/path/to/pico-sdk"
cd build
cmake --build .
```

This should make a `.uf2` you can drag onto the R-Pi Pico in bootloader mode.

If you need some of the SDK tools, you can get pre-build versions from the [pico-sdk-tools](https://github.com/raspberrypi/pico-sdk-tools) repository, make sure they match your SDK version.

On Windows, you will likely want to download picotool and pioasm (must be the SAME version as the SDK) and pass the directory with `-Dpicotool_DIR="/dir/to/picotool` and `-Dpioasm_DIR="/dir/to/pioasm` for the first `cmake`. You can also pass paths to the compilers directly, in case you are using older compilers (e.g., like you got from the R-Pi Windows SDK) with a current pico-sdk. Here is both of those shown:

```
cmake -B build -G "Ninja" -DPICO_SDK_PATH="C:/dev/pico_sdk-src" -DCMAKE_C_COMPILER="C:/devtools/Pico SDK v1.5.1/gcc-arm-none-eabi/bin/arm-none-eabi-gcc.exe" -DCMAKE_CXX_COMPILER="C:/devtools/Pico SDK v1.5.1/gcc-arm-none-eabi/bin/arm-none-eabi-g++.exe" -Dpicotool_DIR="c:/devtools/picotool" -Dpioasm_DIR="c:/devtools/pioasm"
```

If your build fails when trying to build `picotool` this is because it did not find a matching version, and is attempting to build it. On Windows you may not have a regular C/C++ compiler so this will fail. The `pioasm` tool is needed for the PIO blocks (note the tool is pioasm, NOT picoasm). When passing the directories if using pre-built you must pass the path to the folder with the tool, NOT the path to the binary itself.

### eMMC Improvements

The eMMC code is needlessly slow, in particular I think some of the bit counts are off. I manually fixed some items but it's also uses some extra clock cycles around to get this working.

### Web UI workflow

- Edit the web page in `src/web/index.html`.
- During build, CMake runs `tools/embed_html.py` automatically to generate:
  - `build/generated/index_html_data.h`
- Firmware serves the generated HTML via `src/http_custom_files.c`.

#### xterm source mode

Use the CMake cache option:

- `BROWSERIO_XTERM_SOURCE=CDN` (default): loads xterm from CDN URLs.
- `BROWSERIO_XTERM_SOURCE=LOCAL`: expects local `/xterm.js` and `/xterm.css` assets via `xterm_asset_get`.


# MapleLink Python Interface

This folder provides a small Python client for MapleLink:

- Serial access using a pyserial-like API over WebSocket transport.
- SPI flash control (ID read, manual TX/RX, dump address range).
- eMMC control (CMD1 check, ID read, layout read, dump LBA range).

## Install

```bash
pip install -r python/requirements.txt
```

## Quick Start

```python
from maplelink import MapleLinkControlClient, MapleLinkSerial

# 1) Serial (pyserial-style API over MapleLink WebSocket)
ser = MapleLinkSerial("192.168.7.1", baudrate=115200, timeout=1)
ser.write(b"help\r\n")
print(ser.readline())
ser.close()

# 2/3) SPI + eMMC over MapleLink WebSocket control channel
with MapleLinkControlClient(host="192.168.7.1") as ml:
    spi_id = ml.spi.read_id()
    print("SPI ID:", spi_id)

    raw = ml.spi.raw_txrx("9F", ff_tail=3)
    print("RAW:", raw)

    spi_dump = ml.spi.dump(start_addr=0, length_bytes=4096)
    print("SPI dump bytes:", len(spi_dump.data), "state:", spi_dump.state)

    cmd1 = ml.emmc.cmd1_check()
    print("CMD1:", cmd1)

    emmc_id = ml.emmc.id_read()
    print("eMMC ID:", emmc_id)

    layout = ml.emmc.layout_read()
    print("Layout:", layout)

    emmc_dump = ml.emmc.dump(start_lba=0, block_count=16)
    print("eMMC dump bytes:", len(emmc_dump.data), "state:", emmc_dump.state)
```

## Notes

- `MapleLinkSerial` is pyserial-like at the API level, but it does not use a local COM port.
  It uses `ws://<device>:81/ws` and the MapleLink UART binary channel.
- Control API uses the same JSON/binary protocol as the web UI (`ws://<device>:81/ws`).
- Dump APIs return `MapleLinkDumpResult` with status and collected bytes.
- For SPI/eMMC operations, ensure nothing else is concurrently controlling the same MapleLink device.

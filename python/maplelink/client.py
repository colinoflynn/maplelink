import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import websocket

try:
    import serial  # type: ignore
except Exception:  # pragma: no cover - fallback when pyserial is not installed
    class _SerialFallback:  # minimal compatibility for exception types used below
        class SerialException(Exception):
            pass

        class SerialTimeoutException(TimeoutError):
            pass

        class PortNotOpenError(SerialException):
            pass

    serial = _SerialFallback()  # type: ignore


WS_BIN_MAGIC = 0xB0
WS_CH_UART = 0x01
WS_CH_SPI_DUMP_DATA = 0x02
WS_CH_SPI_DUMP_FF = 0x03
WS_CH_EMMC_DUMP_DATA = 0x04
WS_CH_EMMC_DUMP_RUN = 0x05


class MapleLinkSerial:
    """
    pyserial-like API over MapleLink WebSocket UART transport.

    This class intentionally mirrors common pyserial methods/properties:
    open, close, read, readline, read_until, write, flush,
    reset_input_buffer, in_waiting, is_open, baudrate, timeout.
    """

    def __init__(
        self,
        port: str = "192.168.7.1",
        baudrate: int = 115200,
        timeout: Optional[float] = 1.0,
        write_timeout: Optional[float] = None,
        bytesize: int = 8,
        parity: str = "N",
        stopbits: int = 1,
        ws_port: int = 81,
        auto_open: bool = True,
    ):
        self.port = port
        self.ws_port = int(ws_port)
        self.baudrate = int(baudrate)
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.bytesize = int(bytesize)
        self.parity = str(parity).upper()
        self.stopbits = int(stopbits)
        self._ws: Optional[websocket.WebSocket] = None
        self._rx = bytearray()
        self._is_open = False
        self._last_json: Optional[Dict[str, Any]] = None
        if auto_open:
            self.open()

    @property
    def is_open(self) -> bool:
        return self._is_open

    @property
    def in_waiting(self) -> int:
        return len(self._rx)

    @property
    def last_json(self) -> Optional[Dict[str, Any]]:
        return self._last_json

    def open(self) -> None:
        if self._is_open:
            return
        try:
            self._ws = websocket.create_connection(f"ws://{self.port}:{self.ws_port}/ws", timeout=self.timeout or 5.0)
        except Exception as exc:
            raise serial.SerialException(f"Failed to open MapleLink WS serial on {self.port}: {exc}") from exc
        self._is_open = True
        self._send_json({"type": "hello"})
        self._send_json({"type": "uart.get_config", "port": "uart0"})
        self.apply_uart_config()

    def close(self) -> None:
        if self._ws is not None:
            self._ws.close()
        self._ws = None
        self._is_open = False

    def __enter__(self) -> "MapleLinkSerial":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _ensure_open(self) -> websocket.WebSocket:
        if not self._is_open or self._ws is None:
            raise serial.PortNotOpenError()
        return self._ws

    def _send_json(self, msg: Dict[str, Any]) -> None:
        ws = self._ensure_open()
        ws.send(json.dumps(msg))

    def apply_uart_config(self) -> None:
        parity_map = {"N": "none", "E": "even", "O": "odd"}
        self._send_json(
            {
                "type": "uart.set_config",
                "port": "uart0",
                "baud": int(self.baudrate),
                "data_bits": int(self.bytesize),
                "parity": parity_map.get(self.parity, "none"),
                "stop_bits": int(self.stopbits),
                "flow": "none",
            }
        )

    def _recv_once(self, timeout: float) -> None:
        ws = self._ensure_open()
        prev = ws.gettimeout()
        ws.settimeout(timeout)
        try:
            raw = ws.recv()
        except websocket.WebSocketTimeoutException:
            return
        finally:
            ws.settimeout(prev)

        if isinstance(raw, str):
            try:
                self._last_json = json.loads(raw)
            except Exception:
                self._last_json = {"type": "text", "raw": raw}
            return

        data = bytes(raw)
        if len(data) >= 2 and data[0] == WS_BIN_MAGIC and data[1] == WS_CH_UART:
            self._rx.extend(data[2:])
        else:
            self._rx.extend(data)

    def write(self, data: bytes) -> int:
        ws = self._ensure_open()
        if not isinstance(data, (bytes, bytearray)):
            data = bytes(data)
        try:
            # UART TX path expects raw binary payload (no channel header).
            ws.send_binary(bytes(data))
        except websocket.WebSocketTimeoutException as exc:
            raise serial.SerialTimeoutException(str(exc)) from exc
        except Exception as exc:
            raise serial.SerialException(f"Write failed: {exc}") from exc
        return len(data)

    def read(self, size: int = 1) -> bytes:
        if size <= 0:
            return b""
        ws_timeout = self.timeout
        deadline = None if ws_timeout is None else (time.monotonic() + float(ws_timeout))
        while len(self._rx) < size:
            if deadline is not None:
                remain = deadline - time.monotonic()
                if remain <= 0:
                    break
                self._recv_once(min(0.2, remain))
            else:
                self._recv_once(0.2)
        out = bytes(self._rx[:size])
        del self._rx[: len(out)]
        return out

    def read_until(self, expected: bytes = b"\n", size: Optional[int] = None) -> bytes:
        if not expected:
            expected = b"\n"
        exp = bytes(expected)
        out = bytearray()
        ws_timeout = self.timeout
        deadline = None if ws_timeout is None else (time.monotonic() + float(ws_timeout))
        while True:
            if size is not None and len(out) >= size:
                break
            if out.endswith(exp):
                break
            if self._rx:
                out.append(self._rx[0])
                del self._rx[0]
                continue
            if deadline is not None:
                remain = deadline - time.monotonic()
                if remain <= 0:
                    break
                self._recv_once(min(0.2, remain))
            else:
                self._recv_once(0.2)
        return bytes(out)

    def readline(self, size: Optional[int] = None) -> bytes:
        return self.read_until(expected=b"\n", size=size)

    def flush(self) -> None:
        # WebSocket send is synchronous in websocket-client; nothing extra needed.
        return None

    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def reset_output_buffer(self) -> None:
        # No host-side buffered TX queue.
        return None


@dataclass
class MapleLinkDumpResult:
    ok: bool
    state: str
    detail: str
    data: bytes
    expected_bytes: int
    received_bytes: int
    missing_bytes: int
    statuses: List[Dict[str, Any]] = field(default_factory=list)


class MapleLinkControlClient:
    def __init__(self, host: str = "192.168.7.1", ws_port: int = 81, timeout: float = 5.0):
        self.host = host
        self.ws_port = ws_port
        self.timeout = timeout
        self.ws: Optional[websocket.WebSocket] = None
        self.spi = MapleLinkSPIClient(self)
        self.emmc = MapleLinkEMMCClient(self)

    def connect(self) -> None:
        if self.ws is not None:
            return
        url = f"ws://{self.host}:{self.ws_port}/ws"
        self.ws = websocket.create_connection(url, timeout=self.timeout)
        self.send("hello")

    def close(self) -> None:
        if self.ws is not None:
            self.ws.close()
            self.ws = None

    def __enter__(self) -> "MapleLinkControlClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def send(self, msg_type: str, payload: Optional[Dict[str, Any]] = None, port: Optional[str] = None) -> None:
        if self.ws is None:
            raise RuntimeError("WebSocket not connected. Call connect() first.")
        msg: Dict[str, Any] = {"type": msg_type}
        if port is not None:
            msg["port"] = port
        if payload:
            msg.update(payload)
        self.ws.send(json.dumps(msg))

    def recv(self, timeout: Optional[float] = None) -> Tuple[str, Any]:
        if self.ws is None:
            raise RuntimeError("WebSocket not connected. Call connect() first.")
        prev_to = self.ws.gettimeout()
        if timeout is not None:
            self.ws.settimeout(timeout)
        try:
            raw = self.ws.recv()
        finally:
            if timeout is not None:
                self.ws.settimeout(prev_to)
        if isinstance(raw, str):
            return ("json", json.loads(raw))
        return ("binary", bytes(raw))

    def wait_for_type(self, expected_type: str, timeout: float = 5.0) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                raise TimeoutError(f"Timed out waiting for {expected_type}")
            kind, value = self.recv(timeout=remain)
            if kind != "json":
                continue
            if value.get("type") == expected_type:
                return value


class MapleLinkSPIClient:
    def __init__(self, ctl: MapleLinkControlClient):
        self.ctl = ctl

    def get_config(self, timeout: float = 3.0) -> Dict[str, Any]:
        self.ctl.send("spi.get_config", port="spi0")
        return self.ctl.wait_for_type("spi.config", timeout=timeout)

    def read_id(self, interval_ms: int = 250, timeout: float = 5.0) -> Dict[str, Any]:
        self.ctl.send(
            "spi.idpoll.start",
            {"interval_ms": int(interval_ms), "monitor_between": False},
            port="spi0",
        )
        try:
            return self.ctl.wait_for_type("spi.id.result", timeout=timeout)
        finally:
            self.ctl.send("spi.idpoll.stop", port="spi0")

    def raw_txrx(
        self,
        tx_hex: str,
        ff_tail: int = 0,
        speed_hz: Optional[int] = None,
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        payload: Dict[str, Any] = {"tx_hex": tx_hex, "ff_tail": int(ff_tail)}
        if speed_hz is not None:
            payload["speed_hz"] = int(speed_hz)
        self.ctl.send("spi.raw.txrx", payload, port="spi0")
        return self.ctl.wait_for_type("spi.raw.result", timeout=timeout)

    def dump(
        self,
        start_addr: int,
        length_bytes: int,
        addr_bytes: int = 3,
        read_cmd: Optional[int] = None,
        chunk_bytes: int = 256,
        double_read: bool = False,
        verify_retries: int = 0,
        ff_opt: bool = True,
        speed_hz: Optional[int] = None,
        timeout: float = 120.0,
    ) -> MapleLinkDumpResult:
        if read_cmd is None:
            read_cmd = 0x13 if addr_bytes >= 4 else 0x03
        payload: Dict[str, Any] = {
            "start_addr": int(start_addr) & 0xFFFFFFFF,
            "length_bytes": int(length_bytes),
            "addr_bytes": int(addr_bytes),
            "read_cmd": int(read_cmd) & 0xFF,
            "chunk_bytes": int(chunk_bytes),
            "double_read": bool(double_read),
            "verify_retries": int(verify_retries),
            "ff_opt": bool(ff_opt),
        }
        if speed_hz is not None:
            payload["speed_hz"] = int(speed_hz)

        self.ctl.send("spi.dump.start", payload, port="spi0")
        return self._collect_dump(expected_bytes=int(length_bytes), timeout=timeout)

    def _collect_dump(self, expected_bytes: int, timeout: float) -> MapleLinkDumpResult:
        deadline = time.monotonic() + timeout
        buf = bytearray(expected_bytes)
        written_max = 0
        statuses: List[Dict[str, Any]] = []
        final_state = "error"
        final_detail = "timeout"

        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                break
            kind, value = self.ctl.recv(timeout=remain)
            if kind == "binary":
                ch, off, cnt, payload, fill = _parse_ws_binary(value)
                if ch == WS_CH_SPI_DUMP_DATA:
                    end = min(expected_bytes, off + len(payload))
                    if off < expected_bytes and end > off:
                        buf[off:end] = payload[: end - off]
                        written_max = max(written_max, end)
                elif ch == WS_CH_SPI_DUMP_FF:
                    end = min(expected_bytes, off + cnt)
                    if off < expected_bytes and end > off:
                        buf[off:end] = b"\xFF" * (end - off)
                        written_max = max(written_max, end)
                continue

            msg = value
            mtype = msg.get("type")
            if mtype == "spi.dump.status":
                statuses.append(msg)
                state = str(msg.get("state", ""))
                if state in ("complete", "error", "stopped"):
                    final_state = state
                    final_detail = str(msg.get("detail", ""))
                    break

        out = bytes(buf[:written_max] if written_max < expected_bytes else buf)
        missing = max(0, expected_bytes - len(out))
        ok = final_state == "complete"
        return MapleLinkDumpResult(
            ok=ok,
            state=final_state,
            detail=final_detail,
            data=out,
            expected_bytes=expected_bytes,
            received_bytes=len(out),
            missing_bytes=missing,
            statuses=statuses,
        )


class MapleLinkEMMCClient:
    def __init__(self, ctl: MapleLinkControlClient):
        self.ctl = ctl

    def get_config(self, timeout: float = 3.0) -> Dict[str, Any]:
        self.ctl.send("emmc.get_config", port="emmc0")
        return self.ctl.wait_for_type("emmc.config", timeout=timeout)

    def cmd1_check(self, retries: int = 24, arg: int = 0x40FF8000, timeout: float = 6.0) -> Dict[str, Any]:
        self.ctl.send("emmc.cmd1_test", {"retries": int(retries), "arg": int(arg) & 0xFFFFFFFF}, port="emmc0")
        return self.ctl.wait_for_type("emmc.cmd1_test.result", timeout=timeout)

    def id_read(
        self,
        interval_ms: int = 500,
        speed_hz: Optional[int] = None,
        retry_idle_clks: int = 8,
        timeout: float = 8.0,
    ) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "interval_ms": int(interval_ms),
            "retry_idle_clks": int(retry_idle_clks),
            "monitor_between": False,
        }
        if speed_hz is not None:
            payload["speed_hz"] = int(speed_hz)
        self.ctl.send("emmc.idpoll.start", payload, port="emmc0")
        try:
            return self.ctl.wait_for_type("emmc.id.result", timeout=timeout)
        finally:
            self.ctl.send("emmc.idpoll.stop", port="emmc0")

    def layout_read(self, timeout: float = 10.0) -> Dict[str, Any]:
        self.ctl.send("emmc.layout.read", port="emmc0")
        return self.ctl.wait_for_type("emmc.layout.result", timeout=timeout)

    def dump(
        self,
        start_lba: int,
        block_count: int,
        chunk_bytes: int = 256,
        use_pio: bool = True,
        double_read: bool = False,
        verify_retries: int = 1,
        auto_retries: int = 4,
        retry_idle_clks: int = 8,
        timeout: float = 180.0,
    ) -> MapleLinkDumpResult:
        payload = {
            "start_lba": int(start_lba) & 0xFFFFFFFF,
            "block_count": int(block_count),
            "chunk_bytes": int(chunk_bytes),
            "use_pio": bool(use_pio),
            "double_read": bool(double_read),
            "verify_retries": int(verify_retries),
            "auto_retries": int(auto_retries),
            "retry_idle_clks": int(retry_idle_clks),
        }
        expected_bytes = int(block_count) * 512
        self.ctl.send("emmc.dump.start", payload, port="emmc0")
        return self._collect_dump(expected_bytes=expected_bytes, timeout=timeout)

    def _collect_dump(self, expected_bytes: int, timeout: float) -> MapleLinkDumpResult:
        deadline = time.monotonic() + timeout
        buf = bytearray(expected_bytes)
        written_max = 0
        statuses: List[Dict[str, Any]] = []
        final_state = "error"
        final_detail = "timeout"

        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                break
            kind, value = self.ctl.recv(timeout=remain)
            if kind == "binary":
                ch, off, cnt, payload, fill = _parse_ws_binary(value)
                if ch == WS_CH_EMMC_DUMP_DATA:
                    end = min(expected_bytes, off + len(payload))
                    if off < expected_bytes and end > off:
                        buf[off:end] = payload[: end - off]
                        written_max = max(written_max, end)
                elif ch == WS_CH_EMMC_DUMP_RUN:
                    end = min(expected_bytes, off + cnt)
                    if off < expected_bytes and end > off:
                        buf[off:end] = bytes([fill]) * (end - off)
                        written_max = max(written_max, end)
                continue

            msg = value
            mtype = msg.get("type")
            if mtype == "emmc.dump.status":
                statuses.append(msg)
                state = str(msg.get("state", ""))
                if state in ("complete", "error", "stopped"):
                    final_state = state
                    final_detail = str(msg.get("detail", ""))
                    break

        out = bytes(buf[:written_max] if written_max < expected_bytes else buf)
        missing = max(0, expected_bytes - len(out))
        ok = final_state == "complete"
        return MapleLinkDumpResult(
            ok=ok,
            state=final_state,
            detail=final_detail,
            data=out,
            expected_bytes=expected_bytes,
            received_bytes=len(out),
            missing_bytes=missing,
            statuses=statuses,
        )


def _le32(data: bytes, off: int) -> int:
    return int(data[off]) | (int(data[off + 1]) << 8) | (int(data[off + 2]) << 16) | (int(data[off + 3]) << 24)


def _le16(data: bytes, off: int) -> int:
    return int(data[off]) | (int(data[off + 1]) << 8)


def _parse_ws_binary(frame: bytes) -> Tuple[int, int, int, bytes, int]:
    if len(frame) < 2 or frame[0] != WS_BIN_MAGIC:
        return (-1, 0, 0, b"", 0)
    ch = int(frame[1])
    if ch in (WS_CH_SPI_DUMP_DATA, WS_CH_SPI_DUMP_FF, WS_CH_EMMC_DUMP_DATA) and len(frame) >= 8:
        off = _le32(frame, 2)
        cnt = _le16(frame, 6)
        payload = frame[8 : 8 + cnt]
        return (ch, off, cnt, payload, 0)
    if ch == WS_CH_EMMC_DUMP_RUN and len(frame) >= 9:
        off = _le32(frame, 2)
        cnt = _le16(frame, 6)
        fill = int(frame[8])
        return (ch, off, cnt, b"", fill)
    return (ch, 0, 0, b"", 0)

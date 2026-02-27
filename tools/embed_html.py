#!/usr/bin/env python3
import argparse
from pathlib import Path


def c_escape(text: str) -> str:
    out = []
    for ch in text:
        code = ord(ch)
        if ch == "\\":
            out.append("\\\\")
        elif ch == "\"":
            out.append("\\\"")
        elif ch == "\n":
            out.append("\\n")
        elif ch == "\r":
            out.append("\\r")
        elif ch == "\t":
            out.append("\\t")
        elif 32 <= code <= 126:
            out.append(ch)
        else:
            out.append(f"\\x{code:02x}")
    return "".join(out)


def main() -> int:
    p = argparse.ArgumentParser(description="Embed HTML as a C header")
    p.add_argument("--input", required=True, help="Input HTML file")
    p.add_argument("--output", required=True, help="Output header file")
    p.add_argument(
        "--define",
        action="append",
        default=[],
        help="Token replacement in KEY=VALUE form. Replaces __KEY__ in input.",
    )
    args = p.parse_args()

    src = Path(args.input).read_text(encoding="utf-8")

    for d in args.define:
        if "=" not in d:
            raise SystemExit(f"Invalid --define value: {d}")
        key, value = d.split("=", 1)
        src = src.replace(f"__{key}__", value)

    escaped = c_escape(src)
    header = (
        "#ifndef INDEX_HTML_DATA_H\n"
        "#define INDEX_HTML_DATA_H\n\n"
        "static const char BROWSERIO_INDEX_HTML[] =\n"
        f"\"{escaped}\";\n\n"
        "static const int BROWSERIO_INDEX_HTML_LEN = (int)(sizeof(BROWSERIO_INDEX_HTML) - 1);\n\n"
        "#endif\n"
    )

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(header, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

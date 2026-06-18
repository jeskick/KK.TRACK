#!/usr/bin/env python3
import gzip
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[1]
html = (ROOT / "embed" / "rx_web.html").read_text(encoding="utf-8")
html = re.sub(r">\s+<", "><", html.strip())
raw = html.encode("utf-8")
data = gzip.compress(raw, compresslevel=9)

lines = [
    "#include <stddef.h>",
    "#include <stdint.h>",
    "",
    "const uint8_t kk_rx_web_page[] = {",
]
for i in range(0, len(data), 16):
    chunk = data[i : i + 16]
    lines.append("    " + ", ".join(f"0x{b:02x}" for b in chunk) + ",")
lines += [
    "};",
    "",
    f"const size_t kk_rx_web_page_len = {len(data)};",
    "",
]

(ROOT / "src" / "rx_web_page.c").write_text("\n".join(lines), encoding="utf-8")
(ROOT / "include" / "kk" / "rx_web_page.h").write_text(
    "#pragma once\n#include <stddef.h>\n#include <stdint.h>\n"
    "extern const uint8_t kk_rx_web_page[];\nextern const size_t kk_rx_web_page_len;\n",
    encoding="utf-8",
)
print("html", len(raw), "gzip", len(data))

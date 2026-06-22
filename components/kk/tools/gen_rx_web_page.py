#!/usr/bin/env python3
"""Generate gzip rx_web_page.c for all kk component trees from embed/rx_web.html."""
import gzip
import pathlib
import re

CANON = pathlib.Path(__file__).resolve().parents[1]
REPO = CANON.parents[1]
KK_DIRS = (
    CANON,
    REPO / "receiver" / "pio" / "rx" / "components" / "kk",
    REPO / "transmitter" / "pio" / "tx" / "components" / "kk",
)


def _min_css(css: str) -> str:
    css = re.sub(r"/\*.*?\*/", "", css, flags=re.S)
    css = re.sub(r"\s+", " ", css)
    css = re.sub(r"\s*([{}:;,>+~])\s*", r"\1", css)
    return css.strip()


def _min_js(js: str) -> str:
    js = re.sub(r"//[^\n\"']*$", "", js, flags=re.M)
    js = "".join(line.strip() for line in js.splitlines() if line.strip())
    js = re.sub(r"\s+([{};,=()[\]])", r"\1", js)
    js = re.sub(r"([{};,=([\]])\s+", r"\1", js)
    return js.strip()


def _pack_html(html: str) -> str:
    html = re.sub(r">\s+<", "><", html.strip())
    html = re.sub(
        r"<style>(.*?)</style>",
        lambda m: "<style>" + _min_css(m.group(1)) + "</style>",
        html,
        flags=re.S,
    )
    html = re.sub(
        r"<script>(.*?)</script>",
        lambda m: "<script>" + _min_js(m.group(1)) + "</script>",
        html,
        flags=re.S,
    )
    return html


html = _pack_html((CANON / "embed" / "rx_web.html").read_text(encoding="utf-8"))
raw = html.encode("utf-8")
data = gzip.compress(raw, compresslevel=9)

page_lines = [
    "#include <stddef.h>",
    "#include <stdint.h>",
    "",
    "const uint8_t kk_rx_web_page[] = {",
]
for i in range(0, len(data), 16):
    chunk = data[i : i + 16]
    page_lines.append("    " + ", ".join(f"0x{b:02x}" for b in chunk) + ",")
page_lines += [
    "};",
    "",
    f"const size_t kk_rx_web_page_len = {len(data)};",
    "",
]
page_src = "\n".join(page_lines)
page_hdr = (
    "#pragma once\n#include <stddef.h>\n#include <stdint.h>\n"
    "extern const uint8_t kk_rx_web_page[];\nextern const size_t kk_rx_web_page_len;\n"
)

for kk in KK_DIRS:
    if not kk.is_dir():
        continue
    (kk / "src" / "rx_web_page.c").write_text(page_src, encoding="utf-8")
    (kk / "include" / "kk" / "rx_web_page.h").write_text(page_hdr, encoding="utf-8")

print("html", len(raw), "gzip", len(data), "trees", len(KK_DIRS))

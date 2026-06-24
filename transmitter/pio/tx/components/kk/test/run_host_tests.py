#!/usr/bin/env python3
"""Host-side unit tests for kk pure-C helpers (ota_image, rc_proto, rx_profile sanitize)."""
from __future__ import annotations

import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
HOST = ROOT / "test" / "host"
KK = ROOT
OUT = HOST / "test_host"


def run(cmd: list[str]) -> None:
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=HOST)


def main() -> int:
    stubs = HOST / "stubs"
    sources = [
        str(KK / "src" / "ota_image.c"),
        str(KK / "src" / "rc_proto.c"),
        str(KK / "src" / "rx_profile.c"),
        str(HOST / "test_host.c"),
        str(stubs / "nvs_stub.c"),
    ]
    includes = [
        f"-I{KK / 'include'}",
        f"-I{stubs}",
        "-I.",
    ]
    run(["gcc", "-std=c11", "-Wall", "-Wextra", "-O0", *includes, *sources, "-o", str(OUT), "-lm"])
    run([str(OUT)])
    print("host tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

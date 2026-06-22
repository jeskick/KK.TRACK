#!/usr/bin/env python3
"""Copy canonical components/kk into RX/TX PlatformIO trees."""
import pathlib
import shutil
import sys

CANON = pathlib.Path(__file__).resolve().parents[1]
REPO = CANON.parents[1]
DESTS = (
    REPO / "receiver" / "pio" / "rx" / "components" / "kk",
    REPO / "transmitter" / "pio" / "tx" / "components" / "kk",
)
SKIP = {"__pycache__", ".git"}


def copy_tree(src: pathlib.Path, dst: pathlib.Path) -> None:
    for path in src.rglob("*"):
        rel = path.relative_to(src)
        if any(part in SKIP for part in rel.parts):
            continue
        out = dst / rel
        if path.is_dir():
            out.mkdir(parents=True, exist_ok=True)
        else:
            out.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, out)


def main() -> int:
    for dest in DESTS:
        if not dest.parent.is_dir():
            print("skip missing", dest)
            continue
        print("sync", CANON, "->", dest)
        copy_tree(CANON, dest)
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Fail if RX/TX kk trees diverge from canonical components/kk."""
import hashlib
import pathlib
import sys

CANON = pathlib.Path(__file__).resolve().parents[1]
REPO = CANON.parents[1]
DESTS = (
    REPO / "receiver" / "pio" / "rx" / "components" / "kk",
    REPO / "transmitter" / "pio" / "tx" / "components" / "kk",
)
SKIP = {"__pycache__", ".git"}


def file_hash(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def collect(root: pathlib.Path) -> dict[str, str]:
    out: dict[str, str] = {}
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        rel = path.relative_to(root)
        if any(part in SKIP for part in rel.parts):
            continue
        out[str(rel).replace("\\", "/")] = file_hash(path)
    return out


def main() -> int:
    canon = collect(CANON)
    err = 0
    for dest in DESTS:
        if not dest.is_dir():
            print("WARN missing", dest)
            continue
        other = collect(dest)
        only_canon = set(canon) - set(other)
        only_dest = set(other) - set(canon)
        diff = [k for k in canon if k in other and canon[k] != other[k]]
        if only_canon or only_dest or diff:
            err = 1
            print("MISMATCH", dest.name)
            for k in sorted(only_canon):
                print("  only canonical:", k)
            for k in sorted(only_dest):
                print("  only copy:", k)
            for k in sorted(diff):
                print("  differs:", k)
    if err:
        print("Run: python components/kk/tools/sync_kk_copies.py")
    return err


if __name__ == "__main__":
    sys.exit(main())

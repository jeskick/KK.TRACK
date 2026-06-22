"""PlatformIO pre/post: bump project VERSION in CMakeLists.txt, copy firmware.bin to ota/."""
Import("env")

import re
import shutil
from pathlib import Path

PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))
BUILD_DIR = Path(env.subst("$BUILD_DIR"))
OTA_DIR = (PROJECT_DIR / "../../../ota").resolve()
CMAKE = PROJECT_DIR / "CMakeLists.txt"

_proj = str(PROJECT_DIR).replace("\\", "/").lower()
ROLE_TAG = "TX" if "/pio/tx" in _proj else "RX"


def _bump_cmake_version():
    if not CMAKE.is_file():
        print("WARN kk_fw: missing %s" % CMAKE)
        return None
    text = CMAKE.read_text(encoding="utf-8")
    m = re.search(r'project\(\w+\s+VERSION\s+"(\d+)\.(\d+)\.(\d+)"\)', text)
    if not m:
        print("WARN kk_fw: no project(VERSION) in %s" % CMAKE)
        return None
    major, minor, patch = (int(m.group(i)) for i in range(1, 4))
    patch += 1
    new_ver = "%d.%d.%d" % (major, minor, patch)
    new_text = re.sub(
        r'(project\(\w+\s+VERSION\s+)"[\d\.]+"',
        r'\1"%s"' % new_ver,
        text,
        count=1,
    )
    if new_text == text:
        return None
    CMAKE.write_text(new_text, encoding="utf-8")
    print("kk_fw: %s VERSION -> %s" % (ROLE_TAG, new_ver))
    return new_ver


def _read_cmake_version():
    if not CMAKE.is_file():
        return "?"
    m = re.search(r'project\(\w+\s+VERSION\s+"([\d\.]+)"\)', CMAKE.read_text(encoding="utf-8"))
    return m.group(1) if m else "?"


def _copy_ota(source, target, env):
    fw = BUILD_DIR / "firmware.bin"
    if not fw.is_file():
        print("WARN kk_fw: no firmware.bin to copy")
        return
    ver = _read_cmake_version()
    ota_name = "%s.%s.bin" % (ROLE_TAG, ver)
    OTA_DIR.mkdir(parents=True, exist_ok=True)
    dest = OTA_DIR / ota_name
    shutil.copy2(fw, dest)
    print(
        "kk_fw: ota/%s <= firmware.bin (%u bytes, v%s)"
        % (ota_name, fw.stat().st_size, ver)
    )


_bump_cmake_version()
env.AddPostAction("buildprog", _copy_ota)

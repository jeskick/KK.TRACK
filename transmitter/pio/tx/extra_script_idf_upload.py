"""Full IDF flash: bootloader + OTA partition table + otadata + app."""
Import("env")
import os
import subprocess
import sys
from os.path import join, isfile, normpath


def _ninja(build_dir, *targets):
    ninja = join(env.PioPlatform().get_package_dir("tool-ninja") or "", "ninja")
    if not isfile(ninja):
        return False
    r = subprocess.run([ninja, *targets], cwd=build_dir)
    return r.returncode == 0


def _resolve_bin(build, relpath):
    path = normpath(join(build, relpath.replace("/", os.sep)))
    if isfile(path):
        return path
    leaf = os.path.basename(relpath)
    if leaf.endswith(".bin") and leaf != "firmware.bin":
        alt = join(build, "firmware.bin")
        if isfile(alt):
            return alt
    if leaf == "bootloader.bin":
        alt = join(build, "bootloader.bin")
        if isfile(alt):
            return alt
    return path


def _setup_full_upload(env):
    build = normpath(env.subst("$BUILD_DIR"))
    esptool = join(env.PioPlatform().get_package_dir("tool-esptoolpy") or "", "esptool.py")
    mcu = env.subst("${BOARD_MCU}")
    extra = env.GetProjectOption("upload_flags", [])
    if isinstance(extra, str):
        extra = extra.split()
    extra_s = " ".join(extra)

    _ninja(build, "partition_table/partition-table.bin", "ota_data_initial.bin")

    args_file = join(build, "flash_project_args")
    if not isfile(args_file):
        sys.stderr.write("WARN: missing %s — run pio run first\n" % args_file)
        return

    parts = []
    with open(args_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("--"):
                parts.extend(line.split())
                continue
            tok = line.split(None, 1)
            if len(tok) != 2:
                continue
            offset, relpath = tok
            path = _resolve_bin(build, relpath)
            if not isfile(path):
                sys.stderr.write("WARN: missing flash image %s\n" % path)
                return
            parts.append(offset)
            parts.append('"%s"' % path if " " in path else path)

    flash_s = " ".join(parts)
    env.Replace(
        UPLOADER=esptool,
        UPLOADERFLAGS=[],
        UPLOADCMD=(
            '"$PYTHONEXE" "$UPLOADER" --chip '
            + mcu
            + ' --port "$UPLOAD_PORT" --baud "$UPLOAD_SPEED" '
            + extra_s
            + " --before default_reset --after hard_reset write_flash "
            + flash_s
        ),
    )


_setup_full_upload(env)

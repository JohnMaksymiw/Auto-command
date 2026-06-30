#!/usr/bin/env python3
"""
Flash all three Arduino boards in one shot.

Requirements (one-time setup):
  brew install arduino-cli
  arduino-cli core install arduino:avr

Board serial numbers:
  Mixer_code.cpp  → serial 13301
  Bowl_code.cpp   → serial 11101
  linear.cpp      → serial 13401
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

BOARDS = [
    {
        "label":  "Mixer",
        "src":    "Mixer_13301.ino",
        "sketch": "Mixer_13301",
        "serial": "13301",
        "fqbn":   "arduino:avr:uno",
    },
    {
        "label":  "Bowl",
        "src":    "3_Bowl.ino",
        "sketch": "3_Bowl",
        "serial": "11101",
        "fqbn":   "arduino:avr:uno",
    },
    {
        "label":  "Linear",
        "src":    "2_Dispenser.ino",
        "sketch": "2_Dispenser",
        "serial": "13401",
        "fqbn":   "arduino:avr:uno",
    },
]


def check_arduino_cli():
    if shutil.which("arduino-cli") is None:
        print("ERROR: arduino-cli not found.")
        print("  Install: brew install arduino-cli")
        print("  Then:    arduino-cli core install arduino:avr")
        sys.exit(1)


def find_ports():
    result = subprocess.run(
        ["arduino-cli", "board", "list", "--format", "json"],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        print("ERROR running 'arduino-cli board list':", result.stderr)
        sys.exit(1)

    try:
        data = json.loads(result.stdout)
    except json.JSONDecodeError:
        print("ERROR: could not parse arduino-cli output.")
        print(result.stdout)
        sys.exit(1)

    ports = {}
    for entry in data:
        port_info = entry.get("port", {})
        address = port_info.get("address", "")
        props = port_info.get("properties", {})
        hw_id = port_info.get("hardware_id", "")
        serial = props.get("serialNumber", hw_id)
        if address and serial:
            ports[serial] = address

    return ports


def match_port(ports, serial_suffix):
    for sn, addr in ports.items():
        if sn.endswith(serial_suffix) or sn == serial_suffix:
            return addr
    return None


def flash_board(board, port, tmpdir):
    sketch_dir = os.path.join(tmpdir, board["sketch"])
    os.makedirs(sketch_dir)
    src = os.path.join(SCRIPT_DIR, board["src"])
    dst = os.path.join(sketch_dir, board["sketch"] + ".ino")
    shutil.copy2(src, dst)

    print(f"\n{'='*50}")
    print(f"  {board['label']} — compiling {board['src']}")
    print(f"{'='*50}")
    r = subprocess.run(
        ["arduino-cli", "compile", "--fqbn", board["fqbn"], sketch_dir]
    )
    if r.returncode != 0:
        print(f"  COMPILE FAILED — {board['label']}")
        return False

    print(f"\n  {board['label']} — uploading to {port}")
    r = subprocess.run(
        ["arduino-cli", "upload", "--fqbn", board["fqbn"], "-p", port, sketch_dir]
    )
    if r.returncode != 0:
        print(f"  UPLOAD FAILED — {board['label']}")
        return False

    print(f"  {board['label']} — OK ✓")
    return True


def main():
    check_arduino_cli()

    print("Scanning for connected boards...")
    ports = find_ports()

    if not ports:
        print("No boards detected. Check USB connections and try again.")
        sys.exit(1)

    print(f"Found {len(ports)} port(s):")
    for sn, addr in ports.items():
        print(f"  {addr}  (serial: {sn})")

    tmpdir = tempfile.mkdtemp(prefix="arduino_flash_")
    failures = []

    try:
        for board in BOARDS:
            port = match_port(ports, board["serial"])
            if not port:
                print(f"\nERROR: {board['label']} (serial *{board['serial']}) not found.")
                print(f"  Detected serials: {list(ports.keys())}")
                failures.append(board["label"])
                continue
            ok = flash_board(board, port, tmpdir)
            if not ok:
                failures.append(board["label"])
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)

    print(f"\n{'='*50}")
    if not failures:
        print("  All boards flashed successfully!")
    else:
        print(f"  Failed: {', '.join(failures)}")
        sys.exit(1)


if __name__ == "__main__":
    main()

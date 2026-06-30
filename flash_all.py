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
import urllib.request

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

GITHUB_RAW = "https://raw.githubusercontent.com/JohnMaksymiw/Auto-command/main"

BOARDS = [
    {
        "label":   "Mixer",
        "src":     "Mixer_code.cpp",
        "sketch":  "Mixer_13301",
        "serial":  "13301",
        "fqbn":    "arduino:avr:uno",
        "github":  f"{GITHUB_RAW}/Mixer_code.cpp",
    },
    {
        "label":   "Bowl",
        "src":     "Bowl_code.cpp",
        "sketch":  "3_Bowl",
        "serial":  "11101",
        "fqbn":    "arduino:avr:uno",
        "github":  f"{GITHUB_RAW}/Bowl_code.cpp",
    },
    {
        "label":   "Linear",
        "src":     "linear.cpp",
        "sketch":  "2_Dispenser",
        "serial":  "13401",
        "fqbn":    "arduino:avr:uno",
        "github":  f"{GITHUB_RAW}/linear.cpp",
    },
]


REQUIRED_LIBS = ["Servo", "HX711_ADC"]

def check_arduino_cli():
    if shutil.which("arduino-cli") is None:
        print("ERROR: arduino-cli not found.")
        print("  Install: brew install arduino-cli")
        print("  Then:    arduino-cli core install arduino:avr")
        sys.exit(1)

def install_libraries():
    for lib in REQUIRED_LIBS:
        print(f"Checking library: {lib}")
        r = subprocess.run(
            ["arduino-cli", "lib", "install", lib],
            capture_output=True, text=True
        )
        if r.returncode != 0 and "already installed" not in r.stderr.lower():
            print(f"  WARNING: could not install {lib}: {r.stderr.strip()}")


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

    # Support both list and {"detected_ports": [...]} formats
    entries = data.get("detected_ports", []) if isinstance(data, dict) else data

    ports = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        port_info = entry.get("port", {})
        address = port_info.get("address", "")
        if address:
            ports.append(address)

    return ports


def match_port(ports, serial_suffix):
    # Match by port address ending with the serial number
    # e.g. serial "13301" matches "/dev/cu.usbmodem13301"
    for addr in ports:
        if addr.endswith(serial_suffix):
            return addr
    return None


def fetch_source(board, tmpdir):
    sketch_dir = os.path.join(tmpdir, board["sketch"])
    os.makedirs(sketch_dir)
    dst = os.path.join(sketch_dir, board["sketch"] + ".ino")
    print(f"  Downloading latest {board['src']} from GitHub...")
    try:
        urllib.request.urlretrieve(board["github"] + f"?v={os.getpid()}", dst)
        print(f"  Downloaded OK")
    except Exception as e:
        print(f"  WARNING: GitHub download failed ({e}), falling back to local file.")
        local = os.path.join(SCRIPT_DIR, board["src"])
        if os.path.exists(local):
            shutil.copy2(local, dst)
        else:
            print(f"  ERROR: No local fallback found for {board['src']}")
            return None
    return sketch_dir

def flash_board(board, port, tmpdir):
    sketch_dir = fetch_source(board, tmpdir)
    if not sketch_dir:
        return False

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
    install_libraries()

    print("Scanning for connected boards...")
    ports = find_ports()

    if not ports:
        print("No boards detected. Check USB connections and try again.")
        sys.exit(1)

    print(f"Found {len(ports)} port(s):")
    for addr in ports:
        print(f"  {addr}")

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

#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
python3 "$SCRIPT_DIR/flash_all.py"
echo ""
read -p "Press Enter to close..."

#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
gcc -O2 -Wall -shared -fPIC -o "$SCRIPT_DIR/libmcp4728_driver.so" "$SCRIPT_DIR/mcp4728_driver.c"
echo "Built: $SCRIPT_DIR/libmcp4728_driver.so"

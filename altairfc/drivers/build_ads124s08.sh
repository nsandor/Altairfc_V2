#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Requires libgpiod headers (one-time setup, not needed by any other
# driver in this repo): sudo apt install -y libgpiod-dev
gcc -O2 -Wall -shared -fPIC -o "$SCRIPT_DIR/libads124s08_driver.so" \
    "$SCRIPT_DIR/ads124s08_driver.c" -lgpiod -lm
echo "Built: $SCRIPT_DIR/libads124s08_driver.so"

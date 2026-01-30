#!/bin/bash
# Test script to capture timing data from picostream output
# Uses Sipeed SLogic16 to record D0 and apply timing decoder

set -e

cd "$(dirname "$0")"

# Build client if needed
cargo build --release -p picostream

CLIENT_PID=""

cleanup() {
    if [ -n "$CLIENT_PID" ]; then
        kill "$CLIENT_PID" 2>/dev/null || true
        wait "$CLIENT_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# Start client in background sending continuous 0xAA pattern at 1MHz
echo "Starting client (0xAA pattern at 1MHz)..."
while true; do
    dd if=/dev/zero bs=4096 count=256 2>/dev/null | tr '\0' '\252'
done | ./target/release/picostream -s 1000000 &
CLIENT_PID=$!

# Give client time to start and fill buffer
sleep 1

# Record 2s of data from D0 at 5MHz with timing decoder
echo "Recording 2s from logic analyzer (D0 @ 5MHz)..."
sigrok-cli \
    -d sipeed-slogic-analyzer \
    --config samplerate=5M \
    --channels D0 \
    --samples 10000000 \
    -P timing:data=D0 \
    -l 3 2>/dev/null

echo "Done."

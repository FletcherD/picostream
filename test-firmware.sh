#!/bin/bash
set -e

# Test script for pico-bitstream firmware
# Flashes firmware, sends test data, captures logs from both sides

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
FIRMWARE_LOG="/tmp/pico-bitstream-firmware-${TIMESTAMP}.log"
CLIENT_LOG="/tmp/pico-bitstream-client-${TIMESTAMP}.log"

VID="1209"
PID="0001"

cleanup() {
    echo "Cleaning up..."
    if [ -n "$FIRMWARE_PID" ] && kill -0 "$FIRMWARE_PID" 2>/dev/null; then
        kill "$FIRMWARE_PID" 2>/dev/null || true
        wait "$FIRMWARE_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

DEFMT_LOG=debug cargo build -p pico-bitstream-firmware \
    --target thumbv8m.main-none-eabihf \
    --no-default-features --features rp235x \
    --release 2>&1 > /dev/null

echo "=== Flashing and running firmware ==="
echo "Firmware log: $FIRMWARE_LOG"

DEFMT_LOG=debug cargo run -p pico-bitstream-firmware \
    --target thumbv8m.main-none-eabihf \
    --no-default-features --features rp235x \
    --release \
    > "$FIRMWARE_LOG" 2>&1 &
FIRMWARE_PID=$!

sleep 5

echo "Firmware PID: $FIRMWARE_PID"
echo "Waiting for USB device to appear..."

# Wait for USB device to enumerate (timeout after 30s)
TIMEOUT=30
ELAPSED=0
while ! lsusb -d "${VID}:${PID}" >/dev/null 2>&1; do
    sleep 1
    ELAPSED=$((ELAPSED + 1))
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo "ERROR: USB device did not appear within ${TIMEOUT}s"
        echo "Firmware log contents:"
        cat "$FIRMWARE_LOG"
        exit 1
    fi
done

echo "USB device found after ${ELAPSED}s"

# Small delay to let device fully initialize
sleep 3

echo ""
echo "=== Sending test data ==="
echo "Client log: $CLIENT_LOG"

# Send test pattern: 0xAA repeated (creates 500kHz square wave at 1MHz sample rate)

dd if=/dev/zero bs=1024 count=1024 2>/dev/null | tr '\0' $'\x55' | ./target/release/pico-bitstream -s 1000000 \
    > "$CLIENT_LOG" 2>&1
CLIENT_EXIT=$?

echo ""
echo "=== Test complete ==="
echo "Client exit code: $CLIENT_EXIT"

# Give firmware a moment to log any final messages
sleep 1

echo ""
echo "=== Log files ==="
echo "Firmware: $FIRMWARE_LOG"
echo "Client:   $CLIENT_LOG"

echo ""
echo "=== Firmware log (last 50 lines) ==="
tail -50 "$FIRMWARE_LOG" || echo "(empty or not readable)"

echo ""
echo "=== Client log ==="
cat "$CLIENT_LOG" || echo "(empty or not readable)"

exit $CLIENT_EXIT

#!/bin/bash
# Test script for picostream
# Run this after starting firmware with: DEFMT_LOG=debug cargo run --release -p picostream-firmware

set -e

cd "$(dirname "$0")"

echo "Testing picostream client..."
echo "Sending 1KB of 0xAA pattern at 1MHz..."

# Generate 1KB of test data and stream it
dd if=/dev/zero bs=1024 count=1 2>/dev/null | tr '\0' '\252' | \
    RUST_LOG=debug cargo run --release -p picostream -- -s 1000000

echo "Done."

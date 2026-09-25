#!/bin/bash
# Start teleop (interactive)

RUST_NODES=~/rust_nodes
echo "🎮 Starting Teleop..."
echo "Use WASD to move, SPACE to stop, Ctrl+C to quit"
echo ""
$RUST_NODES/teleop_rust

#!/bin/bash
# Stop all Rust nodes

if [ -f ~/rust_nodes.pid ]; then
    PIDS=$(cat ~/rust_nodes.pid)
    echo "🛑 Stopping nodes: $PIDS"
    kill $PIDS 2>/dev/null
    rm ~/rust_nodes.pid
    echo "✅ All nodes stopped"
else
    echo "No PID file found, killing by name..."
    pkill -f "_rust$" 2>/dev/null
    echo "✅ Done"
fi

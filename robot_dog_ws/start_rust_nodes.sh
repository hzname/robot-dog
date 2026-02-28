#!/bin/bash
# Robot Dog Rust Nodes Launcher

RUST_NODES=~/rust_nodes
LOG_DIR=~/rust_logs
mkdir -p $LOG_DIR

echo "🐕 Starting Robot Dog Rust Nodes..."
echo "Logs: $LOG_DIR"
echo ""

# Start IMU node
echo "📡 Starting IMU node..."
$RUST_NODES/imu_node_rust > $LOG_DIR/imu.log 2>&1 &
IMU_PID=$!
echo "  PID: $IMU_PID"

# Start Balance controller
echo "⚖️  Starting Balance controller..."
$RUST_NODES/balance_controller_rust > $LOG_DIR/balance.log 2>&1 &
BALANCE_PID=$!
echo "  PID: $BALANCE_PID"

# Start Gait controller
echo "🦵 Starting Gait controller..."
$RUST_NODES/gait_controller_rust > $LOG_DIR/gait.log 2>&1 &
GAIT_PID=$!
echo "  PID: $GAIT_PID"

# Start Servo driver
echo "🔌 Starting Servo driver..."
$RUST_NODES/servo_driver_rust > $LOG_DIR/servo.log 2>&1 &
SERVO_PID=$!
echo "  PID: $SERVO_PID"

echo ""
echo "✅ All nodes started!"
echo ""
echo "Commands:"
echo "  ./stop.sh        - Stop all nodes"
echo "  ./teleop.sh      - Start teleop (interactive)"
echo "  tail -f ~/rust_logs/*.log - View logs"
echo ""
echo "PIDs: IMU=$IMU_PID Balance=$BALANCE_PID Gait=$GAIT_PID Servo=$SERVO_PID"

# Save PIDs
echo "$IMU_PID $BALANCE_PID $GAIT_PID $SERVO_PID" > ~/rust_nodes.pid

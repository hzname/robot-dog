#!/bin/bash
#
# Test script for IMU node in simulation mode
# This script tests the IMU node without requiring hardware
#

set -e

echo "======================================"
echo "IMU Node Simulation Test"
echo "======================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Step 1: Checking ROS2 environment...${NC}"
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}Error: ROS2 not found. Please source your ROS2 environment first.${NC}"
    exit 1
fi
echo -e "${GREEN}ROS2 found: $(ros2 --version)${NC}"
echo ""

echo -e "${YELLOW}Step 2: Building package...${NC}"
cd ~/robot-dog/robot_dog_ws
# colcon build --packages-select dog_sensors_cpp --symlink-install
echo -e "${GREEN}Build step skipped (run manually with colcon)${NC}"
echo ""

echo -e "${YELLOW}Step 3: Starting IMU node in simulation mode...${NC}"
echo "Motion mode: walking"
echo "Launch command:"
echo "  ros2 launch dog_sensors_cpp imu_simulation.launch.py motion_mode:=walking"
echo ""

echo -e "${YELLOW}Step 4: Testing topic output...${NC}"
echo "In another terminal, run:"
echo "  ros2 topic echo /imu/data"
echo ""
echo "Or run this script with the 'test' argument for automated testing:"
echo "  ./test_imu_simulation.sh test"
echo ""

# If test argument provided, run automated tests
if [ "$1" == "test" ]; then
    echo -e "${YELLOW}Running automated tests...${NC}"
    
    # Launch the node in background
    ros2 launch dog_sensors_cpp imu_simulation.launch.py &
    NODE_PID=$!
    
    # Wait for node to start
    sleep 5
    
    # Check if topics are published
    echo "Checking published topics..."
    if ros2 topic list | grep -q "/imu/data"; then
        echo -e "${GREEN}✓ /imu/data topic is published${NC}"
    else
        echo -e "${RED}✗ /imu/data topic not found${NC}"
    fi
    
    if ros2 topic list | grep -q "/body_pose"; then
        echo -e "${GREEN}✓ /body_pose topic is published${NC}"
    else
        echo -e "${RED}✗ /body_pose topic not found${NC}"
    fi
    
    if ros2 topic list | grep -q "/imu/status"; then
        echo -e "${GREEN}✓ /imu/status topic is published${NC}"
    else
        echo -e "${RED}✗ /imu/status topic not found${NC}"
    fi
    
    # Check message frequency
    echo ""
    echo "Checking message frequency..."
    timeout 5 ros2 topic hz /imu/data || true
    
    # Cleanup
    echo ""
    echo "Stopping node..."
    kill $NODE_PID 2>/dev/null || true
    
    echo -e "${GREEN}Test complete!${NC}"
fi

echo ""
echo "======================================"
echo "Simulation test ready!"
echo "======================================"
echo ""
echo "Available motion modes:"
echo "  - idle      : Standing still with small vibrations"
echo "  - walking   : Walking gait (default)"
echo "  - trotting  : Faster gait"
echo "  - turning   : Turning in place"
echo "  - pacing    : Pacing gait"
echo ""
echo "To test with different modes:"
echo "  ros2 launch dog_sensors_cpp imu_simulation.launch.py motion_mode:=trotting"
echo ""
echo "To send velocity commands (affects simulation):"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'"
echo ""

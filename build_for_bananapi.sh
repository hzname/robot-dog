#!/bin/bash
# build_for_bananapi.sh — Cross-compile Rust nodes for Banana Pi (ARM64)

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUST_NODES_DIR="$WORKSPACE_DIR/rust_nodes"
TARGET="aarch64-unknown-linux-gnu"

echo "🦀 Building Rust nodes for Banana Pi (ARM64)..."
echo "Workspace: $WORKSPACE_DIR"
echo ""

# Проверка toolchain
if ! rustup target list | grep -q "$TARGET (installed)"; then
    echo "📦 Installing ARM64 target..."
    rustup target add $TARGET
fi

# Создаём директорию для бинарников
mkdir -p $RUST_NODES_DIR

# Сборка каждого пакета
PACKAGES=(
    "dog_control_rust:gait_controller_rust"
    "dog_hardware_rust:servo_driver_rust"
    "dog_sensors_rust:imu_node_rust"
    "dog_balance_rust:balance_controller_rust"
    "dog_teleop_rust:teleop_rust"
)

for pkg in "${PACKAGES[@]}"; do
    IFS=':' read -r dir binary <<< "$pkg"
    echo "🔨 Building $dir → $binary..."
    
    cd "$WORKSPACE_DIR/src/$dir"
    cargo build --release --target $TARGET
    
    # Копируем бинарник
    cp "target/$TARGET/release/$binary" "$RUST_NODES_DIR/"
    echo "  ✅ $binary"
    echo ""
done

echo "📁 Binaries in: $RUST_NODES_DIR"
ls -lh $RUST_NODES_DIR
echo ""
echo "🚀 Ready for deployment! Run:"
echo "  ./deploy_to_bananapi.sh"

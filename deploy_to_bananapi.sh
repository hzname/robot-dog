#!/bin/bash
# deploy_to_bananapi.sh — Deploy to Banana Pi via SSH

BANANA_IP="${1:-banana-pi.local}"
BANANA_USER="${2:-banana}"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "🚀 Deploying to Banana Pi: $BANANA_USER@$BANANA_IP"
echo ""

# Проверка доступности
if ! ping -c 1 $BANANA_IP > /dev/null 2>&1; then
    echo "❌ Banana Pi not reachable at $BANANA_IP"
    echo "Usage: ./deploy_to_bananapi.sh [IP] [user]"
    exit 1
fi

# Создаём директории на Banana Pi
echo "📂 Creating directories..."
ssh $BANANA_USER@$BANANA_IP "mkdir -p ~/robot_dog_ws/rust_nodes ~/robot_dog_ws/install"

# Копируем файлы
echo "📦 Copying binaries..."
rsync -avz --progress "$WORKSPACE_DIR/rust_nodes/" $BANANA_USER@$BANANA_IP:~/robot_dog_ws/rust_nodes/

echo "📦 Copying install files..."
rsync -avz --progress "$WORKSPACE_DIR/install/" $BANANA_USER@$BANANA_IP:~/robot_dog_ws/install/

echo "📄 Copying compose file..."
rsync -avz "$WORKSPACE_DIR/docker-compose.deploy.yml" $BANANA_USER@$BANANA_IP:~/robot_dog_ws/

echo ""
echo "✅ Deployment complete!"
echo ""
echo "To start on Banana Pi:"
echo "  ssh $BANANA_USER@$BANANA_IP"
echo "  cd ~/robot_dog_ws"
echo "  docker-compose -f docker-compose.deploy.yml up -d"

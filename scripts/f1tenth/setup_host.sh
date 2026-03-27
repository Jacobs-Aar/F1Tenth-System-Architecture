#!/usr/bin/env bash
# =============================================================================
# scripts/f1tenth/setup_host.sh
# =============================================================================
# Run this ONCE on a fresh Jetson (or Ubuntu 22.04 VM) to install all
# host-level prerequisites before building the Docker container.
#
# Usage:
#   chmod +x scripts/f1tenth/setup_host.sh
#   ./scripts/f1tenth/setup_host.sh
# =============================================================================

set -euo pipefail

echo "============================================"
echo "  F1Tenth Host Setup Script"
echo "============================================"

# -- 1. System update ---------------------------------------------------------
echo "[1/5] Updating system packages..."
sudo apt-get update && sudo apt-get upgrade -y

# -- 2. Install Docker Engine ------------------------------------------------
echo "[2/5] Installing Docker Engine..."
if ! command -v docker &> /dev/null; then
    # Install using the official convenience script
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    rm get-docker.sh
    # Allow running docker without sudo
    sudo usermod -aG docker "$USER"
    echo "  -> Docker installed. You will need to log out and back in for group changes."
else
    echo "  -> Docker already installed: $(docker --version)"
fi

# -- 3. Install Docker Compose plugin ----------------------------------------
echo "[3/5] Installing Docker Compose plugin..."
if ! docker compose version &> /dev/null; then
    sudo apt-get install -y docker-compose-plugin
else
    echo "  -> Docker Compose already installed: $(docker compose version)"
fi

# -- 4. Install NVIDIA Container Toolkit --------------------------------------
echo "[4/5] Installing NVIDIA Container Toolkit..."
if ! dpkg -l | grep -q nvidia-container-toolkit; then
    # Add the NVIDIA GPG key and repository
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
        sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L "https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list" | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    echo "  -> NVIDIA Container Toolkit installed."
else
    echo "  -> NVIDIA Container Toolkit already installed."
fi

# -- 5. Add user to dialout group (for VESC serial access) -------------------
echo "[5/5] Adding $USER to dialout group (VESC serial access)..."
sudo usermod -aG dialout "$USER"

echo ""
echo "============================================"
echo "  Setup complete!"
echo "============================================"
echo ""
echo "IMPORTANT: Log out and back in for group changes to take effect."
echo ""
echo "Next steps:"
echo "  1. cd docker/f1tenth"
echo "  2. docker compose build"
echo "  3. docker compose up -d"
echo "  4. docker exec -it f1tenth_stack bash"
echo ""

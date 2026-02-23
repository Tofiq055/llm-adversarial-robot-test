#!/bin/bash
# ══════════════════════════════════════════════════════════════════
# A4 Projesi - Minimal Host Hazırlık Scripti
# ══════════════════════════════════════════════════════════════════
# Bu script SADECE Docker ve NVIDIA araçlarını kurar. 
# ROS2, Gazebo, LLM ve diğer her şey Docker içinde çalışacaktır.
# ══════════════════════════════════════════════════════════════════

set -e

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; NC='\033[0m'
ok()   { echo -e "${GREEN}  ✓ $1${NC}"; }
info() { echo -e "${YELLOW}  ℹ $1${NC}"; }
err()  { echo -e "${RED}  ✗ $1${NC}"; }
header() { echo -e "\n${CYAN}══════════════════════════════════════════${NC}"; echo -e "${CYAN}  $1${NC}"; echo -e "${CYAN}══════════════════════════════════════════${NC}"; }

# 1. NVIDIA Sürücü Kontrolü
header "AŞAMA 1: NVIDIA KONTROLÜ"
if ! nvidia-smi &>/dev/null; then
    err "NVIDIA sürücüleri bulunamadı! Lütfen önce kurun:"
    echo "    sudo ubuntu-drivers autoinstall && sudo reboot"
    exit 1
fi
ok "NVIDIA Driver bulundu: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader)"

# 2. Docker Kurulumu
header "AŞAMA 2: DOCKER ENGINE"
if ! command -v docker &>/dev/null; then
    info "Docker kuruluyor..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker $USER
    rm get-docker.sh
    ok "Docker kuruldu. Değişikliklerin aktif olması için oturumu kapatıp açın (veya 'newgrp docker' çalıştırın)."
else
    ok "Docker zaten kurulu."
fi

# 3. NVIDIA Container Toolkit
header "AŞAMA 3: NVIDIA CONTAINER TOOLKIT"
if ! dpkg -l | grep -q nvidia-container-toolkit; then
    info "NVIDIA Container Toolkit kuruluyor..."
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null
    sudo apt-get update -qq
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    ok "NVIDIA Container Toolkit kuruldu."
else
    ok "NVIDIA Container Toolkit zaten kurulu."
fi

header "✅ HAZIRLIK TAMAMLANDI"
echo -e "Şimdi projeyi başlatmak için şu komutu çalıştırabilirsiniz:\n"
echo -e "    ${GREEN}docker compose up --build${NC}\n"

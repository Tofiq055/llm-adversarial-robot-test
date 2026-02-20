#!/bin/bash
# ══════════════════════════════════════════════════════════════════
# Grup A — UR5e / Gazebo / MoveIt2 Proje Kurulum Scripti
# ══════════════════════════════════════════════════════════════════
# Tüm Grup A öğrencileri için genel kurulum scripti.
# Ubuntu 22.04 + NVIDIA GPU sistemlerde çalışır.
#
# Kullanım:  bash setup.sh
# DİKKAT:   "sudo bash" ile ÇALIŞTIRMAYIN!
# ══════════════════════════════════════════════════════════════════

if [ "$(id -u)" -eq 0 ]; then
    echo "  ✗ Bu script'i 'sudo bash' ile çalıştırmayın!"
    echo "  Doğru: bash setup.sh"
    exit 1
fi

set -e

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; NC='\033[0m'
ok()   { echo -e "${GREEN}  ✓ $1${NC}"; }
skip() { echo -e "${YELLOW}  ⏭ $1 (zaten mevcut)${NC}"; }
info() { echo -e "${YELLOW}  ℹ $1${NC}"; }
err()  { echo -e "${RED}  ✗ $1${NC}"; }
header() { echo -e "\n${CYAN}══════════════════════════════════════════${NC}"; echo -e "${CYAN}  $1${NC}"; echo -e "${CYAN}══════════════════════════════════════════${NC}"; }

# ══════════════════════════════════════════
header "KULLANICI BİLGİLERİ"
# ══════════════════════════════════════════

echo ""
echo "  Bu script sisteminizi Grup A projesi için hazırlar."
echo "  ROS2 Humble, Gazebo, MoveIt2, Docker, Ollama kurar."
echo "  GitHub repo ve SSH key ayarlarını yapar."
echo ""

# Varsayılanlar (env'den veya boş)
DEF_NAME="${GIT_USER_NAME:-$(git config --global user.name 2>/dev/null)}"
DEF_EMAIL="${GIT_USER_EMAIL:-$(git config --global user.email 2>/dev/null)}"
DEF_GITHUB="${GITHUB_USER:-}"

read -p "  Adınız [$DEF_NAME]: " INPUT_NAME
GITHUB_NAME="${INPUT_NAME:-$DEF_NAME}"

read -p "  E-posta [$DEF_EMAIL]: " INPUT_EMAIL
GITHUB_EMAIL="${INPUT_EMAIL:-$DEF_EMAIL}"

read -p "  GitHub kullanıcı adı [$DEF_GITHUB]: " INPUT_GITHUB
GITHUB_USER="${INPUT_GITHUB:-$DEF_GITHUB}"

if [ -z "$GITHUB_NAME" ] || [ -z "$GITHUB_EMAIL" ] || [ -z "$GITHUB_USER" ]; then
    err "Tüm alanlar zorunlu! Tekrar çalıştırın."
    exit 1
fi

REPO_NAME="llm-adversarial-robot-test"

echo ""
ok "Kullanıcı: $GITHUB_NAME <$GITHUB_EMAIL> (@$GITHUB_USER)"

# ══════════════════════════════════════════
header "BÖLÜM 1: SİSTEM KONTROLÜ"
# ══════════════════════════════════════════

echo "  OS:     $(lsb_release -d -s 2>/dev/null || echo 'bilinmiyor')"
echo "  Kernel: $(uname -r)"
echo "  GPU:    $(lspci | grep -i nvidia | head -1 | sed 's/.*: //' 2>/dev/null || echo 'NVIDIA yok')"
echo "  RAM:    $(free -h | grep Mem | awk '{print $2}')"
echo "  Disk:   $(df -h / | tail -1 | awk '{print $4}') boş"

if ! nvidia-smi &>/dev/null; then
    err "NVIDIA driver kurulu değil!"
    echo "    sudo apt update && sudo ubuntu-drivers autoinstall && sudo reboot"
    exit 1
fi
ok "NVIDIA Driver: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader)"

# ══════════════════════════════════════════
header "BÖLÜM 2: SWAP"
# ══════════════════════════════════════════

if swapon --show --noheadings 2>/dev/null | grep -q .; then
    skip "Swap mevcut: $(swapon --show --noheadings | awk '{print $3}')"
else
    info "8 GB swap oluşturuluyor..."
    sudo fallocate -l 8G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    grep -q '/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab > /dev/null
    sudo sysctl vm.swappiness=10 > /dev/null
    grep -q 'vm.swappiness' /etc/sysctl.conf || echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf > /dev/null
    ok "8 GB swap oluşturuldu"
fi

# ══════════════════════════════════════════
header "BÖLÜM 3: GPU + PPA TEMİZLİĞİ"
# ══════════════════════════════════════════

PM=$(nvidia-smi -q | grep "Persistence Mode" | head -1 | awk '{print $NF}')
[ "$PM" = "Enabled" ] && skip "GPU persistence mode aktif" || { sudo nvidia-smi -pm 1 > /dev/null; ok "GPU persistence mode aktif"; }

# Bozuk PPA temizliği
if [ -f /etc/apt/sources.list.d/noobslab-ubuntu-apps-jammy.list ]; then
    sudo rm -f /etc/apt/sources.list.d/noobslab-ubuntu-apps-jammy.list
    info "Bozuk PPA kaldırıldı"
fi

# ══════════════════════════════════════════
header "BÖLÜM 4: TEMEL ARAÇLAR"
# ══════════════════════════════════════════

PKGS=""
for p in cmake curl wget htop nvtop tmux git build-essential pkg-config libssl-dev locales software-properties-common ca-certificates gnupg; do
    dpkg -l "$p" &>/dev/null || PKGS="$PKGS $p"
done
if [ -n "$PKGS" ]; then
    sudo apt update -qq && sudo apt install -y -qq $PKGS
    ok "Temel araçlar kuruldu"
else
    skip "Tüm temel araçlar kurulu"
fi

locale | grep -q "en_US.UTF-8" && skip "Locale OK" || {
    sudo locale-gen en_US en_US.UTF-8 > /dev/null 2>&1
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 > /dev/null 2>&1
    export LANG=en_US.UTF-8
    ok "Locale ayarlandı"
}

# ══════════════════════════════════════════
header "BÖLÜM 5: ROS2 HUMBLE"
# ══════════════════════════════════════════

if dpkg -l ros-humble-desktop &>/dev/null 2>&1; then
    skip "ROS2 Humble Desktop kurulu"
else
    info "ROS2 Humble kuruluyor (~15-20 dk)..."
    [ -f /usr/share/keyrings/ros-archive-keyring.gpg ] || \
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    [ -f /etc/apt/sources.list.d/ros2.list ] || {
        sudo add-apt-repository universe -y > /dev/null 2>&1
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
            http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
            sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    }
    sudo apt update -qq && sudo apt install -y ros-humble-desktop
    ok "ROS2 Humble kuruldu"
fi

grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro=humble 2>/dev/null || true

# ══════════════════════════════════════════
header "BÖLÜM 6: GAZEBO + MOVEIT2 + ros2_control"
# ══════════════════════════════════════════

ROS_PKGS=""
for p in ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
    ros-humble-moveit ros-humble-moveit-setup-assistant ros-humble-moveit-visual-tools \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap \
    ros-humble-ur-msgs \
    python3-colcon-common-extensions python3-vcstool; do
    dpkg -l "$p" &>/dev/null 2>&1 || ROS_PKGS="$ROS_PKGS $p"
done
if [ -n "$ROS_PKGS" ]; then
    sudo apt install -y $ROS_PKGS
    ok "Gazebo, MoveIt2, ros2_control kuruldu"
else
    skip "Tüm ROS2 paketleri kurulu"
fi

# ══════════════════════════════════════════
header "BÖLÜM 7: UR5e WORKSPACE (STARTER KIT)"
# ══════════════════════════════════════════

# Conda'yı devre dışı bırak (ROS2 uyumluluğu)
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    eval "$(conda shell.bash hook)" 2>/dev/null || true
    conda deactivate 2>/dev/null || true
    export PATH=$(echo $PATH | tr ':' '\n' | grep -v anaconda | grep -v miniconda | tr '\n' ':')
    ok "Sistem Python aktif: $(/usr/bin/python3 --version)"
fi

# Driver gereksiz — sadece simülasyon
[ -d ~/ur5e_ws/src/Universal_Robots_ROS2_Driver ] && {
    rm -rf ~/ur5e_ws/src/Universal_Robots_ROS2_Driver ~/ur5e_ws/build ~/ur5e_ws/install ~/ur5e_ws/log
    info "Gereksiz Driver repo kaldırıldı"
}

UR5E_COUNT=$(ls ~/ur5e_ws/install/ 2>/dev/null | wc -l)
if [ "$UR5E_COUNT" -ge 4 ]; then
    skip "UR5e workspace derlenmiş ($UR5E_COUNT paket)"
else
    info "UR5e workspace kuruluyor (~10-15 dk)..."
    mkdir -p ~/ur5e_ws/src
    [ -d ~/ur5e_ws/src/Universal_Robots_ROS2_Description ] || \
        git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ~/ur5e_ws/src/Universal_Robots_ROS2_Description
    [ -d ~/ur5e_ws/src/Universal_Robots_ROS2_Gazebo_Simulation ] || \
        git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git ~/ur5e_ws/src/Universal_Robots_ROS2_Gazebo_Simulation
    cd ~/ur5e_ws && rm -rf build install log
    source /opt/ros/humble/setup.bash
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ok "UR5e workspace derlendi"
fi

grep -q "source ~/ur5e_ws/install/setup.bash" ~/.bashrc || echo "source ~/ur5e_ws/install/setup.bash" >> ~/.bashrc

# ══════════════════════════════════════════
header "BÖLÜM 8: DOCKER + NVIDIA CONTAINER TOOLKIT"
# ══════════════════════════════════════════

if command -v docker &>/dev/null; then
    skip "Docker: $(docker --version | head -1)"
else
    info "Docker kuruluyor..."
    sudo install -m 0755 -d /etc/apt/keyrings
    [ -f /etc/apt/keyrings/docker.gpg ] || {
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
        sudo chmod a+r /etc/apt/keyrings/docker.gpg
    }
    [ -f /etc/apt/sources.list.d/docker.list ] || {
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
            https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | \
            sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    }
    sudo apt update -qq
    sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    sudo usermod -aG docker $USER
    ok "Docker kuruldu (oturum kapatıp açın veya: newgrp docker)"
fi

if dpkg -l nvidia-container-toolkit &>/dev/null 2>&1; then
    skip "NVIDIA Container Toolkit"
else
    info "NVIDIA Container Toolkit kuruluyor..."
    [ -f /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg ] || {
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
            sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    }
    [ -f /etc/apt/sources.list.d/nvidia-container-toolkit.list ] || {
        curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null
    }
    sudo apt update -qq && sudo apt install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    ok "NVIDIA Container Toolkit kuruldu"
fi

# ══════════════════════════════════════════
header "BÖLÜM 9: CONDA ORTAMI"
# ══════════════════════════════════════════

# Conda PATH geri yükle
if ! command -v conda &>/dev/null; then
    [ -d "$HOME/anaconda3/bin" ] && export PATH="$HOME/anaconda3/bin:$PATH"
    [ -d "$HOME/miniconda3/bin" ] && export PATH="$HOME/miniconda3/bin:$PATH"
    eval "$(conda shell.bash hook)" 2>/dev/null || true
fi

if command -v conda &>/dev/null; then
    conda env list | grep -q "^a4 " && skip "Conda 'a4' ortamı" || {
        conda create -n a4 python=3.11 -y
        ok "Conda 'a4' ortamı oluşturuldu"
    }
    eval "$(conda shell.bash hook)" 2>/dev/null || true
    conda activate a4
    pip install --quiet transformers accelerate bitsandbytes huggingface-hub \
        pandas matplotlib tabulate jinja2 pyyaml python-dotenv \
        pytest pytest-cov flake8 ollama 2>/dev/null || true
    conda deactivate
    ok "Python paketleri kuruldu"
else
    info "Conda kurulu değil — conda gerekiyorsa: https://docs.conda.io/en/latest/miniconda.html"
fi

# ══════════════════════════════════════════
header "BÖLÜM 10: OLLAMA (YEREL LLM)"
# ══════════════════════════════════════════

if command -v ollama &>/dev/null; then
    skip "Ollama kurulu"
else
    curl -fsSL https://ollama.com/install.sh | sh
    ok "Ollama kuruldu"
fi

# ══════════════════════════════════════════
header "BÖLÜM 11: SSH KEY + GITHUB"
# ══════════════════════════════════════════

git config --global user.name "$GITHUB_NAME"
git config --global user.email "$GITHUB_EMAIL"
git config --global init.defaultBranch main

[ -f ~/.ssh/id_ed25519 ] && skip "SSH key mevcut" || {
    ssh-keygen -t ed25519 -C "$GITHUB_EMAIL" -f ~/.ssh/id_ed25519 -N ""
    ok "SSH key oluşturuldu"
}
eval "$(ssh-agent -s)" > /dev/null 2>&1
ssh-add ~/.ssh/id_ed25519 2>/dev/null || true

if gh auth status &>/dev/null 2>&1; then
    skip "GitHub CLI giriş yapılmış"
else
    info "GitHub'a giriş yapılıyor (tarayıcı açılacak)..."
    gh auth login -p ssh -h github.com -w
fi

# SSH key GitHub'a ekle
FP=$(ssh-keygen -lf ~/.ssh/id_ed25519.pub | awk '{print $2}')
if gh ssh-key list 2>/dev/null | grep -q "$FP"; then
    skip "SSH key GitHub'da"
else
    if ! gh ssh-key add ~/.ssh/id_ed25519.pub --title "Ubuntu-$(hostname)" 2>/dev/null; then
        gh auth refresh -h github.com -s admin:public_key
        gh ssh-key add ~/.ssh/id_ed25519.pub --title "Ubuntu-$(hostname)"
    fi
    ok "SSH key GitHub'a eklendi"
fi

# ══════════════════════════════════════════
header "BÖLÜM 12: PROJE REPOSU KLONLAMA"
# ══════════════════════════════════════════

REPO_DIR=~/Documents/$REPO_NAME
if [ -d "$REPO_DIR/.git" ]; then
    skip "Repo zaten klonlanmış: $REPO_DIR"
    cd "$REPO_DIR" && git pull origin main 2>/dev/null || true
else
    info "Repo klonlanıyor..."
    mkdir -p ~/Documents
    ssh -o StrictHostKeyChecking=no -T git@github.com 2>/dev/null || true
    git clone "git@github.com:Tofiq055/$REPO_NAME.git" "$REPO_DIR"
    ok "Repo klonlandı: $REPO_DIR"
fi

# ══════════════════════════════════════════
header "BÖLÜM 13: DOĞRULAMA"
# ══════════════════════════════════════════

PASS=0; FAIL=0; WARN=0
p() { echo -e "${GREEN}  ✅ $1${NC}"; PASS=$((PASS+1)); }
f() { echo -e "${RED}  ❌ $1${NC}  → $2"; FAIL=$((FAIL+1)); }
w() { echo -e "${YELLOW}  ⚠️  $1${NC}"; WARN=$((WARN+1)); }

swapon --show --noheadings 2>/dev/null | grep -q . && p "Swap" || f "Swap yok" "Tekrar çalıştır"
nvidia-smi &>/dev/null && p "GPU Driver" || f "Driver yok" "sudo ubuntu-drivers autoinstall"
[ -f /opt/ros/humble/setup.bash ] && p "ROS2 Humble" || f "ROS2 yok" "Tekrar çalıştır"
dpkg -l ros-humble-gazebo-ros-pkgs 2>/dev/null | grep -q "^ii" && p "Gazebo" || f "Gazebo yok" ""
dpkg -l ros-humble-moveit 2>/dev/null | grep -q "^ii" && p "MoveIt2" || f "MoveIt2 yok" ""
[ "$(ls ~/ur5e_ws/install/ 2>/dev/null | wc -l)" -ge 4 ] && p "UR5e workspace" || f "UR5e derlenmemiş" ""
command -v docker &>/dev/null && p "Docker" || f "Docker yok" ""
command -v ollama &>/dev/null && p "Ollama" || f "Ollama yok" ""
command -v colcon &>/dev/null && p "colcon" || f "colcon yok" ""
[ -f ~/.ssh/id_ed25519 ] && p "SSH key" || f "SSH key yok" ""
[ -d "$REPO_DIR/.git" ] && p "Repo klonlandı" || f "Repo yok" ""
groups | grep -q docker && p "Docker grubu" || w "docker grubunda değil: sudo usermod -aG docker \$USER && newgrp docker"

echo ""
echo -e "  ${GREEN}✅ $PASS${NC}  ${YELLOW}⚠️ $WARN${NC}  ${RED}❌ $FAIL${NC}"

[ $FAIL -eq 0 ] && header "✅ KURULUM TAMAMLANDI!" || header "⚠️  $FAIL sorun var — yukarıya bak"

echo ""
echo "  Sonraki adımlar:"
echo "    1. Yeni terminal aç"
echo "    2. ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur5e"
echo "    3. docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi"
echo "    4. ollama pull <model>"
echo ""

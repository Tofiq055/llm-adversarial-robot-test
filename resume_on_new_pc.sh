#!/bin/bash
# ══════════════════════════════════════════════════════════════════
# A4 Projesi — Başka PC'den Devam Etme Scripti
# ══════════════════════════════════════════════════════════════════
# Bu scripti yeni bir bilgisayarda çalıştırarak projeyi
# kaldığın yerden devam ettirirsin.
#
# Gereksinimler: Ubuntu 22.04 + NVIDIA GPU (driver kurulu)
# ══════════════════════════════════════════════════════════════════

set -e

GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
ok()     { echo -e "${GREEN}  ✓ $1${NC}"; }
header() { echo -e "\n${CYAN}══ $1 ══${NC}"; }

header "ADIM 1: Repo Klonla ve Branch'e Geç"
if [ ! -d ".git" ]; then
    git clone https://github.com/Tofiq055/llm-adversarial-robot-test.git
    cd llm-adversarial-robot-test
fi
git checkout a4/tofiq
ok "a4/tofiq branch'inde"

header "ADIM 2: Host Hazırlığı (Docker + NVIDIA)"
bash setup_host.sh
ok "Host hazır"

header "ADIM 3: Docker Konteynerlerini İnşa Et ve Başlat"
docker compose up -d --build
ok "Konteynerler ayakta"

header "ADIM 4: Konfigürasyon ve Modelleri Hazırla"
if [ ! -f ".env" ]; then
    cp .env.example .env
    echo "  ℹ .env oluşturuldu. Lütfen içindeki GITHUB_TOKEN'ı güncelleyin."
fi

# Base model çek
docker compose exec ollama ollama pull qwen2.5-coder:3b
ok "Base model (qwen2.5-coder:3b) hazır"

# Fine-tuned model kontrolü
if [ -f "qwen2.5-coder-3b-ros2-lora.gguf" ]; then
    header "Fine-tuned Model Yükleniyor"
    docker cp Modelfile.qwen-ros2 a4_ollama:/Modelfile.qwen-ros2
    docker cp qwen2.5-coder-3b-ros2-lora.gguf a4_ollama:/qwen2.5-coder-3b-ros2-lora.gguf
    docker exec -i a4_ollama ollama create a4-qwen-ros2 -f /Modelfile.qwen-ros2
    ok "a4-qwen-ros2 modeli oluşturuldu"
else
    echo -e "\n⚠️  UYARI: qwen2.5-coder-3b-ros2-lora.gguf bulunamadı!"
    echo "Lütfen bu dosyayı manuel olarak kopyalayın ve şunu çalıştırın:"
    echo "  docker exec -i a4_ollama ollama create a4-qwen-ros2 -f /Modelfile.qwen-ros2"
fi

header "ADIM 5: Bağlantı Testi"
docker compose exec testrunner python3 /app/scripts/hello_llm.py

header "✅ HAZIRSIN! Kaldığın yerden devam edebilirsin."
echo -e "Faydalı komutlar:"
echo -e "  ${GREEN}docker compose ps${NC}                    → Konteyner durumları"
echo -e "  ${GREEN}docker compose exec -it ollama ollama run a4-qwen-ros2${NC}  → Fine-tuned model ile sohbet"
echo -e "  ${GREEN}xhost +local:docker && docker compose exec sim bash -c '...'${NC} → Gazebo aç"

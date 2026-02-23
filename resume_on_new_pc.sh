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

header "ADIM 4: LLM Modelini İndir (~4.1 GB)"
echo "İndirmek istiyor musun? (y/n)"
read -r answer
if [ "$answer" = "y" ]; then
    docker compose exec ollama ollama pull dolphin-mistral:7b
    ok "dolphin-mistral:7b indirildi"
else
    echo "  ℹ Sonra indirmek için: docker compose exec ollama ollama pull dolphin-mistral:7b"
fi

header "ADIM 5: Bağlantı Testi"
docker compose exec testrunner python /app/scripts/hello_llm.py

header "✅ HAZIRSIN! Kaldığın yerden devam edebilirsin."
echo -e "Faydalı komutlar:"
echo -e "  ${GREEN}docker compose ps${NC}                    → Konteyner durumları"
echo -e "  ${GREEN}docker compose exec -it ollama ollama run dolphin-mistral:7b${NC}  → LLM ile sohbet"
echo -e "  ${GREEN}xhost +local:docker && docker compose exec sim bash -c '...'${NC} → Gazebo aç"

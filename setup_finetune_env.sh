#!/bin/bash

# Renkli Çıktılar
GREEN='\03[0;32m'
CYAN='\03[0;36m'
NC='\03[0m' # No Color

echo -e "${CYAN}🚀 Qwen2.5-Coder:3B Fine-Tuning Ortamı Kuruluyor...${NC}"
echo "6GB VRAM (RTX 3060) İçin Optimize Edildi."

# 1. Sanal ortamı (venv) oluştur (Host sistemi kirletmemek için)
if [ ! -d "unsloth_venv" ]; then
    echo -e "${CYAN}[1/4] Sanal ortam 'unsloth_venv' oluşturuluyor...${NC}"
    sudo apt-get install -y python3-venv
    python3 -m venv unsloth_venv
else
    echo -e "${GREEN}[1/4] 'unsloth_venv' zaten var, atlanıyor.${NC}"
fi

# 2. Sanal ortama geçiş yap
echo -e "${CYAN}[2/4] Sanal ortama aktif ediliyor...${NC}"
source unsloth_venv/bin/activate

# 3. PyTorch ve CUDA'yı yükle
echo -e "${CYAN}[3/4] PyTorch ve Unsloth Bağımlılıkları Yükleniyor...${NC}"
pip install --upgrade pip
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# 4. HuggingFace kütüphaneleri (trl, peft, bitsandbytes vb.)
echo -e "${CYAN}[4/4] Yapay zeka kütüphaneleri indiriliyor...${NC}"
pip install "transformers==4.46.3" "trl==0.8.6" peft accelerate bitsandbytes datasets typing_extensions

echo -e "\n${GREEN}🔥 BÜTÜN KURULUMLAR TAMAMLANDI!${NC}"
echo -e "Eğitimi lokalde doğrudan başlatmak için aşağıdaki komutu terminalinize yapıştırıp ENTER'a basın:\n"
echo -e "${CYAN}source unsloth_venv/bin/activate && python3 finetune_qwen.py${NC}\n"

# 🔄 A4 Projesi — Başka PC'ye Geçiş Rehberi

> **Son Push:** 2026-02-27 | **Branch:** `a4/tofiq` | **Remote:** `origin (GitHub)`

---

## 1. Yeni PC Gereksinimleri

| Gereksinim | Minimum | Önerilen |
|---|---|---|
| **GPU** | NVIDIA (6GB+ VRAM) | RTX 3060+ (12GB+ VRAM) |
| **RAM** | 16GB | 32GB |
| **Disk** | 30GB boş | 50GB boş |
| **OS** | Ubuntu 22.04 | Ubuntu 22.04 |
| **NVIDIA Driver** | 525+ | 535+ |

---

## 2. Adım Adım Kurulum

### 2.1 Temel Araçlar
```bash
# Git
sudo apt update && sudo apt install -y git curl

# Docker & Docker Compose
sudo apt install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER
# Yeniden giriş yap (logout/login)

# NVIDIA Container Toolkit (GPU erişimi için)
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### 2.2 Projeyi Klonla
```bash
cd ~/Documents/github
git clone https://github.com/Tofiq055/llm-adversarial-robot-test.git
cd llm-adversarial-robot-test
git checkout a4/tofiq
```

### 2.3 Docker Konteynerlerini Kur ve Başlat
```bash
# İlk kurulum (image build ~15-20 dk)
docker compose build

# Konteynerleri başlat
docker compose up -d

# Durumu kontrol et
docker compose ps
```

### 2.4 Ollama Modellerini Yükle
```bash
# Temel model (1.9GB)
docker exec a4_ollama ollama pull qwen2.5-coder:3b

# Fine-tuned modeli kaydet (GGUF dosyası repoda mevcut)
docker exec a4_ollama ollama create a4-qwen-ros2 -f /models/Modelfile.qwen-ros2

# Modelleri doğrula
docker exec a4_ollama ollama list
```

### 2.5 Fine-Tuning Ortamını Kur (Opsiyonel — eğitim yapacaksan)
```bash
# Python venv
python3 -m venv venv
source venv/bin/activate

# Gerekli kütüphaneler
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip install transformers peft trl datasets bitsandbytes accelerate
pip install ollama
```

---

## 3. Mevcut Durumun Özeti (Nereden Devam Edilecek)

### ✅ Tamamlanan İşler
| İş | Durum | Konum |
|---|---|---|
| Docker 3-konteyner mimarisi | ✅ Çalışıyor | `docker-compose.yml` |
| 65 adversarial prompt | ✅ Hazır | `data/prompts/` |
| Test Pipeline (`test_runner.py`) | ✅ Çalışıyor | `src/.../scripts/test_runner.py` |
| Safety Listener (TDD) | ✅ Çalışıyor | `src/.../scripts/safety_listener.py` |
| Safety Analyzer | ✅ Çalışıyor | `src/.../scripts/safety_analyzer.py` |
| HAM model (qwen2.5-coder:3b) 65-prompt testi | ✅ Tamamlandı | `data/generated_scripts/*_qwen2.5_coder_3b.py` |
| Fine-tuned v1 (a4-qwen-ros2) 65-prompt testi | ✅ Tamamlandı | `data/generated_scripts/*_a4_qwen_ros2.py` |
| Karşılaştırma raporu | ✅ Yazıldı | `data/results/experiment_report.md` |
| CSV sonuçlar | ✅ Kaydedildi | `data/results/experiment_results.csv` |

### 🔜 Devam Edilecek İşler (Fine-Tuning v2)
| İş | Öncelik | Referans |
|---|---|---|
| Bağımsız adversarial promptlar yaz (ezberleme önleme) | **Yüksek** | Karar 21.1 |
| Kötü amaçlı ROS2 kodu veri seti üret (LLM ile) | **Yüksek** | Karar 21.2 |
| GitHub'dan ek güvenli kod kazı | Orta | Karar 21.2 |
| System prompt tasarla | **Yüksek** | Karar 21.5 |
| Eğitim parametrelerini güncelle (epoch≥3, r=16) | **Yüksek** | Karar 21.3 |
| Cloud eğitim platformu araştır | Orta | Karar 21.4 |
| Fine-tuned v2 modelini eğit + test et | **Yüksek** | Checklist |

---

## 4. Hızlı Doğrulama (Her Şey Çalışıyor mu?)

```bash
cd ~/Documents/github/llm-adversarial-robot-test

# 1. Docker servisleri çalışıyor mu?
docker compose ps
# Beklenen: ollama, sim, testrunner → "running"

# 2. Ollama yanıt veriyor mu?
curl -s http://127.0.0.1:11434/api/tags | python3 -m json.tool | head -5

# 3. Model çalışıyor mu?
curl -s http://127.0.0.1:11434/api/generate \
  -d '{"model":"a4-qwen-ros2","prompt":"print hello","stream":false}' | \
  python3 -c "import sys,json; print(json.load(sys.stdin)['response'])"

# 4. Test pipeline çalışıyor mu? (2 prompt ile smoke test)
docker exec -e PYTHONUNBUFFERED=1 a4_testrunner \
  python /app/scripts/test_runner.py --model a4-qwen-ros2 --limit 2

# 5. Safety Listener testleri geçiyor mu?
docker exec a4_testrunner python -m pytest /app/tests/test_safety_listener.py -v
```

---

## 5. Önemli Dosyalar

| Dosya | Açıklama |
|---|---|
| `guidelines.md` | Proje kuralları (10 kural) |
| `docs/PROJECT_DECISIONS.md` | Tüm teknik kararlar (21 karar) |
| `docs/TASK_CHECKLIST.md` | Görev listesi |
| `bitirmetezibilgi.md` | Tez gereksinimleri |
| `finetune_qwen.py` | Fine-tuning scripti |
| `ros2_dataset.jsonl` | Eğitim veri seti (89 satır) |
| `qwen2.5-coder-3b-ros2-lora.gguf` | Fine-tuned LoRA ağırlıkları |
| `Modelfile.qwen-ros2` | Ollama model tanımı |
| `docker-compose.yml` | 3-konteyner mimarisi |

---

*Oluşturulma tarihi: 2026-02-27*

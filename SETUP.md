# 🚀 Kurulum Rehberi

Bu rehber, **Grup A** projeleri için Ubuntu 22.04 + NVIDIA GPU sisteminizi hazırlar.

## Gereksinimler

| Bileşen | Minimum |
|---|---|
| OS | Ubuntu 22.04 LTS |
| GPU | NVIDIA (driver kurulu) |
| RAM | 8 GB+ (16 GB önerilir) |
| Disk | 30 GB boş alan |

## Hızlı Kurulum

```bash
# 1. Repo'yu klonla
git clone https://github.com/Tofiq055/llm-adversarial-robot-test.git
cd llm-adversarial-robot-test

# 2. Script'i çalıştır
bash setup.sh
```

Script **interaktif** olarak adınızı, e-postanızı ve GitHub kullanıcı adınızı sorar.
Zaten kurulu bileşenleri otomatik atlar (tekrar çalıştırılabilir).

## Ne Kurulur?

| Bileşen | Açıklama |
|---|---|
| Swap 8 GB | LLM inference için gerekli |
| ROS2 Humble | Robot framework |
| Gazebo Classic 11 | Simülasyon ortamı |
| MoveIt2 | Hareket planlama |
| UR5e Workspace | Simülasyon starter kit |
| Docker + NVIDIA Toolkit | Container'lı çalışma |
| Ollama | Yerel LLM inference |
| Conda (a4) | Python 3.11 ortamı |
| SSH + GitHub | Repo erişimi |

## Kurulum Sonrası Test

```bash
# Yeni terminal aç, sonra:

# UR5e simülasyon
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur5e

# Docker GPU
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi

# LLM modeli çek
ollama pull codellama:7b-code
```

## Branch Yapısı

```
main ──────── stabil, birleşik
  └── dev ──── günlük entegrasyon
        ├── a4/tofiq ── adversarial test
        └── a2/elvin ── safety supervisor
```

**Çalışma akışı:** Kendi branch'inizde geliştirin → PR ile `dev`'e merge edin.

## Sorun mu var?

Script sonunda **DOĞRULAMA RAPORU** gösterir. ❌ olan maddelerin yanında FIX komutu yazar.

Hâlâ sorun varsa GitHub Issues'ta bildirin.

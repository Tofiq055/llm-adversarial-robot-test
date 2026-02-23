# 🚀 Hızlı Kurulum Rehberi (Docker Mimarisi)

Bu rehber, **Grup A** (A4 Adversarial Test ve A2 Safety Supervisor) projeleri için sisteminizi hazırlar.

**DİKKAT:** Proje host (yerel) bilgisayar üzerinde DEĞİL, tamamen izole Docker konteynerleri içinde çalışacak şekilde tasarlanmıştır.

## Gereksinimler

| Bileşen | Minimum |
|---|---|
| OS | Ubuntu 22.04 LTS |
| GPU | NVIDIA (driver kurulu olmalı) |
| RAM | 8 GB+ (16 GB önerilir) |
| Disk | 30 GB boş alan |

> [!important]
> **RTX 3060 Kısıtlaması**: Sistemin aynı anda hem 3D simülasyon modelini (Gazebo) oluşturması hem de Yerel YZ modelini (Ollama) GPU üzerinde tutabilmesi için VRAM'in (6GB) dikkatli kullanılması gerekir.

---

## Kurulum Adımları

### 1. Host Hazırlığı (Sadece 1 Kere)

İşletim sisteminizde sadece **NVIDIA sürücüsü**, **Docker** ve **NVIDIA Container Toolkit** olması yeterlidir. Bunları kurmak için:

```bash
# Repo'yu klonla
git clone https://github.com/Tofiq055/llm-adversarial-robot-test.git
cd llm-adversarial-robot-test

# Host hazırlık betiğini çalıştır
bash setup_host.sh
```

*(Eğer betik `newgrp docker` veya oturumu kapatıp açmanızı isterse mutlaka yapın).*

### 2. Sistemi Ayağa Kaldırmak (Docker Compose)

Tüm simülasyon, test ortamı ve yapay zeka altyapısını başlatmak için:

```bash
docker compose up --build
```

Bu komut 3 adet konteyner ayağa kaldırır:
1. `sim`: UR5e, Gazebo, ROS2 ve MoveIt2.
2. `ollama`: Yerel yapay zeka motoru.
3. `testrunner`: Python ile yazılmış test scriptlerini çalıştıracağınız izole alan.

---

## 3. Doğrulama (Starter Kit Testleri)

Sistem ayağa kalktıktan sonra, projenin doğru çalıştığını test etmek için:

**A. UR5e Gazebo Simülasyonunu Açmak:**
```bash
# Host terminalinde X11 görüntü aktarımına izin ver:
xhost +local:docker

# Simülasyonu başlat:
docker compose exec sim bash -c "ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur5e"
```

**B. Ollama LLM Testi:**
```bash
# Ollama'dan Llama3 (veya codellama) modelini çekin:
docker compose exec ollama ollama pull codellama:7b-code

# Modelin çalıştığını teyit edin:
curl -X POST http://localhost:11434/api/generate -d '{"model":"codellama:7b-code","prompt":"python hello world print"}'
```

---

## Geliştirme Akışı

Projenizin kaynak kodları (`src/`, `data/`) yerel bilgisayarınızdaki klasörlerle **eşzamanlı (volume mapping)** olarak konteynerlere bağlıdır.
- Kodlarınızı kendi IDE'nizle (VS Code vb.) dışarıda düzenleyebilirsiniz.
- Çalıştırmak için `docker compose exec testrunner ...` komutlarını kullanabilirsiniz.

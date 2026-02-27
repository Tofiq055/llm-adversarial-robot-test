# 🚚 Proje Göç Rehberi (Migration Guide)

Bu rehber, projenizi mevcut bilgisayarınızdan başka bir bilgisayara (veya temiz kurulum yapılmış bir sisteme) hatasız bir şekilde taşımanız için gereken adımları içerir.

## 📦 1. Taşınacak Dosyalar (Manuel Transfer)

Aşağıdaki dosyalar `.gitignore` içerisinde olduğu için GitHub'a pushlanmamıştır. Bu dosyaları bir USB bellek veya bulut depolama (Drive/Dropbox) üzerinden manuel olarak yeni bilgisayara kopyalamalısınız:

| Dosya / Klasör | Açıklama |
|---|---|
| `.env` | GitHub Token ve API konfigürasyonlarını içerir. |
| `qwen2.5-coder-3b-ros2-lora.gguf` | Eğittiğimiz ana model dosyasıdır (En Kritik Dosya). |
| `qwen2.5_coder_3b_ros2_lora/` | (Opsiyonel) LoRA adaptör klasörü. Sadece tekrar eğitim yapmak isterseniz gereklidir. |

## 🛠️ 2. Yeni Bilgisayarda Kurulum Adımları

### Adım 1: Repo Hazırlığı
Yeni bilgisayarda terminali açın ve projeyi çekin:
```bash
git clone https://github.com/Tofiq055/llm-adversarial-robot-test.git
cd llm-adversarial-robot-test
git checkout a4/tofiq
```

### Adım 2: Manuel Dosyaları Yerleştirme
Manuel olarak kopyaladığınız `.env` ve `.gguf` dosyalarını projenin ana dizinine (`llm-adversarial-robot-test/`) yapıştırın.

### Adım 3: Otomatik Kurulumu Başlatın
Sizin için hazırladığım `resume_on_new_pc.sh` scriptini çalıştırın. Bu script Docker, NVIDIA araçlarını ve Ollama modelini otomatik olarak kuracaktır:
```bash
bash setup_host.sh   # Sadece Docker/NVIDIA kurulu değilse
bash resume_on_new_pc.sh
```

## ✅ 3. Doğrulama (Success Check)

Her şeyin doğru çalıştığını anlamak için şu testi yapın:
```bash
docker compose exec testrunner python3 /app/scripts/hello_llm.py
```
Eğer "Prepared, Captain!" yanıtını alıyorsanız, sistem yeni bilgisayarda başarıyla ayağa kalkmış demektir.

---
> [!IMPORTANT]
> Yeni bilgisayarda da NVIDIA bir ekran kartı ve Ubuntu 22.04+ yüklü olması önerilir.

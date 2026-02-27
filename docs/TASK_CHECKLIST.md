# A4 Project — Gray-Box Adversarial Test Platform

## ✅ Altyapı (Tamamlandı)
- [x] Docker multi-container mimarisi (sim, ollama, testrunner)
- [x] Branch stratejisi (main, dev, a4/tofiq)
- [x] CI/CD + Güvenlik taraması (Trivy/Syft)
- [x] LLM model seçimi ve bağlantı testi (dolphin-mistral:7b)

---

## A4 Adım 1: Görev Tanımları
- [x] 3 robot görevi tanımla (pose, waypoint, pick-place)
- [x] Her görev için doğru/beklenen çıktıyı (ground truth) belirle
- [x] Görev tanımlarını YAML dosyasına yaz (`data/tasks/`)

## A4 Adım 2: Prompt Şablonları (Genişletildi)
- [x] Güvenli baseline prompt'lar oluştur
- [x] Riskli (adversarial) varyantlar oluştur
- [x] Suffix injection varyantları oluştur
- [x] Prompt'ları YAML/JSON olarak `data/prompts/` klasörüne kaydet
- [x] **EK ADIM:** Gemini ile 50 adet yeni "obfuscated" prompt üretilip toplam sayı 65'e çıkarıldı.

## A4 Adım 3: LLM Model Eğitimi (Fine-Tuning) ve Simülasyon Pipeline
- [x] 65 adetlik geniş prompt setiyle `deepseek-coder:6.7b` modelini çalıştırıp ilk CSV raporunu üret (Baseline Test)
- [ ] ROS2 Github Repolarından Script kazıma (veri seti) Python aracı yazılacak ve veriler toplanacak
- [ ] Donanım kısıtlarına uygun şekilde modeli eğit (Unsloth / LoRA adaptasyonu)
- [ ] Eğitilen modeli GGUF formatına dönüştürüp **Hugging Face**'e yükle (Hoca talebi)
- [x] `test_runner.py` — YAML'dan prompt okuyan altyapı (6/6 test)
- [ ] (İPTAL EDİLDİ) `safety_analyzer.py` — Statik güvenlik analizi
- [ ] Üretilen kodu gerçek ROS2/Gazebo simülasyonunda koşturacak pipeline'ı tamamla

## A4 Adım 4: Safety Supervisor Entegrasyonu
- [x] Safety supervisor olaylarını otomatik topla (Pasif Dinleyici ile)
- [x] İhlaller, engellemeler ve timeout'ları kaydet (is_safe_run)
- [ ] Elvin (A2) modülü ile uçtan uca çalıştır

## A4 Adım 5: Metrikler
- [x] Unsafe oranı hesapla → 40/65 (%61.5) skor 0
- [x] Safe başarı oranı hesapla → 3/65 (%4.6) skor ≥60
- [x] Engelleme oranı hesapla → MoveIt2 %38, Collision %25, Velocity %5
- [x] Tespit gecikmesi ölç → Timeout: 2/65, Kod üretemedi: 12/65

## A4 Adım 6: Deney Seti ve Raporlama
- [x] Kendi eğittiğimiz Hugging Face modeli ile 65 deney koşusu yap (a4-qwen-ros2)
- [x] Sonuçları CSV olarak kaydet (`data/results/experiment_results.csv`)
- [x] Özet rapor (Markdown) oluştur (`data/results/experiment_report.md`)
- [ ] Demo videosu çek

---
## 🔄 A4 Fine-Tuning v2: Saldırgan LLM Eğitimi (Güvenlik Bypass)

### Veri Seti Hazırlığı
- [ ] 65 test promptundan **tamamen bağımsız** yeni adversarial promptlar yaz (ezberleme önleme)
- [ ] Kaliteli LLM (Gemini/Claude) ile bu bağımsız promptlar için **kısıtlama aşan zararlı ROS2 kodları** üret
- [ ] GitHub'dan ek güvenli ROS2/MoveIt2 scriptleri kazı (hibrit veri seti)
- [ ] ROS2/MoveIt2/UR5e resmi dokümantasyonlarındaki örnek kodları ekle
- [ ] Veri setini 89 → **500+ satıra** büyüt

### System Prompt Tasarımı
- [ ] Saldırgan kimlik system prompt'u yaz (güvenlik bypass talimatları)
- [ ] System prompt'u eğitim formatına (Alpaca template) entegre et
- [ ] Her eğitim örneğine system prompt eklenmesini sağla

### Eğitim Parametreleri
- [ ] Epoch: 1 → **3-5 (minimum 3)**
- [ ] LoRA rank: 8 → **16-32**
- [ ] Target modules: attention + **MLP (gate/up/down_proj)**
- [ ] max_seq_length: 512 → **1024-2048**
- [ ] Batch size optimizasyonu (VRAM'e göre)

### Cloud Eğitim Araştırması
- [ ] Google Colab (T4 GPU, 15GB VRAM) değerlendir
- [ ] Kaggle Notebooks (P100 GPU, 16GB VRAM) değerlendir
- [ ] Lightning.ai / HF AutoTrain / Vast.ai araştır
- [ ] En uygun platformu seç ve eğitimi çalıştır

### Test ve Karşılaştırma
- [ ] Fine-tuned v2 modelini GGUF'a çevir ve Ollama'ya kaydet
- [ ] 65-prompt Red Teaming testini v2 modelle tekrarla
- [ ] HAM vs FT-v1 vs FT-v2 üçlü karşılaştırma raporu oluştur



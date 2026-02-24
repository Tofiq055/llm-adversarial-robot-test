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
- [ ] Safety supervisor olaylarını otomatik topla
- [ ] İhlaller, engellemeler ve timeout'ları kaydet
- [ ] Elvin (A2) modülü ile uçtan uca çalıştır

## A4 Adım 5: Metrikler
- [ ] Unsafe oranı hesapla (Simülasyon çökmesi veya engellemeye takılma)
- [ ] Safe başarı oranı hesapla (Görev tamamlama)
- [ ] Engelleme oranı hesapla
- [ ] Tespit gecikmesi ölç

## A4 Adım 6: Deney Seti ve Raporlama
- [ ] Kendi eğittiğimiz Hugging Face modeli ile 50+ deney koşusu yap
- [ ] Sonuçları CSV olarak kaydet
- [ ] Özet rapor (Markdown) oluştur
- [ ] Demo videosu çek

---
## 🛠️ A4 Unsloth & QLoRA Altyapı Kurulumu (Yeni)
- [ ] Python sanal ortamı (`venv` veya `conda`)uşturulacak
- [ ] `torch`, `unsloth`, `trl`, `peft` kütüphaneleri kurulacak
- [ ] Hugging Face hesabı ile login (`huggingface-cli login`) işlemi yapılacak
- [ ] ROS2 Github Repolarından Script kazıma (veri seti) Python aracı yazılacak

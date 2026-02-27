# 📊 A4 Adversarial Red Teaming — Experiment Report
> **Model:** `a4-qwen-ros2` (Qwen2.5-Coder:3B + ROS2 LoRA Fine-Tuning)
> **Date:** 2026-02-27
> **Total Experiments:** 65
> **Pipeline:** `test_runner.py` → Ollama → Sandbox (a4_sim) → CSV

---

## 1. Genel Sonuçlar

| Metrik | Değer |
|---|---|
| **Ortalama Güvenlik Skoru** | **16.4/100** |
| **Medyan Skor** | 0/100 |
| **Minimum / Maksimum** | 0 / 80 |
| **Tamamen Güvensiz (Skor 0)** | 40/65 (%61.5) |
| **Güvenli (Skor ≥ 60)** | 3/65 (%4.6) |

---

## 2. Skor Dağılımı

| Aralık | Sayı | Oran | Görsel |
|---|---|---|---|
| 0 (GÜVENSİZ) | 40 | %61.5 | ████████████████████ |
| 1–30 (RİSKLİ) | 9 | %13.8 | ███ |
| 31–59 (RİSKLİ+) | 13 | %20.0 | ████ |
| 60–79 (Kabul Edilebilir) | 1 | %1.5 | ▍ |
| 80–100 (GÜVENLİ) | 2 | %3.1 | ▍ |

---

## 3. Kategori Bazlı Analiz

| Kategori | Ortalama | Güvenli/Toplam | Açıklama |
|---|---|---|---|
| **baseline** (güvenli istem) | 46.7/100 | 2/3 | Model güvenli istemlerde bile tutarsız |
| **suffix_injection** | 36.7/100 | 0/3 | Enjeksiyon kısmen etkili |
| **obfuscated** (gizlenmiş) | 14.8/100 | 1/50 | Obfuscation büyük oranda güvenliği kırıyor |
| **adversarial** (saldırgan) | 8.3/100 | 0/9 | Model saldırgan istemlere çok savunmasız |

---

## 4. Güvenlik Kontrol Oranları

| Kontrol | Evet/ Toplam | Oran |
|---|---|---|
| ✅ MoveIt2 Planlama | 25/65 | %38 |
| ✅ Çarpışma Kontrolü | 16/65 | %25 |
| ✅ Hız Limiti (0.1 scaling) | 3/65 | **%5** |
| ⚠️ Doğrudan Joint Komutu | 23/65 | %35 |
| ✅ Execution Başarılı | 2/65 | %3 |
| ⏱️ Timeout (>30s) | 2/65 | %3 |
| ❌ Kod Üretemedi | 12/65 | %18 |

---

## 5. Risk Seviyesine Göre

| Risk Seviyesi | Ortalama Skor | Deney Sayısı |
|---|---|---|
| safe (güvenli istem) | 46.7/100 | 3 |
| high (yüksek risk) | 19.4/100 | 9 |
| medium (orta risk) | 14.2/100 | 53 |

---

## 6. Görev Tipine Göre

| Görev Tipi | Ortalama Skor | Deney Sayısı |
|---|---|---|
| `task_pick_place` | 24.3/100 | 21 |
| `task_waypoint` | 15.7/100 | 22 |
| `task_pose` | 9.5/100 | 22 |

---

## 7. Kritik Bulgular

1. **Fine-tuning yetersiz:** ROS2 veri seti ile ince ayar yapılmış olmasına rağmen, model adversarial prompt'lar karşısında güvenlik önlemlerini büyük ölçüde ihmal etmektedir.
2. **Hız limiti en zayıf halka:** Sadece **%5** (3/65) deneyinde hız sınırı uygulandı. Bu, modelin `max_velocity_scaling_factor` parametresini öğrenemediğini göstermektedir.
3. **Doğrudan joint komutu:** %35 oranında model MoveIt2'yi atlatarak doğrudan joint seviyesinde komut göndermektedir. Bu, gerçek dünyada felaket senaryosudur.
4. **Kod üretim tutarsızlığı:** %18 oranında model hiç çalıştırılabilir Python kodu üretememiştir.
5. **Obfuscation etkisi:** 50 obfuscated prompt'un 49'u güvensiz sonuç vermiştir (%98 başarısızlık).

---

## 8. Sonuç

> **Qwen2.5-Coder:3B modeli, ROS2 veri seti ile fine-tune edilmiş haliyle bile, adversarial prompt'lar karşısında güvenlik sınırlarını koruyamamaktadır.** Özellikle obfuscation (gizlenmiş) ve adversarial (saldırgan) kategorilerinde model neredeyse tamamen başarısız olmaktadır. Bu, robotik LLM'lerin güvenlik açısından ciddi riskler taşıdığını ve çok daha kapsamlı alignment/güvenlik eğitimine ihtiyaç duyduğunu kanıtlamaktadır.

---

*Veri kaynağı: `data/results/experiment_results.csv` (65 satır, 17 sütun)*
*Test Pipeline: `src/llm_adversarial_test/scripts/test_runner.py`*
*Üretilen Kodlar: `data/generated_scripts/` (137 dosya, 3 model)*

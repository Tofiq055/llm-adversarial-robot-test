# ÇALIŞMA PROGRAMI (PROJE PLANI)

**Dersin Adı:** Graduation Thesis (Bitirme Projesi)  
**Öğrenci Adı Soyadı:** Tofiq Valiyev  
**Grup:** Grup A – Güvenilir Robot Yazılımı (UR5e / Gazebo / MoveIt2)  
**Proje Konusu:** A4 – Gray-Box Adversarial Prompt/Suffix Test Platformu + Simülasyon Güvenlik Skoru  
**Danışman Öğretim Üyesi:** Dr. Yunus Emre Çoğurcu  
**Teslim Tarihi:** 27.02.2026 (Son Tarih: 1 Mart 2026)  

---

## 1. Projenin Amacı ve Kapsamı

Bu projenin temel amacı, endüstriyel robotların (özellikle UR5e) otonom kontrolünde Büyük Dil Modellerinin (LLM) kullanımından doğabilecek güvenlik risklerini nicel olarak ölçebilen bir test platformu geliştirmektir. 

Platform, LLM API'sine gönderilen görev isteklerini (prompt) farklı güvenlik ve zorluk seviyelerinde manipüle ederek (adversarial testing / suffix injection) modelin "unsafe" (güvensiz) robot kontrol kodu üretme eğilimini analiz edecektir. Üretilen kodlar, ros2_control ve MoveIt2 ile çalışan bir UR5e Gazebo simülasyonunda izole bir ortamda (sandbox) test edilecek ve eşzamanlı çalışan "Güvenlik Denetçisi" (Safety Supervisor) tarafından puanlanacaktır.

Temel teslimatlar; otomatik test pipeline'ını yöneten bir Python test runner, 50'den fazla test koşusunu içeren detaylı bir deney seti raporu (CSV + Markdown) ve sistemin çalışma prensibini gösteren bir demo videosudur.

---

## 2. Kullanılacak Yöntem ve Teknolojiler

Proje, tamamen izole edilmiş, tekrarlanabilir bir multi-container Docker mimarisi üzerine inşa edilmektedir:
- **Simülasyon Ortamı:** Gazebo Classic 11, ROS2 Humble, MoveIt2, ur_simulation_gazebo.
- **LLM Entegrasyonu:** Yerel Ollama motoru üzerinden açık kaynaklı, sansürsüz dil modelleri (örn. dolphin-mistral, dolphin-llama3).
- **Test ve Geliştirme:** Python 3.11, `ollama-python` kütüphanesi, Bash scripting, GitHub Actions ile CI/CD (Trivy + Syft taramaları).
- **İletişim Protokolü:** LLM test platformu (Container C) ile Simülasyon (Container A) arasında ROS2 DDS (host ağ modu).

---

## 3. İş Paketleri ve Zaman Çizelgesi (Gantt Planı)

Proje hedeflerine ulaşmak için aşağıdaki iş paketleri planlanmış ve takvimlendirilmiştir:

| İş Paketi (İP) No | İş Paketi Adı | Başlangıç ve Bitiş Süresi | Çıktı / Teslim Formatı |
|:---:|---|---|---|
| **İP-1** | **Literatür Taraması ve Gereksinim Analizi**<br>- LLM güvenlik zafiyetlerinin incelenmesi<br>- UR5e ve MoveIt2 dokümantasyon incelemesi | 10.02.2026 - 16.02.2026 | Gereksinim dokümanı |
| **İP-2** | **Altyapı ve Kurulum (Starter Kit)**<br>- Docker multi-container mimarisinin teşkili<br>- Gazebo + ROS2 + MoveIt2 simülasyonunun testi | 17.02.2026 - 23.02.2026 | `docker-compose.yml`, `Dockerfile` setleri, rosbag kaydı |
| **İP-3** | **Görev ve Prompt Şablonlarının Tasarımı**<br>- En az 3 temel robot görevi (pose, waypoint, pick-place)<br>- Baseline, adversarial ve suffix varyantlı 15+ prompt tasarımı | 24.02.2026 - 02.03.2026 | `ur5e_tasks.yaml`, `adversarial_prompts.yaml` |
| **İP-4** | **Kod Üretimi ve Test Pipeline'ının (Test Runner) Geliştirilmesi**<br>- Ollama API entegrasyonu<br>- Üretilen ROS2/Python kodunun sandbox'ta çalıştırılması | 03.03.2026 - 22.03.2026 | **1. Gelişme Raporu (22 Mart)**<br>Test Runner Python Kodu |
| **İP-5** | **Safety Supervisor Entegrasyonu ve Metrik Toplama**<br>- Denetçi loglarının (hız ihlali, çarpışma) yakalanması<br>- Olayların otomatik kaydedilmesi | 23.03.2026 - 19.04.2026 | **2. Gelişme Raporu (19 Nisan)**<br>Entegrasyon modülü |
| **İP-6** | **Deneylerin Çalıştırılması ve Veri Analizi**<br>- 3 farklı modelle 50+ koşuluk deney setinin otomasyonu<br>- Unsafe oranı, safe başarı oranı metrik hesaplamaları | 20.04.2026 - 10.05.2026 | Deney Sonuçları (CSV Veri Seti) |
| **İP-7** | **Raporlama ve Demo Hazırlığı**<br>- Karşılaştırmalı analiz raporunun yazımı<br>- Sistemin işleyişini anlatan demo videonun kaydı | 11.05.2026 - 31.05.2026 | **Final Raporu (31 Mayıs)**<br>Demo Videosu Linki |

---

## 4. Ara Rapor ve Final Teslim Tarihleri

Bitirme projesi yönergesine uygun olarak aşağıdaki aşamalarda rapor teslimleri gerçekleştirilecektir:

*   **Çalışma Programı (Bu belge):** 27.02.2026 (Son Teslim: 1 Mart 2026)
*   **1. Gelişme Raporu (Ara Sınav Yerine):** 22 Mart 2026
*   **2. Gelişme Raporu:** 19 Nisan 2026
*   **Final Raporu & Demo Videosu:** 31 Mayıs 2026

---
*İşbu Çalışma Programı, danışman öğretim üyesinin onayı ile yürürlüğe girecektir.*

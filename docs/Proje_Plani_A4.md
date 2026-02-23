# ÇALIŞMA PROGRAMI (PROJE PLANI)

**Dersin Adı:** Graduation Thesis (Bitirme Projesi)  
**Öğrenci Adı Soyadı:** Tofiq Valiyev  
**Grup:** Grup A – Güvenilir Robot Yazılımı (UR5e / Gazebo / MoveIt2)  
**Proje Konusu:** A4 – Gray-Box Adversarial Prompt/Suffix Test Platformu + Simülasyon Güvenlik Skoru  
**Danışman Öğretim Üyesi:** Dr. Yunus Emre Çoğurcu  

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

## 3. Proje Aşamaları ve İş Paketleri

Proje, literatür araştırmasından başlayarak test otomasyonuna ve nihai raporlamaya kadar bir dizi mantıksal aşamadan oluşmaktadır. Belirli tarih kısıtlamaları olmaksızın, projenin genel ilerleyişi şu iş paketleri üzerinden sağlanacaktır:

### Aşama 1: Altyapı ve Hazırlık
- **Gereksinim Analizi:** LLM güvenlik zafiyetlerinin incelenmesi ve UR5e/MoveIt2 ortamının araştırılması.
- **Sistem Kurulumu (Starter Kit):** Docker kullanılarak izole edilmiş, çoklu konteyner mimarisine sahip (sim, ollama, testrunner) reprodüksiyon ortamının oluşturulması.
- **Görev Tanımları:** UR5e robotu için test edilecek temel görevlerin (pose, waypoint, pick-place) belirlenmesi ve doğru (ground truth) çıktıların hazırlanması.

### Aşama 2: Test Tasarımı ve Entegrasyon
- **Adversarial Prompt Tasarımı:** Modelin güvenlik mekanizmalarını aşmayı hedefleyen güvenli (baseline), riskli (adversarial) ve dolaylı (obfuscated/suffix) prompt şablonlarının oluşturulması.
- **Kod Üretimi Pipeline'ı:** Promptların otomatik olarak LLM'e (Ollama) iletilmesi, üretilen kodların alınıp formatlanması ve simülasyon (sandbox) konteynerinde çalıştırılmasını sağlayan Python tabanlı "Test Runner"ın geliştirilmesi.
- **Safety Supervisor (Güvenlik Denetçisi) Entegrasyonu:** Simülasyon sırasında oluşan ihlalleri, çarpışmaları ve hız aşımlarını otomatik kaydedecek mekanizmanın entegre edilmesi.

### Aşama 3: Deneyler ve Raporlama
- **Deney Setinin Koşulması:** Geliştirilen pipeline üzerinden, en az 50 test koşusunu barındıran deney setinin farklı LLM modelleriyle (örn: dolphin-mistral:7b) otomatik olarak çalıştırılması.
- **Metrik Toplama ve Veri Analizi:** "Unsafe oranı", "safe başarı oranı" ve "engelleme gecikmesi" gibi metriklerin hesaplanarak CSV formatında dışa aktarılması.
- **Nihai Raporlama:** Tüm analiz sonuçlarını içeren karşılaştırmalı final raporunun (Markdown formatında) yazılması ve sistemin genel çalışma mekanizmasını gösteren demo videosunun hazırlanması.

---

## 4. Raporlama Süreçleri

Bitirme projesi yönergesi kapsamında aşağıdaki genel aşamalar takip edilecektir:
1.  **Çalışma Programı (Bu belge):** Projenin kapsamının, amacının ve genel iş paketlerinin sunulması.
2.  **Gelişme Raporları:** İlerlemelerin, tamamlanan pipeline aşamalarının ve ilk entegrasyon sonuçlarının raporlanması (Ara Rapor niteliğinde).
3.  **Final Raporu:** Tüm deney setinin sonuçlarının, başarı analizlerinin ve proje demosunun nihai olarak belgelenip telim edilmesi.

---
*İşbu Çalışma Programı, danışman öğretim üyesinin onayı ile yürürlüğe girecektir.*

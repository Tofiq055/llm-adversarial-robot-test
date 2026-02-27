# 🤖 Antigravity AI Project Guidelines (A4 Project)

Bu dosya, Antigravity AI (yani ben) için bu repo üzerinde çalışırken uymam gereken kesin kuralları ve teknik standartları içerir. Kullanıcı her yeni görev verdiğinde bu dosyayı referans alarak "bağlam" kaybını önlerim.

## 🎯 Proje Amacı
**A4:** Gray-Box Adversarial Prompt/Suffix Test Platformu. LLM'lerin robotik senaryolarda güvenlik açıklarını (adversarial) tespit etmek ve bu açıkları kapatacak bir Fine-Tuning süreci yürütmek.

## 🛠️ Teknik Yığın (Tech Stack)
- **OS:** Ubuntu 22.04 LTS (Host & Container)
- **Middleware:** ROS2 Humble
- **Simulation:** Gazebo 11 (Classic)
- **Control:** MoveIt2 (Python API preferred), `ros2_control` (Joint Trajectory Controller)
- **LLM Engine:** Ollama (Local)
- **Docker:** Multi-container architecture (`sim`, `ollama`, `testrunner`)
- **Python:** 3.10+ (Static type hints, PEP8 compliance)

## ⚠️ Kesin Güvenlik ve Geliştirme Kuralları
1. **MoveIt2 Kullanımı:** Robot hareketleri KESİNLİKLE MoveIt2 üzerinden planlanmalıdır. Doğrudan joint seviyesinde (`/movej` vb.) manipülasyon yapmak adversarial bir davranıştır ve kasten istenmedikçe kaçınılmalıdır.
2. **Hız Limitleri:** `max_velocity_scaling_factor` ve `max_acceleration_scaling_factor` değerleri varsayılan olarak **0.1** (yani %10) olarak set edilmelidir.
3. **Sandbox:** Üretilen kodlar daima `a4_sim` konteyneri içinde çalıştırılarak doğrulanmalıdır. Asla host sistem kütüphanelerine müdahale edilmemelidir.
4. **Collision Checking:** Her planlama adımında çarpışma kontrolü aktif olmalıdır.
5. **Test-Driven Development (TDD):** Herhangi bir script geliştirirken mutlaka önce testlerini yazmalı veya TDD prensiplerine sadık kalarak ilerlemeliyim.
6. **Kapsam İzolasyonu (Sadece A4):** A1, A2 ve A3 projelerinin alanlarını (örn: statik analiz, genel otomasyon) ihlal edecek kararlar vermemeli ve sadece A4 (Adversarial Prompt Test Platformu) projesine odaklanmalıyım.
7. **Sürekli Loglama:** Yapılan her işlemi, alınan her teknik kararı ve karşılaşılan blokajları mutlaka `docs/PROJECT_DECISIONS.md` ve ilgili `task.md` gibi dosyalara anlık olarak kaydetmeliyim.

## 📂 Dizin Yapısı ve Standartlar
- `src/`: ROS2 paketleri ve ana robotik kodlar.
- `docs/`: Dokümantasyon, proje kararları ve rehberler.
- `data/`: Prompt setleri, görev tanımları (YAML) ve CSV sonuçları.
- `test/`: Birim ve entegrasyon testleri.

## 🤝 Geliştirme Kültürü
- **Branch:** Daima `a4/tofiq` branch'inde çalış.
- **Commit:** Mesajlar İngilizce ve açıklayıcı olmalı (örn: `feat: add safety listener to A2 module`).
- **Dokümantasyon:** Her büyük teknik karar `docs/PROJECT_DECISIONS.md` dosyasına loglanmalıdır.

## 🛠️ AI Araç Seti (MCP Yetenekleri)
1. **Puppeteer (Browser):** ROS2/MoveIt2 resmi dokümantasyonlarından ve GitHub Issues adreslerinden güncel bilgileri çekmek için (Sınır: Sadece teknik araştırma).
2. **FileSystem:** Host sistemin geri kalanına zarar vermeme kuralı gereği sadece `data/` klasörünü okuma ve yazma yetkisi (A4 Kapsam İzolasyonu).
3. **Sequential Thinking:** Karmaşık hataları çözerken TDD kuralına uymak adına kod yazmadan önce adım adım içsel planlama yapmak için.

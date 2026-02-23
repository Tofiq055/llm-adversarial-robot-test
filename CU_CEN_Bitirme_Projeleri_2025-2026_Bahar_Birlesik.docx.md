**Çukurova Üniversitesi**

Bilgisayar Mühendisliği Bölümü

2025–2026 Bahar Yarıyılı Bitirme Projeleri

**Bitirme Projeleri Dokümanı**

Grup A: Güvenilir Robot Yazılımı (UR5e / Gazebo / MoveIt2)

Grup B: Sürü İHA ile Orman Yangını Erken Tespiti (Dijital İkiz / Edge AI)

Danışman: Dr. Yunus Emre ÇOĞURCU

Tarih: Şubat 2026

Toplam: 20 Proje (Grup A: 10 \+ Grup B: 10\)

**Bütün öğrenciler 20 Şubat 2026 tarihine kadar projelerini seçip danışmandan onay almalıdır. 20 Şubat 2026 tarihinden sonra proje ve danışmanlık verilmeyecektir.**

# **1\. Zorunlu Kurallar ve Çalışma Standartları**

**Bu kurallar her iki gruptaki (Grup A ve Grup B) tüm projeler için geçerlidir.**

## **1.1 Haftalık İlerleme ve İletişim**

* Haftalık ilerleme toplantıları zorunludur. Her toplantı için kısa gündem ve çıktı (meeting notes) hazırlanacaktır.

* Her hafta en az 1 kez danışmana, GitHub Issues/Project Board üzerinden ilerleme güncellemesi yapılacaktır.

* Riskler ve blokajlar en geç 24 saat içinde Issue olarak açılmalı ve etiketlenmelidir (blocker, risk, question).

* Takım içi arayüz sözleşmeleri: mesaj şemaları, dosya formatları, klasör yapısı ve API sözleşmeleri haftalık toplantıda netleştirilmelidir.

## **1.2 GitHub Teslimi ve Depo Yapısı**

* Tüm projeler GitHub üzerinden teslim edilecektir. Kod, dokümantasyon, deney scriptleri ve rapor aynı depoda bulunmalıdır.

* Depo; ROS2 paket yapısına uygun olmalı (src/, launch/, config/, msg/srv/action/, test/, docs/).

* Branch stratejisi: main (stabil), dev (geliştirme). Her özellik için feature/\* branch ve Pull Request zorunludur.

* Her PR için: açıklama, ilgili Issue linki, derlenebilirlik ve minimum test çalıştırma kanıtı gereklidir.

* Sürümleme: en az 1 adet release tag (v1.0) ve final teslimde code-freeze etiketi.

## **1.3 README, Kurulum ve Demo Videoları**

* README.md zorunludur ve şu alt başlıkları içermelidir: problem tanımı, mimari şema, bağımlılıklar, kurulum, çalıştırma, parametreler, veri formatı, deneylerin yeniden üretimi, beklenen çıktı.

* Kurulum 'sıfırdan' adım adım açıklanmalı; tek komutla kurulum/çalıştırma hedeflenmelidir (ör. docker compose veya bash script).

* Projenin son halinin nasıl çalıştığını gösteren video/videolar (YouTube unlisted veya Drive link) README içinde yer almalıdır.

* Her demo videosu en az şu kısımları göstermelidir: (i) kurulum, (ii) çalıştırma, (iii) örnek senaryo çıktıları, (iv) metrik üretimi (varsa).

## **1.4 ROS2 Humble, Docker ve Mühendislik Kalitesi**

* Tüm projeler ROS2 Humble ile geliştirilecektir. Launch dosyaları, parameter YAML'ları ve uygun QoS ayarları kullanılmalıdır.

* Reprodüksiyon için Docker (multi-stage) ve mümkünse docker-compose sağlanmalıdır.

* Kod kalitesi: modüler tasarım, tekrar eden koddan kaçınma, yapılandırmaların YAML ile yönetimi, hata yakalama ve loglama.

* Test: kritik fonksiyonlar için unit/integration test hedeflenir; mümkünse GitHub Actions ile otomatik build/test.

* Tedarik zinciri güvenliği: SBOM (CycloneDX veya benzeri) üretimi ve temel zafiyet taraması (Trivy/Grype) hedeflenir.

* Lisans: depoya uygun bir açık kaynak lisansı eklenmelidir (ör. MIT/Apache-2.0). Üçüncü taraf kodlar lisanslarına uygun kullanılmalıdır.

## 

## **1.5 Güvenli Kullanım, Etik ve Sorumlu Paylaşım**

* Bu projeler araştırma kapsamındadır. Tüm 'adversarial' çalışmalar yalnızca simülasyon ortamında yürütülmelidir.

* Gerçek robot/İHA sistemlerinde izin/etik onay olmadan adversarial senaryolar uygulanmayacaktır.

* Sorumlu açıklama (responsible disclosure) yaklaşımı benimsenmelidir: bulguların paylaşımı danışman onayıyla yapılır.

* Deney verileri ve loglar için veri sözlüğü (dataset card) hazırlanmalıdır: kaynak, tarih, parametreler, format, örnekler.

* Akademik dürüstlük: intihal ve kopya kesinlikle yasaktır. Kullanılan her fikir/algoritma/şekil uygun şekilde kaynaklandırılmalıdır.

* Grup B İHA projeleri için: Saha uçuş testlerinde gerekli izinler alınmalı, güvenlik bölgesi oluşturulmalı ve acil iniş prosedürleri tanımlanmalıdır.

## **1.6 Zaman Planı (Bahar Dönemi)**

* Şubat–Mart: Temel altyapı, simülasyon ve baseline bileşenler.

* Nisan: Bileşen olgunlaştırma, deneyler, performans iyileştirme.

* Mayıs başı: Mühendislik/teknik kısımlar tamamlanmış olmalı; bitirme raporu yazımına başlanmalıdır.

* Final haftaları: Rapor tamamlanması, demo ve teslim.

# **2\. Bitirme Raporu Gereksinimleri (Zorunlu)**

* Rapor dili İngilizcedir.

* Yaklaşık 50 sayfa hedeflenir (ekler hariç).

* En az 30 akademik referans zorunludur (IEEE veya benzeri tutarlı format).

* Rapor, projeyi özgün yapan katkıları Introduction bölümünde açıkça belirtmelidir.

* Hipotezler ve nasıl test edildiği; metodoloji, deney tasarımı ve metriklerle ayrıntılı açıklanmalıdır.

* Deneylerin yeniden üretilebilirliği: veri seti tanımı, parametreler, ortam bilgisi ve çalıştırma adımları raporda bulunmalıdır.

* Önerilen ekler: sistem diyagramı, tespit örnekleri, log örnekleri, CI çıktıları, SBOM çıktısı.

## **2.1 Rapor Bölümleri (Önerilen Yapı)**

* Abstract ve Keywords

* 1\. Introduction (Problem, motivasyon, özgün katkılar, hipotezler ve hedefler)

* 2\. Literature Review (en güncel ve ilgili çalışmalar; boşluk analizi)

* 3\. Methodology (mimari, algoritmalar, threat model, deney tasarımı, metrikler)

* 4\. Results (nicel sonuçlar, tablolar, grafikler, baseline/ablation karşılaştırmaları)

* 5\. Discussion (yorum, başarısızlık modları, sınırlılıklar, genelleme)

* 6\. Conclusions and Future Work

* References

* Appendices

# **3\. Grup A – Güvenilir Robot Yazılımı (UR5e / Gazebo / MoveIt2)**

Bu projeler, güvenilir robot yazılımı geliştirme hedeflerini lisans seviyesinde yönetilebilir parçalara ayırmak üzere tasarlanmıştır. Her proje birbirinden bağımsızdır (tek öğrenci, tek repo) ve standart platform olarak ROS2 Humble \+ Gazebo simülasyonunu kullanır. Fiziki robot zorunluluğu yoktur.

## **Zorunlu Giriş Şartı – UR5e Gazebo \+ MoveIt2 Starter Kit**

*Tüm Grup A öğrencilerinin ilk 2–3 hafta içinde tamamlaması gereken ortak altyapı. Bu starter kit bitirme projesi sayılmaz; önkoşuldur.*

1. UR5e Gazebo simülasyonunu (Gazebo Classic 11\) çalıştır.

2. ros2\_control kontrolörlerini doğrula: joint\_state\_broadcaster \+ joint\_trajectory\_controller.

3. MoveIt2 konfigürasyonunu çalıştır: planlama \+ yürütme.

4. En az 1 senaryoda rosbag2 kayıt al ve replay ile tekrar üretilebilirliği göster.

5. Kurulum ve çalıştırmayı tek komutla otomatikleştir (script veya Docker).

6. README'ye: kurulum, çalıştırma, hata giderme ve demo videosu linklerini ekle.

## **Grup A – Proje Özet Tablosu**

| No | Proje Başlığı | Zorluk | Donanım |
| :---- | :---- | :---- | :---- |
| A1 | Deney Senaryo Otomasyonu: rosbag2 \+ Replay \+ Metrik Üretimi | Orta | Yazılım |
| A2 | ROS2 Güvenlik Denetçisi: Workspace \+ Hız/İvme Limitleri \+ Güvenli Durdurma | Orta | Yazılım |
| A3 | Robot Kontrol Kodunda Statik Analiz ve Güvenlik Skoru (LLM Kodları Dahil) | Orta-Zor | Yazılım |
| A4 | Gray-Box Adversarial Prompt/Suffix Test Platformu \+ Simülasyon Güvenlik Skoru | Zor | Yazılım |
| A5 | ROS2 Hata Enjeksiyonu ile Dayanıklılık: Gecikme/Dropout/TF Hataları | Orta | Yazılım |
| A6 | Jetson Orin \+ RealSense \+ Termal Kamera: Yakınlık Algılama ve ROS2 Entegrasyonu | Orta | Var |
| A7 | Endüstriyel Kalite Şablonu: Docker \+ CI \+ SBOM \+ Sürümleme (ROS2 Humble) | Kolay-Orta | Yazılım |
| A8 | Unity Robot Seçici (UR5e/KUKA KR3/ABB GoFa) \+ ROS2 Model Switcher | Orta-Zor | Yazılım |
| A9 | ROS2 Humble Kritik/Savunma Risk Analizi: SROS2, DDS-Security | Orta | Yazılım |
| A10 | Çok-Robot MoveIt2 Benchmark: UR5e/KR3/GoFa Planlama Karşılaştırması | Orta | Yazılım |

# **A1 – Deney Senaryo Otomasyonu: rosbag2 \+ Replay \+ Metrik Üretimi ELVIN DAVIDOV**

**Amaç**

UR5e senaryolarını otomatik koşturan bir deney çalıştırma altyapısı geliştirmek. Her koşu için otomatik kayıt, metrik çıkarımı ve raporlama üretmek.

| Zorluk Seviyesi | Orta |
| :---- | :---- |
| **Gerekli Donanım** | Yok (Simülasyon) |
| **Repo Adı** | ur5e-experiment-automation |
| **Teslim Edilecekler** | Senaryo koşucu \+ analiz scriptleri, 3 senaryo × 10 tekrar CSV \+ otomatik rapor, demo videoları. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | En az 3 görev senaryosu tanımla (home→pose, waypoint, pick-place). |
| **2** | Senaryo koşucu düğümü yaz: hedefler, timeout, başarı kriterleri. |
| **3** | Kayıt şablonu oluştur: joint\_states, TF, planlanan/yürütülen trajectory, olay log'u. |
| **4** | Replay \+ analiz pipeline'ı yaz: bag → metrik çıkarımı (süre, hedef hatası, yol uzunluğu, başarım). |
| **5** | YAML ile deney konfigürasyonu; tek komutla 10 tekrar koşabilsin. |
| **6** | Sonuçları CSV \+ kısa otomatik rapor olarak üret (tablo/grafik). |

# **A2 – ROS2 Güvenlik Denetçisi: Workspace \+ Hız/İvme Limitleri \+ Güvenli Durdurma ELVIN DAVIDOV**

**Amaç**

UR5e hareketlerini izleyen ve güvenlik ihlallerinde yavaşlatma/durdurma uygulayan bir ROS2 denetçi geliştirmek. Tespit ve tepki metriklerini ölçmek.

| Zorluk Seviyesi | Orta |
| :---- | :---- |
| **Gerekli Donanım** | Yok (Simülasyon) |
| **Repo Adı** | ur5e-safety-supervisor |
| **Teslim Edilecekler** | Denetçi ROS2 paketi \+ test senaryoları, CSV \+ grafiklerle metrik raporu, demo videosu. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Kuralları tanımla: joint limitleri, workspace bounding box, yasak bölgeler. |
| **2** | Denetçi düğümü geliştir: joint\_states/TF dinle, ihlal tespiti yap, olay log'u üret. |
| **3** | Tepki mantığı: soft hız ölçekleme \+ hard stop (simülasyonda). |
| **4** | MoveIt2 ile entegrasyon: planlanan trajectory pre-check \+ runtime kontrol. |
| **5** | Her kural için en az 3 ihlal ve 3 normal test senaryosu oluştur. |
| **6** | Metrikleri çıkar: tespit gecikmesi, FP/FN, stop mesafesi, recovery süresi. |
| **7** | Launch test \+ CI entegrasyonu ekle. |

# **A3 – Robot Kontrol Kodunda Statik Analiz ve Güvenlik Skoru (LLM Kodları Dahil) KAMAL ASADOV**

**Amaç**

ROS2 kontrol düğümlerindeki güvenlik anti-pattern'lerini statik analiz ile yakalamak ve skorlamak. CI içinde otomatik raporlayıp 'merge gate' olarak çalıştırmak.

| Zorluk Seviyesi | Orta-Zor |
| :---- | :---- |
| **Gerekli Donanım** | Yok (Bilgisayar) |
| **Repo Adı** | ros2-static-safety-analyzer |
| **Teslim Edilecekler** | Statik analiz aracı \+ 10+ kural \+ testler, CI raporlama, LLM çıktı değerlendirmesi. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Kural seti tanımla (timeout yok, exception yok, stop yok, QoS yanlış, vb.). |
| **2** | Python için AST tabanlı kural motoru yaz; her kural için pozitif/negatif örnek üret. |
| **3** | C++ için en az 2 kontrolü clang-tidy/cppcheck ile ekle (veya minimal checker). |
| **4** | Raporlama: SARIF/Markdown üret ve CI'de yayınla. |
| **5** | En az 30 LLM-üretimi kontrol scripti oluştur ve analizden geçir; bulguları sınıflandır. |
| **6** | Simülasyonda: analiz fail ise çalıştırmayı engelle; pass ise senaryoyu koştur. |

# **A4 – Gray-Box Adversarial Prompt/Suffix Test Platformu \+ Simülasyon Güvenlik Skoru TOFIG VALIYEV**

**Amaç**

LLM API ile UR5e görev kodu üreten ve simülasyon \+ denetçi ile puanlayan test platformu geliştirmek. Prompt/suffix varyantlarının 'unsafe' davranışa etkisini nicel ölçmek.

| Zorluk Seviyesi | Zor |
| :---- | :---- |
| **Gerekli Donanım** | Yok (Simülasyon \+ LLM API) |
| **Repo Adı** | llm-adversarial-robot-test |
| **Teslim Edilecekler** | Test runner \+ raporlama (CSV \+ Markdown), 50+ koşu deney seti, demo videosu. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | En az 3 görev tanımla (pose, waypoint, pick-place). |
| **2** | Prompt şablonları oluştur: güvenli baseline \+ riskli varyantlar \+ suffix varyantları. |
| **3** | Üretilen kodu container içinde derle/çalıştır pipeline'ı kur. |
| **4** | Safety supervisor olaylarını, ihlalleri ve timeout'ları otomatik topla. |
| **5** | Metrikler: unsafe oranı, safe başarı oranı, engelleme oranı, tespit gecikmesi. |
| **6** | 50+ koşu ile deney seti üret; sonuçları CSV \+ özet raporla sun. |

# **A5 – ROS2 Hata Enjeksiyonu ile Dayanıklılık: Gecikme/Dropout/TF Hataları**

**Amaç**

ROS2 mesajlaşma ve TF katmanında hata koşullarını simüle etmek ve dayanıklılık metrikleri üretmek. Recovery stratejilerinin etkisini ölçmek.

| Zorluk Seviyesi | Orta |
| :---- | :---- |
| **Gerekli Donanım** | Yok (Simülasyon) |
| **Repo Adı** | ros2-fault-injection-resilience |
| **Teslim Edilecekler** | Fault-injection paketleri \+ deney scriptleri, 3 hata × 3 senaryo sonuç tabloları, demo videosu. |
| **Çapraz İlişki** | Grup B Proje 9 (HIL Gecikme Analizi) ile hata modelleri ve gecikme ölçüm yöntemleri paylaşılabilir. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Hata modellerini seç: topic drop, latency, jitter, TF frame kaybı, sensor timeout. |
| **2** | Fault-injection düğümleri geliştir (relay \+ delay/drop). |
| **3** | 3 senaryoda kademeli hata uygula; başarı/timeout/safety stop ölç. |
| **4** | Recovery stratejileri uygula: safe stop \+ retry, yeniden başlatma, fallback hız. |
| **5** | Metrikleri raporla: başarı oranı, recovery süresi, stop sayısı, yanlış stop. |
| **6** | 30+ koşu ile deney raporu üret. |

# **A6 – Jetson Orin \+ RealSense \+ Termal Kamera: Yakınlık Algılama ve ROS2 Entegrasyonu**

**Amaç**

RealSense D435 ve termal USB-C kameradan ROS2 Humble ile veri almak ve basit risk/ihlal sinyali üretmek. Bu sinyali simülasyon denetçisine bağlayacak standart bir arayüz tanımlamak.

| Zorluk Seviyesi | Orta |
| :---- | :---- |
| **Gerekli Donanım** | 1x Jetson Orin Nano, 1x RealSense D435, 1x Termal USB-C Kamera |
| **Repo Adı** | jetson-proximity-hazard-ros2 |
| **Teslim Edilecekler** | Algı düğümleri \+ hazard arayüzü (msg tanımı), Jetson demo videosu, gecikme CSV. |
| **Çapraz İlişki** | Grup B Proje 2 (Termal Sürücü), Proje 3 (RealSense SLAM) ve Proje 5 (Edge AI Pipeline) ile donanım sürücüleri ve sensör füzyon yaklaşımları doğrudan paylaşılır. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | RealSense ve termal kamerayı ROS2 Humble'da çalıştır; topic'leri ve frame\_id'leri standardize et. |
| **2** | Derinlik için ROI tabanlı minimum mesafe çıkarımı; termal için eşikleme tabanlı sıcak obje tespiti uygula. |
| **3** | Hazard seviyeleri üret (safe/warn/stop) ve ROS2 topic olarak yayınla. |
| **4** | Gecikme ölçümü: kamera→hazard pipeline gecikmesini timestamp ile ölç ve raporla. |
| **5** | Simülasyonda 'dummy hazard publisher' ile denetçi entegrasyonunu test et. |
| **6** | Jetson deploy: container ve otomatik startup script hazırla. |

# **A7 – Endüstriyel Kalite Şablonu: Docker \+ CI \+ SBOM \+ Sürümleme (ROS2 Humble)**

**Amaç**

ROS2 projelerinde reprodüksiyon, test ve temel tedarik-zinciri güvenliğini otomatikleştiren bir template repo hazırlamak. UR5e Gazebo demosunu CI içinde 'green' çalıştırmak.

| Zorluk Seviyesi | Kolay-Orta |
| :---- | :---- |
| **Gerekli Donanım** | Yok (Bilgisayar) |
| **Repo Adı** | ros2-industrial-template |
| **Teslim Edilecekler** | Template repo \+ CI pipeline, SBOM ve tarama raporu, demo videosu. |
| **Çapraz İlişki** | Her iki gruptaki (A ve B) tüm projeler bu şablonu referans alarak Docker, CI ve SBOM standartlarını uygulayabilir. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Template repo oluştur: README şablonu, LICENSE, CONTRIBUTING, kod stili kuralları. |
| **2** | Multi-stage Docker ile tek komutla build/run yap; UR5e Gazebo demo çalışsın. |
| **3** | GitHub Actions: lint (ament), build, unit test, launch test ekle. |
| **4** | SBOM üret (CycloneDX vb.) \+ zafiyet taraması (Trivy/Grype) ekle. |
| **5** | Release akışı: tag, changelog, docker image digest/artifact yayınlama. |

# **A8 – Unity Robot Seçici (UR5e/KUKA KR3/ABB GoFa) \+ ROS2 Model Switcher**

**Amaç**

Unity arayüzünde robot seçimi ile ROS2 tarafında seçilen robotun Gazebo ve RViz'de dinamik değiştirilmesi. MoveIt2 ile planlama ve simülasyonda hareketin yürütülmesi.

| Zorluk Seviyesi | Orta-Zor |
| :---- | :---- |
| **Gerekli Donanım** | Yok (Bilgisayar \+ Unity) |
| **Repo Adı** | unity-ros2-robot-switcher |
| **Teslim Edilecekler** | Unity projesi \+ ROS2 robot\_switcher paketi, MoveIt2 demo videoları, URDF/MoveIt2 config. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Unity'de 3 butonlu UI tasarla; seçimi ROS2'ye TCP üzerinden service/message olarak gönder. |
| **2** | ROS2'de 'robot\_switcher' düğümü yaz: mevcut modeli kaldır, seçilen URDF'yi spawn et, robot\_description güncelle. |
| **3** | RViz için robot\_state\_publisher'ı dinamik güncelleme stratejisi uygula. |
| **4** | Her robot için MoveIt2 config oluştur (UR5e hazır paket; KR3/GoFa için konfig üretimi). |
| **5** | Planla-yürüt döngüsü: seçili robot için hedef pose gönder ve planı yürüt; 3 örnek senaryo. |
| **6** | Robotlar arası geçişte TF frame tutarlılığını test et; otomatik test senaryosu ekle. |
| **7** | Demo: Unity açık → robot seç → Gazebo+RViz değişsin → MoveIt2 hareket etsin. |

# **A9 – ROS2 Humble Kritik/Savunma Risk Analizi: SROS2, DDS-Security \- ÇAĞRI DEMİR**

**Amaç**

ROS2 Humble tabanlı bir sistemin kritik/savunma ortamlarda taşıdığı riskleri analiz etmek. SROS2/DDS-Security yeteneklerini değerlendirip sertleştirme önerileri çıkarmak.

| Zorluk Seviyesi | Orta |
| :---- | :---- |
| **Gerekli Donanım** | Yok (Bilgisayar) |
| **Repo Adı** | ros2-security-risk-analysis |
| **Teslim Edilecekler** | Tehdit modeli \+ risk matrisi, SROS2 demo \+ performans kıyası, SBOM \+ yol haritası raporu. |
| **Çapraz İlişki** | Grup B projeleri (özellikle ROS2 tabanlı İHA haberleşmesi) için de güvenlik önerileri üretebilir. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Tehdit modeli oluştur: varlıklar, saldırı yüzeyi, güven varsayımları, risk matrisi. |
| **2** | ROS2 güvenlik mimarisini incele: SROS2 keystore, enclave, izin politikaları, DDS-Security. |
| **3** | Deney kurulumu: 2–3 düğümlü demo üzerinde güvenlik açık/kapalı kıyaslaması (latency \+ erişim). |
| **4** | SBOM \+ bağımlılık analizi: ROS2 workspace için SBOM üret ve riskleri sınıflandır. |
| **5** | Operasyonel öneriler: ağ segmentasyonu, anahtar yönetimi, logging/monitoring. |
| **6** | Çıktı: 'Minimum Güvenlik Baseline' dokümanı \+ geliştirme yol haritası. |

# **A10 – Çok-Robot MoveIt2 Benchmark: UR5e/KR3/GoFa Planlama Karşılaştırması**

**Amaç**

Aynı görev setini farklı robot modellerinde koşarak planlama başarımını karşılaştırmak. Planlama süresi, başarı oranı, yol uzunluğu, jerk/smoothness metrikleri.

| Zorluk Seviyesi | Orta |
| :---- | :---- |
| **Gerekli Donanım** | Yok (Simülasyon) |
| **Repo Adı** | moveit2-multi-robot-benchmark |
| **Teslim Edilecekler** | Benchmark çalıştırıcısı \+ analiz scriptleri, 3 robot × görev seti CSV \+ grafik, demo videosu. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | 3 robot için ortak görev seti tanımla: 5 hedef pose \+ 2 waypoint yolu. |
| **2** | Her robot için MoveIt2 planner ayarlarını standardize et. |
| **3** | Her hedefi 20 tekrar planla; planlama süresi ve başarıyı kaydet. |
| **4** | Yol metrikleri üret: yol uzunluğu, eklem limitlerine yakınlık, max hız/ivme. |
| **5** | Sonuçları CSV \+ grafiklerle raporla; bulguları tartış. |
| **6** | Reprodüksiyon için konfigürasyonları ve seed yönetimini dokümante et. |

# **4\. Grup B – Sürü İHA ile Orman Yangını Erken Tespiti ve Dijital İkiz**

Bu projeler, "Termal Dijital İkiz Tabanlı Sürü İHA Sistemi ile Orman Yangınlarının Erken Tespiti ve Önlenmesi" kapsamında tasarlanmıştır. Mevcut donanım: 2x Jetson Orin Nano Super, 2x Intel RealSense D435, 2x Termal USB-C kamera. Drone'lar henüz temin edilemediğinden projeler drone gelmeden tamamlanabilecek şekilde planlanmıştır.

## **Grup B – Donanım Dağılımı**

| Donanım | Kullanan Projeler |
| :---- | :---- |
| **Jetson Orin Nano \#1** | B2, B4, B5, B6, B7, B9, B10 |
| **Jetson Orin Nano \#2** | B3, B6 |
| **Intel RealSense D435 \#1** | B3, B5, B9 \+ (A6 paylaşım) |
| **Intel RealSense D435 \#2** | Yedek / Paralel test |
| **Termal USB-C Kamera \#1** | B2, B5, B9 \+ (A6 paylaşım) |
| **Termal USB-C Kamera \#2** | Yedek / Paralel test |
| **Tablet/Telefon (WiFi)** | B7, B10 |

## **Grup B – Proje Özet Tablosu**

| No | Proje Başlığı | Zorluk | Donanım | Revize? |
| :---- | :---- | :---- | :---- | :---- |
| B1 | Gazebo \+ PX4 SITL Tabanlı Sürü İHA Simülasyon Ortamı | Orta | Yazılım | Hayır |
| B2 | ROS2 Tabanlı Termal Kamera Sürücüsü ve Veri Toplama Düğümü | Kolay-Orta | Var | Hayır |
| B3 | Intel RealSense D435 ile Görsel SLAM ve GPS-Denied Konum Tahmini | Orta | Var | **Evet** |
| B4 | Termal/RGB Yangın Tespiti: YOLOv7-Tiny \+ Video İyileştirme Pipeline | Orta-Zor | Var | **Evet** |
| B5 | Jetson Orin Nano Edge AI Çıkarım Pipeline'ı ve Performans Analizi | Orta | Var | Hayır |
| B6 | Merkeziyetsiz Federatif Öğrenme (DFL) ve Kooperatif Lokalizasyon | Zor | Var | **Evet** |
| B7 | Uydu LST Tabanlı 3B Termal Dijital İkiz \+ Mobil Arayüz | Orta-Zor | Var | **Evet** |
| B8 | Sürü İHA Dağıtık Kontrol, GPS-Denied Navigasyon ve Formasyon | Orta-Zor | Yazılım | **Evet** |
| B9 | Hardware-in-the-Loop (HIL) Test Altyapısı ve Gecikme Analizi | Orta | Var | Hayır |
| B10 | Mobil Komuta-Kontrol Arayüzü ve Sürü İHA Veri Yönetimi | Kolay-Orta | Var | **Evet** |

# **B1 – Gazebo \+ PX4 SITL Tabanlı Sürü İHA Simülasyon Ortamı**

**Proje Özeti \- İBRAHIM CAN DINÇER**

Sürü İHA sisteminin temelini oluşturacak Gazebo \+ PX4 SITL simülasyon ortamı. GPS-denied senaryoları da simüle edebilecek.

| Zorluk Seviyesi | Orta |
| :---- | :---- |
| **Gerekli Donanım** | Yok (Yalnızca Bilgisayar) |
| **Repo Adı** | swarm-uav-simulation-env |
| **Teslim Edilecekler** | ROS2 workspace, Gazebo world dosyaları, PX4 SITL konfigürasyonları, GPS-denied senaryoları, launch dosyaları. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Ubuntu 22.04 üzerine ROS2 Humble kurulumu ve temel ROS2 kavramlarının öğrenilmesi. |
| **2** | PX4 Autopilot kaynak kodunun klonlanması ve SITL modunun derlenmesi. |
| **3** | Gazebo simülatörünün kurulumu ve PX4 ile entegrasyonu. |
| **4** | Tek İHA için SITL simülasyonu, MAVLink komutları (kalkış, iniş, waypoint takibi). |
| **5** | Çoklu İHA desteği için PX4 SITL yapılandırması (3 sanal İHA). |
| **6** | swarm\_manager\_node yazılması: Tüm İHA durumlarını toplayan ve formasyon komutları gönderen node. |
| **7** | Yoğun ağaç örtüsüne sahip orman haritası modeli oluşturulması (Gazebo world). |
| **8** | Rüzgar, GPS gürültüsü ve GPS-denied senaryolarının parametrik eklenmesi. |
| **9** | README.md, kurulum kılavuzu ve launch dosyalarının hazırlanması. |
| **10** | 3 İHA üçgen formasyon uçuşu \+ GPS-denied geçiş senaryosu videosunun kaydedilmesi. |

# **B2 – ROS2 Tabanlı Termal Kamera Sürücüsü ve Veri Toplama Düğümü**

**Proje Özeti \- ERDEM KANDILCI**

Termal USB-C kameraların ROS2 entegrasyonu, termal görüntü akışı ve sıcaklık haritası yayını.

| Zorluk Seviyesi | Kolay-Orta |
| :---- | :---- |
| **Gerekli Donanım** | 1x Termal USB-C Kamera, 1x Jetson Orin Nano |
| **Repo Adı** | thermal-camera-ros2-driver |
| **Teslim Edilecekler** | ROS2 termal kamera sürücü paketi, kalibrasyon scripti, rosbag dosyaları, RViz konfigürasyonu. |
| **Çapraz İlişki** | Grup A Proje A6 (Jetson Yakınlık Algılama) ile termal kamera sürücüsü doğrudan paylaşılır. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Jetson Orin Nano üzerine JetPack SDK ve ROS2 Humble kurulumu. |
| **2** | Termal USB-C kameranın Jetson üzerinde tanınması, V4L2 ile ham görüntü akışı. |
| **3** | thermal\_camera\_driver\_node: sensor\_msgs/Image formatında /thermal/image\_raw topic'i. |
| **4** | Sıcaklık dönüşümü kalibrasyon parametreleri (piksel → °C). |
| **5** | /thermal/temperature\_map topic'ine heatmap yayını. |
| **6** | ROS2 bag ile veri kaydı, timestamp senkronizasyonu. |
| **7** | rqt/RViz2 görselleştirmesi. |
| **8** | Farklı mesafe/açılarda kalibrasyon testleri ve doğruluk raporu. |
| **9** | YAML parametre dosyası (çözünürlük, FPS, sıcaklık eşikleri). |
| **10** | README.md ve kullanım kılavuzu. |

# **B3 – Intel RealSense D435 ile Görsel SLAM ve GPS-Denied Konum Tahmini**

**Proje Özeti**

GPS-denied ortamda Visual SLAM (RTAB-Map/ORB-SLAM3) \+ ağaç gövdesi eşleme ile konum tahmini.

| Zorluk Seviyesi | Orta |
| :---- | :---- |
| **Gerekli Donanım** | 1x Intel RealSense D435, 1x Jetson Orin Nano |
| **Repo Adı** | realsense-visual-slam-gps-denied |
| **Teslim Edilecekler** | ROS2 Visual SLAM paketi, GPS-denied konum düğümü, öznitelik eşleme modülü, SLAM test raporu. |
| **Durum** | **REVİZE EDİLDİ** |
| **Çapraz İlişki** | Grup A Proje A6 (RealSense Yakınlık) ile kamera sürücüsü paylaşılır. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Jetson üzerine RealSense SDK, realsense-ros ve RTAB-Map kurulumu. |
| **2** | RGB-D akışının ROS2 topic olarak yayınlanması. |
| **3** | RTAB-Map veya ORB-SLAM3 ile gerçek zamanlı 3B harita \+ konum tahmini (Visual SLAM). |
| **4** | GPS-denied senaryo: GPS kapatılarak yalnızca Visual SLAM ile konum doğruluğu testi. |
| **5** | Görsel öznitelik çıkarma (SIFT/ORB) ve homografi ile ardışık kare konum tahmini. |
| **6** | Ağaç gövdesi/doğal landmark eşleme: Derinlik verisinden silindirik obje segmentasyonu. |
| **7** | slam\_localizer\_node: GPS varsa GPS, kesilince Visual SLAM'e geçen hibrit düğüm. |
| **8** | İç/dış mekan SLAM doğruluk testleri, RMSE raporlaması. |
| **9** | Collaborative SLAM kavramsal tasarımı (iki kamera harita paylaşımı). |
| **10** | README.md, SLAM performans raporu, GPS-denied test dokümantasyonu. |

# **B4 – Termal/RGB Yangın Tespiti: YOLOv7-Tiny \+ Video İyileştirme Pipeline**

**Proje Özeti \- DUDU FEYZA KAVUN**

YOLOv7-Tiny \+ video iyileştirme \+ ufuk tespiti ile termal/RGB yangın tespiti pipeline'ı.

| Zorluk Seviyesi | Orta-Zor |
| :---- | :---- |
| **Gerekli Donanım** | 1x Jetson Orin Nano (eğitim: bilgisayar GPU) |
| **Repo Adı** | thermal-fire-detection-yolo |
| **Teslim Edilecekler** | Eğitim scripti, video iyileştirme \+ ufuk tespiti modülleri, ONNX/TensorRT modeller, ROS2 pipeline, performans raporu. |
| **Durum** | **REVİZE EDİLDİ** |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Termal/RGB yangın veri setleri araştırması (FLAME, D-Fire, MODIS Active Fire vb.). |
| **2** | YOLO formatında etiketleme (Roboflow/CVAT): yangın, duman, normal sınıfları. |
| **3** | Video iyileştirme modülü: CLAHE, histogram eşitleme, termal normalizasyon. |
| **4** | Ufuk çizgisi tespiti: Gökyüzü maskeleme (Canny \+ Hough veya basit CNN). |
| **5** | YOLOv7-Tiny eğitimi: mAP@0.5, precision, recall, F1 raporlaması. |
| **6** | YOLOv8-nano ile karşılaştırma (hız vs. doğruluk analizi). |
| **7** | ONNX dönüşümü \+ TensorRT optimizasyonu (Jetson). |
| **8** | fire\_detector\_node: Video iyileştirme → Ufuk maskeleme → YOLO çıkarım → Sonuç yayınlama. |
| **9** | Farklı senaryolarda test (gündüz/gece/duman), confidence threshold optimizasyonu. |
| **10** | Model kartı, YOLOv7 vs YOLOv8 karşılaştırma tablosu, performans raporu. |

# **B5 – Jetson Orin Nano Edge AI Çıkarım Pipeline'ı ve Performans Analizi**

**Proje Özeti \- GİZEM EZER**

Jetson üzerinde çoklu sensör füzyonu (termal \+ derinlik), kaynak kullanımı optimizasyonu.

| Zorluk Seviyesi | Orta |
| :---- | :---- |
| **Gerekli Donanım** | 1x Jetson Orin Nano, 1x Termal USB-C, 1x RealSense D435 |
| **Repo Adı** | jetson-edge-ai-pipeline |
| **Teslim Edilecekler** | ROS2 sensör füzyon paketi, Docker Compose, benchmark scripti, performans raporu. |
| **Çapraz İlişki** | Grup A Proje A6 ile aynı donanımı kullanır; sensör sürücüleri paylaşılabilir. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Jetson üzerinde Docker/NVIDIA L4T ML container kurulumu. |
| **2** | Termal \+ RealSense eşzamanlı veri toplama. |
| **3** | sensor\_fusion\_node: message\_filters ile zaman senkronize birleştirme. |
| **4** | Termal anomali \+ derinlik verisinden konum tahmini karar mantığı. |
| **5** | tegrastats/jtop ile CPU/GPU/bellek/güç modu izleme. |
| **6** | Farklı güç modlarında (7W, 15W, 25W) çıkarım hızı karşılaştırması. |
| **7** | ROS2 diagnostics ile sistem sağlık bilgisi yayını. |
| **8** | Docker Compose ile tek komut başlatma. |
| **9** | Performans benchmark raporu (tablo \+ grafik). |
| **10** | README.md ve kurulum kılavuzu. |

# **B6 – Merkeziyetsiz Federatif Öğrenme (DFL) ve Kooperatif Lokalizasyon**

**Proje Özeti**

DFL-UN merkeziyetsiz FL \+ FedLoc kooperatif lokalizasyon. Merkezi sunucu bağımlılığı yok.

| Zorluk Seviyesi | Zor |
| :---- | :---- |
| **Gerekli Donanım** | 2x Jetson Orin Nano |
| **Repo Adı** | decentralized-federated-learning-jetson |
| **Teslim Edilecekler** | DFL peer-to-peer kodları, FedLoc modülü, gossip aggregation, 3-senaryo performans raporu. |
| **Durum** | **REVİZE EDİLDİ** |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | DFL kavramları: DFL-UN, gossip protokolleri, P2P model paylaşımı. Merkezi FL vs DFL analizi. |
| **2** | Flower framework kurulumu (Jetson). Alternatif: NVIDIA FLARE veya custom DFL. |
| **3** | YOLOv7-Tiny (Proje B4) kullanarak FL görevi: Jetson'larda yerel fine-tuning. |
| **4** | İki Jetson WiFi Direct/ad-hoc bağlantısı, P2P testleri. |
| **5** | DFL istemci: Merkezi sunucu olmadan peer'a güncelleme gönderen gossip aggregation. |
| **6** | FedLoc: Visual SLAM (B3) verisiyle kooperatif konum tahmini. GPS-denied birden fazla cihaz. |
| **7** | Non-IID veri senaryosu: Farklı arazi/sıcaklık verisiyle DFL vs merkezi FL karşılaştırması. |
| **8** | Round bazlı doğruluk, iletişim maliyeti, yakınsama hızı grafikleri. |
| **9** | 3 senaryo: (a) Merkezi eğitim, (b) FedAvg, (c) DFL-UN — accuracy, latency, bandwidth. |
| **10** | README.md, DFL mimari diyagramı, FedLoc akış şeması, sonuç raporu. |

# **B7 – Uydu LST Tabanlı 3B Termal Dijital İkiz \+ Mobil Arayüz**

**Proje Özeti \- NACIYE BEYZA HODOĞLUGIL**

GEE/MODIS/Landsat LST verileri ile Doğu Akdeniz termal dijital ikiz. Adana/Kozan alt kümesinde Jetson+YOLOv7-Tiny ile yangın riski tahmini. Mobil WiFi arayüz.

| Zorluk Seviyesi | Orta-Zor |
| :---- | :---- |
| **Gerekli Donanım** | 1x Jetson Orin Nano \+ Tablet/Telefon (WiFi) |
| **Repo Adı** | satellite-thermal-digital-twin |
| **Teslim Edilecekler** | GEE scriptleri, LST pipeline, 3B dijital ikiz web uygulaması, yangın riski modeli, Flask/FastAPI sunucu, mobil arayüz. |
| **Durum** | **REVİZE EDİLDİ** |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Google Earth Engine (GEE) Python API kurulumu, temel sorgular. |
| **2** | MODIS MOD11A1 LST: Doğu Akdeniz (Adana/Mersin/Hatay) son 5 yıl sorgusu. |
| **3** | Landsat Collection 2 Surface Temperature (30m): Aynı bölge yüksek çözünürlüklü LST. |
| **4** | GEE'den offline indirme: GeoTIFF batch export, bölge kırpma (clip to ROI). |
| **5** | Python (rasterio, xarray) ile ön işleme: Bulut maskeleme, DN→°C, zaman serisi, interpolasyon. |
| **6** | CesiumJS/Three.js ile 3B arazi (DEM) \+ LST ısı haritası. Timeline ile tarih geçişi. |
| **7** | Adana/Kozan alt kümesi: Landsat LST \+ MODIS \+ FIRMS yangın kayıtları ile etiketli veri seti. |
| **8** | Jetson üzerinde YOLOv7-Tiny ile yangın riski tahmini. FL ile model güncelleme (B6 entegrasyonu). |
| **9** | Jetson Flask/FastAPI sunucu: WiFi ile tablet/telefon erişimi. Termal harita \+ tahmin arayüzü. |
| **10** | README.md, GEE scriptleri, dijital ikiz ekran görüntüleri, mobil kılavuz. |

# **B8 – Sürü İHA Dağıtık Kontrol, GPS-Denied Navigasyon ve Formasyon**

**Proje Özeti**

Dağıtık kontrol \+ GPS-denied collaborative SLAM navigasyon \+ formasyon algoritmaları.

| Zorluk Seviyesi | Orta-Zor |
| :---- | :---- |
| **Gerekli Donanım** | Yok (Simülasyon – B1 ve B3 çıktılarını kullanır) |
| **Repo Adı** | swarm-formation-gps-denied |
| **Teslim Edilecekler** | ROS2 formasyon kontrol, GPS-denied navigasyon düğümü, collaborative SLAM prototipi, simülasyon dosyaları. |
| **Durum** | **REVİZE EDİLDİ** |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Sürü robotik \+ GPS-denied navigasyon literatürü: Reynolds kuralları, collaborative SLAM. |
| **2** | B1 simülasyonunda yoğun ağaçlık senaryosu, 5 İHA konfigürasyonu. |
| **3** | Formasyon kontrol (Python/C++): Üçgen, çizgi, daire formasyonları. |
| **4** | formation\_control\_node: Komşu İHA'larla haberleşen dağıtık kontrol düğümü. |
| **5** | GPS-denied navigasyon: GPS kesilince Visual Odometry/SLAM moduna geçen hibrit düğüm. |
| **6** | Collaborative SLAM: Komşu İHA'ların feature map paylaşımı ile konum iyileştirme. |
| **7** | Engel kaçınma (ağaç gövdeleri, arazi engelleri). |
| **8** | İHA arızası \+ GPS kaybı birleşik senaryo testi. |
| **9** | Metrikler: Formasyon hatası, GPS-denied RMSE, collaborative SLAM iyileşme oranı. |
| **10** | README.md, GPS-denied senaryo açıklamaları, simülasyon videoları. |

# **B9 – Hardware-in-the-Loop (HIL) Test Altyapısı ve Gecikme Analizi**

**Proje Özeti**

Gerçek sensörlerin simülasyona entegrasyonu, gecikme/paket kaybı ölçümü ve analizi.

| Zorluk Seviyesi | Orta |
| :---- | :---- |
| **Gerekli Donanım** | 1x Jetson Orin Nano, 1x RealSense D435, 1x Termal USB-C |
| **Repo Adı** | hil-test-latency-analysis |
| **Teslim Edilecekler** | ROS2 HIL bridge \+ gecikme izleme paketleri, istatistiksel analiz scripti, karşılaştırma raporu. |
| **Çapraz İlişki** | Grup A Proje A5 (Hata Enjeksiyonu) ile gecikme modelleri ve fault-injection yöntemleri paylaşılabilir. |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | HIL mimarisi araştırması ve tasarımı. |
| **2** | Gazebo (B1) \+ Jetson gerçek sensörlerin aynı ROS2 ağında çalıştırılması. |
| **3** | hil\_bridge\_node: Simülasyon komutları \+ gerçek sensör verisi köprüsü. |
| **4** | latency\_monitor\_node: Gönderim/alım zaman damgası karşılaştırma. |
| **5** | ROS2 QoS ayarlarının (reliable vs best effort) gecikme etkisi testi. |
| **6** | Sensör bazında gecikme profili: Termal, RealSense, topic iletim. |
| **7** | Yapay paket kaybı senaryoları (tc/netem) ve dayanıklılık testi. |
| **8** | İstatistiksel analiz: Ortalama, medyan, std, %95 ve %99 yüzdelik dilimler. |
| **9** | HIL vs SITL karşılaştırmalı analiz raporu. |
| **10** | README.md ve HIL kurulum/test kılavuzu. |

# **B10 – Mobil Komuta-Kontrol Arayüzü ve Sürü İHA Veri Yönetimi**

**Proje Özeti**

Jetson üzerinde çalışan komuta-kontrol arayüzü. Mobil cihazdan WiFi ile termal görüntü, alarm ve İHA durumu izleme.

| Zorluk Seviyesi | Kolay-Orta |
| :---- | :---- |
| **Gerekli Donanım** | 1x Jetson Orin Nano \+ Tablet/Telefon (WiFi) |
| **Repo Adı** | mobile-command-control-app |
| **Teslim Edilecekler** | React.js web uygulaması, Flask/FastAPI backend, WebSocket/REST API, MJPEG stream, WiFi hotspot konfigürasyonu. |
| **Durum** | **REVİZE EDİLDİ** |

**Adım Adım Yapılacaklar**

| Adım | Açıklama |
| :---- | :---- |
| **1** | Gereksinim analizi: Son kullanıcı profili (orman muhafaza, itfaiye), veri/komut ihtiyaçları. |
| **2** | React.js responsive web uygulaması iskeleti (masaüstü \+ mobil uyumlu). |
| **3** | Jetson Flask/FastAPI backend: ROS2 topic → REST API \+ WebSocket. WiFi hotspot erişim. |
| **4** | İHA durumları paneli: Konum, batarya, hız, GPS/SLAM modu, bağlantı durumu kartları. |
| **5** | 2B harita (Leaflet.js): İHA konumları, uçuş rotaları, yangın riski ısı haritası. |
| **6** | Termal canlı izleme: MJPEG stream veya WebRTC ile gerçek zamanlı termal görüntü. |
| **7** | Yangın alarm paneli: YOLOv7-Tiny tespit sonuçları, konum \+ güven skoru \+ alarm geçmişi. |
| **8** | Görev yönetimi: Alan tarama / noktaya gitme komutu. Dijital ikiz (B7) veri entegrasyonu. |
| **9** | Veri dışa aktarma (CSV/JSON), offline önbellekleme. |
| **10** | README.md, mobil ekran görüntüleri, WiFi hotspot kurulum kılavuzu. |

# **5\. Gruplar Arası Çapraz İlişkiler ve Donanım Paylaşımı**

Aşağıdaki tablo, Grup A ve Grup B projeleri arasındaki ortak noktaları ve paylaşılabilecek bileşenleri göstermektedir.

| Grup A Projesi | Grup B Projesi | Paylaşılan Bileşen / İlişki |
| :---- | :---- | :---- |
| **A6 (Jetson Yakınlık)** | **B2, B3, B5** | RealSense \+ Termal kamera ROS2 sürücüleri ve Jetson deploy yöntemleri doğrudan paylaşılır. |
| **A5 (Hata Enjeksiyonu)** | **B9 (HIL Gecikme)** | ROS2 gecikme/paket kaybı modelleri ve fault-injection düğümleri ortak kullanılabilir. |
| **A7 (CI/Docker Şablon)** | **Tüm B Projeleri** | Docker, CI, SBOM ve sürümleme şablonu her iki grubun tüm projeleri için referans alınabilir. |
| **A9 (ROS2 Güvenlik)** | **B6, B8** | SROS2/DDS-Security önerileri, İHA sürü haberleşmesi için de uygulanabilir. |
| **A2 (Güvenlik Denetçisi)** | **B8 (Sürü Kontrol)** | Workspace/hız limitleri mantığı, İHA güvenli uçuş bölgesi kontrolüne adapte edilebilir. |

# **6\. Genel Notlar ve Öneriler**

Versiyon Kontrol: Her öğrenci kendi GitHub reposunda çalışacak, haftalık commit zorunluluğu olacaktır. Ana entegrasyon için ayrı bir integration reposu oluşturularak alt modüller (git submodule) olarak birleştirilecektir.

Haftalık Toplantılar: Her hafta 30 dakikalık ilerleme toplantısı yapılarak projeler arası koordinasyon sağlanacaktır. Öğrenciler kısa bir ilerleme raporu (weekly standup notu) hazırlayacaktır.

Dokümantasyon: Her repoda README.md, kurulum kılavuzu ve API dokümantasyonu bulunmalıdır. **Kullanılan dil ve kod içi yorumlar İngilizce olmalı ve tutarlılık sağlanmalıdır.**

Uydu Verisi (Grup B – B7): Google Earth Engine akademik hesap gereklidir. MODIS ve Landsat verileri ücretsiz erişilebilirdir.

Mobil Test (Grup B – B7, B10): Herhangi bir Android/iOS tablet veya telefon yeterlidir. Jetson WiFi hotspot olarak yapılandırılarak cihazlar doğrudan bağlanabilir.

Drone Geldiğinde (Grup B): Tüm yazılım modülleri hazır olduğundan, drone temin edildiğinde yalnızca fiziksel entegrasyon ve saha uçuş testleri yapılacaktır.

Grup A Öğrencileri İçin: Tüm projeler için UR5e Gazebo \+ MoveIt2 Starter Kit ilk 2–3 haftada tamamlanmalıdır.

Çapraz İşbirliği: Grup A ve Grup B öğrencileri arasında donanım paylaşımı (Jetson, RealSense, Termal kamera) ve ortak ROS2 bileşenleri konusunda iletişim kurulması teşvik edilir.
Ana gorevimiz A4 – Gray-Box Adversarial Prompt/Suffix Test Platformu + Simülasyon Güvenlik Skoru TOFIG VALIYE
gorevi uygularken diger proje alanlarini ihlal etmemeliyiz ve gorevle ilgili kurallara ve surece uymaliyiz!

Çukurova Üniversitesi
Bilgisayar Mühendisliği Bölümü
2025–2026 Bahar Yarıyılı Bitirme Projeleri

Bitirme Projeleri Dokümanı
Grup A: Güvenilir Robot Yazılımı (UR5e / Gazebo / MoveIt2)
Grup B: Sürü İHA ile Orman Yangını Erken Tespiti (Dijital İkiz / Edge AI)


Danışman: Dr. Yunus Emre ÇOĞURCU
Tarih: Şubat 2026
Toplam: 20 Proje (Grup A: 10 + Grup B: 10)




Bütün öğrenciler 20 Şubat 2026 tarihine kadar projelerini seçip danışmandan onay almalıdır. 20 Şubat 2026 tarihinden sonra proje ve danışmanlık verilmeyecektir.

1. Zorunlu Kurallar ve Çalışma Standartları
Bu kurallar her iki gruptaki (Grup A ve Grup B) tüm projeler için geçerlidir.
1.1 Haftalık İlerleme ve İletişim
Haftalık ilerleme toplantıları zorunludur. Her toplantı için kısa gündem ve çıktı (meeting notes) hazırlanacaktır.
Her hafta en az 1 kez danışmana, GitHub Issues/Project Board üzerinden ilerleme güncellemesi yapılacaktır.
Riskler ve blokajlar en geç 24 saat içinde Issue olarak açılmalı ve etiketlenmelidir (blocker, risk, question).
Takım içi arayüz sözleşmeleri: mesaj şemaları, dosya formatları, klasör yapısı ve API sözleşmeleri haftalık toplantıda netleştirilmelidir.
1.2 GitHub Teslimi ve Depo Yapısı
Tüm projeler GitHub üzerinden teslim edilecektir. Kod, dokümantasyon, deney scriptleri ve rapor aynı depoda bulunmalıdır.
Depo; ROS2 paket yapısına uygun olmalı (src/, launch/, config/, msg/srv/action/, test/, docs/).
Branch stratejisi: main (stabil), dev (geliştirme). Her özellik için feature/* branch ve Pull Request zorunludur.
Her PR için: açıklama, ilgili Issue linki, derlenebilirlik ve minimum test çalıştırma kanıtı gereklidir.
Sürümleme: en az 1 adet release tag (v1.0) ve final teslimde code-freeze etiketi.
1.3 README, Kurulum ve Demo Videoları
README.md zorunludur ve şu alt başlıkları içermelidir: problem tanımı, mimari şema, bağımlılıklar, kurulum, çalıştırma, parametreler, veri formatı, deneylerin yeniden üretimi, beklenen çıktı.
Kurulum 'sıfırdan' adım adım açıklanmalı; tek komutla kurulum/çalıştırma hedeflenmelidir (ör. docker compose veya bash script).
Projenin son halinin nasıl çalıştığını gösteren video/videolar (YouTube unlisted veya Drive link) README içinde yer almalıdır.
Her demo videosu en az şu kısımları göstermelidir: (i) kurulum, (ii) çalıştırma, (iii) örnek senaryo çıktıları, (iv) metrik üretimi (varsa).
1.4 ROS2 Humble, Docker ve Mühendislik Kalitesi
Tüm projeler ROS2 Humble ile geliştirilecektir. Launch dosyaları, parameter YAML'ları ve uygun QoS ayarları kullanılmalıdır.
Reprodüksiyon için Docker (multi-stage) ve mümkünse docker-compose sağlanmalıdır.
Kod kalitesi: modüler tasarım, tekrar eden koddan kaçınma, yapılandırmaların YAML ile yönetimi, hata yakalama ve loglama.
Test: kritik fonksiyonlar için unit/integration test hedeflenir; mümkünse GitHub Actions ile otomatik build/test.
Tedarik zinciri güvenliği: SBOM (CycloneDX veya benzeri) üretimi ve temel zafiyet taraması (Trivy/Grype) hedeflenir.
Lisans: depoya uygun bir açık kaynak lisansı eklenmelidir (ör. MIT/Apache-2.0). Üçüncü taraf kodlar lisanslarına uygun kullanılmalıdır.



1.5 Güvenli Kullanım, Etik ve Sorumlu Paylaşım
Bu projeler araştırma kapsamındadır. Tüm 'adversarial' çalışmalar yalnızca simülasyon ortamında yürütülmelidir.
Gerçek robot/İHA sistemlerinde izin/etik onay olmadan adversarial senaryolar uygulanmayacaktır.
Sorumlu açıklama (responsible disclosure) yaklaşımı benimsenmelidir: bulguların paylaşımı danışman onayıyla yapılır.
Deney verileri ve loglar için veri sözlüğü (dataset card) hazırlanmalıdır: kaynak, tarih, parametreler, format, örnekler.
Akademik dürüstlük: intihal ve kopya kesinlikle yasaktır. Kullanılan her fikir/algoritma/şekil uygun şekilde kaynaklandırılmalıdır.
Grup B İHA projeleri için: Saha uçuş testlerinde gerekli izinler alınmalı, güvenlik bölgesi oluşturulmalı ve acil iniş prosedürleri tanımlanmalıdır.
1.6 Zaman Planı (Bahar Dönemi)
Şubat–Mart: Temel altyapı, simülasyon ve baseline bileşenler.
Nisan: Bileşen olgunlaştırma, deneyler, performans iyileştirme.
Mayıs başı: Mühendislik/teknik kısımlar tamamlanmış olmalı; bitirme raporu yazımına başlanmalıdır.
Final haftaları: Rapor tamamlanması, demo ve teslim.

2. Bitirme Raporu Gereksinimleri (Zorunlu)
Rapor dili İngilizcedir.
Yaklaşık 50 sayfa hedeflenir (ekler hariç).
En az 30 akademik referans zorunludur (IEEE veya benzeri tutarlı format).
Rapor, projeyi özgün yapan katkıları Introduction bölümünde açıkça belirtmelidir.
Hipotezler ve nasıl test edildiği; metodoloji, deney tasarımı ve metriklerle ayrıntılı açıklanmalıdır.
Deneylerin yeniden üretilebilirliği: veri seti tanımı, parametreler, ortam bilgisi ve çalıştırma adımları raporda bulunmalıdır.
Önerilen ekler: sistem diyagramı, tespit örnekleri, log örnekleri, CI çıktıları, SBOM çıktısı.
2.1 Rapor Bölümleri (Önerilen Yapı)
Abstract ve Keywords
1. Introduction (Problem, motivasyon, özgün katkılar, hipotezler ve hedefler)
2. Literature Review (en güncel ve ilgili çalışmalar; boşluk analizi)
3. Methodology (mimari, algoritmalar, threat model, deney tasarımı, metrikler)
4. Results (nicel sonuçlar, tablolar, grafikler, baseline/ablation karşılaştırmaları)
5. Discussion (yorum, başarısızlık modları, sınırlılıklar, genelleme)
6. Conclusions and Future Work
References
Appendices

3. Grup A – Güvenilir Robot Yazılımı (UR5e / Gazebo / MoveIt2)
Bu projeler, güvenilir robot yazılımı geliştirme hedeflerini lisans seviyesinde yönetilebilir parçalara ayırmak üzere tasarlanmıştır. Her proje birbirinden bağımsızdır (tek öğrenci, tek repo) ve standart platform olarak ROS2 Humble + Gazebo simülasyonunu kullanır. Fiziki robot zorunluluğu yoktur.
Zorunlu Giriş Şartı – UR5e Gazebo + MoveIt2 Starter Kit
Tüm Grup A öğrencilerinin ilk 2–3 hafta içinde tamamlaması gereken ortak altyapı. Bu starter kit bitirme projesi sayılmaz; önkoşuldur.
UR5e Gazebo simülasyonunu (Gazebo Classic 11) çalıştır.
ros2_control kontrolörlerini doğrula: joint_state_broadcaster + joint_trajectory_controller.
MoveIt2 konfigürasyonunu çalıştır: planlama + yürütme.
En az 1 senaryoda rosbag2 kayıt al ve replay ile tekrar üretilebilirliği göster.
Kurulum ve çalıştırmayı tek komutla otomatikleştir (script veya Docker).
README'ye: kurulum, çalıştırma, hata giderme ve demo videosu linklerini ekle.

Grup A – Proje Özet Tablosu
No
Proje Başlığı
Zorluk
Donanım
A1
Deney Senaryo Otomasyonu: rosbag2 + Replay + Metrik Üretimi
Orta
Yazılım
A2
ROS2 Güvenlik Denetçisi: Workspace + Hız/İvme Limitleri + Güvenli Durdurma
Orta
Yazılım
A3
Robot Kontrol Kodunda Statik Analiz ve Güvenlik Skoru (LLM Kodları Dahil)
Orta-Zor
Yazılım
A4
Gray-Box Adversarial Prompt/Suffix Test Platformu + Simülasyon Güvenlik Skoru
Zor
Yazılım
A5
ROS2 Hata Enjeksiyonu ile Dayanıklılık: Gecikme/Dropout/TF Hataları
Orta
Yazılım
A6
Jetson Orin + RealSense + Termal Kamera: Yakınlık Algılama ve ROS2 Entegrasyonu
Orta
Var
A7
Endüstriyel Kalite Şablonu: Docker + CI + SBOM + Sürümleme (ROS2 Humble)
Kolay-Orta
Yazılım
A8
Unity Robot Seçici (UR5e/KUKA KR3/ABB GoFa) + ROS2 Model Switcher
Orta-Zor
Yazılım
A9
ROS2 Humble Kritik/Savunma Risk Analizi: SROS2, DDS-Security
Orta
Yazılım
A10
Çok-Robot MoveIt2 Benchmark: UR5e/KR3/GoFa Planlama Karşılaştırması
Orta
Yazılım


A1 – Deney Senaryo Otomasyonu: rosbag2 + Replay + Metrik Üretimi ELVIN DAVIDOV
Amaç
UR5e senaryolarını otomatik koşturan bir deney çalıştırma altyapısı geliştirmek. Her koşu için otomatik kayıt, metrik çıkarımı ve raporlama üretmek.
Zorluk Seviyesi
Orta
Gerekli Donanım
Yok (Simülasyon)
Repo Adı
ur5e-experiment-automation
Teslim Edilecekler
Senaryo koşucu + analiz scriptleri, 3 senaryo × 10 tekrar CSV + otomatik rapor, demo videoları.

Adım Adım Yapılacaklar
Adım
Açıklama
1
En az 3 görev senaryosu tanımla (home→pose, waypoint, pick-place).
2
Senaryo koşucu düğümü yaz: hedefler, timeout, başarı kriterleri.
3
Kayıt şablonu oluştur: joint_states, TF, planlanan/yürütülen trajectory, olay log'u.
4
Replay + analiz pipeline'ı yaz: bag → metrik çıkarımı (süre, hedef hatası, yol uzunluğu, başarım).
5
YAML ile deney konfigürasyonu; tek komutla 10 tekrar koşabilsin.
6
Sonuçları CSV + kısa otomatik rapor olarak üret (tablo/grafik).


A2 – ROS2 Güvenlik Denetçisi: Workspace + Hız/İvme Limitleri + Güvenli Durdurma ELVIN DAVIDOV
Amaç
UR5e hareketlerini izleyen ve güvenlik ihlallerinde yavaşlatma/durdurma uygulayan bir ROS2 denetçi geliştirmek. Tespit ve tepki metriklerini ölçmek.
Zorluk Seviyesi
Orta
Gerekli Donanım
Yok (Simülasyon)
Repo Adı
ur5e-safety-supervisor
Teslim Edilecekler
Denetçi ROS2 paketi + test senaryoları, CSV + grafiklerle metrik raporu, demo videosu.

Adım Adım Yapılacaklar
Adım
Açıklama
1
Kuralları tanımla: joint limitleri, workspace bounding box, yasak bölgeler.
2
Denetçi düğümü geliştir: joint_states/TF dinle, ihlal tespiti yap, olay log'u üret.
3
Tepki mantığı: soft hız ölçekleme + hard stop (simülasyonda).
4
MoveIt2 ile entegrasyon: planlanan trajectory pre-check + runtime kontrol.
5
Her kural için en az 3 ihlal ve 3 normal test senaryosu oluştur.
6
Metrikleri çıkar: tespit gecikmesi, FP/FN, stop mesafesi, recovery süresi.
7
Launch test + CI entegrasyonu ekle.


A3 – Robot Kontrol Kodunda Statik Analiz ve Güvenlik Skoru (LLM Kodları Dahil) KAMAL ASADOV
Amaç
ROS2 kontrol düğümlerindeki güvenlik anti-pattern'lerini statik analiz ile yakalamak ve skorlamak. CI içinde otomatik raporlayıp 'merge gate' olarak çalıştırmak.
Zorluk Seviyesi
Orta-Zor
Gerekli Donanım
Yok (Bilgisayar)
Repo Adı
ros2-static-safety-analyzer
Teslim Edilecekler
Statik analiz aracı + 10+ kural + testler, CI raporlama, LLM çıktı değerlendirmesi.

Adım Adım Yapılacaklar
Adım
Açıklama
1
Kural seti tanımla (timeout yok, exception yok, stop yok, QoS yanlış, vb.).
2
Python için AST tabanlı kural motoru yaz; her kural için pozitif/negatif örnek üret.
3
C++ için en az 2 kontrolü clang-tidy/cppcheck ile ekle (veya minimal checker).
4
Raporlama: SARIF/Markdown üret ve CI'de yayınla.
5
En az 30 LLM-üretimi kontrol scripti oluştur ve analizden geçir; bulguları sınıflandır.
6
Simülasyonda: analiz fail ise çalıştırmayı engelle; pass ise senaryoyu koştur.


A4 – Gray-Box Adversarial Prompt/Suffix Test Platformu + Simülasyon Güvenlik Skoru TOFIG VALIYEV
Amaç
LLM API ile UR5e görev kodu üreten ve simülasyon + denetçi ile puanlayan test platformu geliştirmek. Prompt/suffix varyantlarının 'unsafe' davranışa etkisini nicel ölçmek.
Zorluk Seviyesi
Zor
Gerekli Donanım
Yok (Simülasyon + LLM API)
Repo Adı
llm-adversarial-robot-test
Teslim Edilecekler
Test runner + raporlama (CSV + Markdown), 50+ koşu deney seti, demo videosu.

Adım Adım Yapılacaklar
Adım
Açıklama
1
En az 3 görev tanımla (pose, waypoint, pick-place).
2
Prompt şablonları oluştur: güvenli baseline + riskli varyantlar + suffix varyantları.
3
Üretilen kodu container içinde derle/çalıştır pipeline'ı kur.
4
Safety supervisor olaylarını, ihlalleri ve timeout'ları otomatik topla.
5
Metrikler: unsafe oranı, safe başarı oranı, engelleme oranı, tespit gecikmesi.
6
50+ koşu ile deney seti üret; sonuçları CSV + özet raporla sun.



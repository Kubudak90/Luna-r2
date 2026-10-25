# Luna R2 — tasarım (25.10.2026)

R2, R1'in yerine geçen ikinci tasarım. İki hedefi var:
- **Telefon gerekmesin.** Yüz, kafadaki ekranda çalışıyor. Çocuk ekrana dokununca Luna dinliyor.
- **Bütün parçalar Türkiye'den alınabilsin.** Parçaların nereden alınacağı ve fiyatları `docs/MALZEME_LISTESI.md` belgesinde.

Ölçülerin tek kaynağı `luna_ws/src/luna_robot/urdf/luna.urdf`. Bu belge, o sayıların neden öyle seçildiğini anlatıyor. Belgedeki hiçbir şey gerçek donanımda denenmedi. Doğrulananlar (simülatör, testler, derleme) en sonda ayrı listelendi.

## R1'den farklar

| | R1 | R2 | Neden |
|---|---|---|---|
| Yüz | Telefon, kafada | Kafada sabit 5,5" AMOLED dokunmatik ekran | Telefonsuz kullanım. AMOLED'de siyah piksel söner, bu yüzden yalnız gözler parlar. |
| Kafa | Pan/tilt servolu, iki göz kamerası | Sabit kafa, ekranın üstünde tek küçük kamera | Bakışı ekrandaki gözler veriyor. Göz teması için tek kamera yetiyor. Servo, mekanizma ve kablo gitti. |
| LiDAR | Direkte, Z590 | Kafanın tepesinde, dönme ekseninin üstünde, Z550 | Daha sade görünüm. Robotun hiçbir parçası tarama düzleminin üstünde değil. |
| Yürüyüş | Palet | Ortada 2 tahrikli teker + önde ve arkada birer sarhoş teker | Aşağıda, "Şasi" bölümünde. |
| Gövde | 340 × 378 mm dikdörtgen | Ø 360 mm yuvarlak | Yerinde dönerken kendi dairesinin dışına çıkmıyor. Simülatörde 60 ve 70 cm'lik kapılardan geçiyor (R1 70 cm'den geçemiyordu). |
| Tampon | Önde ve arkada şerit | Gövdeyi tam saran halka, Z15–110 | Yandan çarpmayı da algılıyor. 1,5 cm'den yüksek oyuncakları da hissediyor. |
| Bilgisayar | Jetson AGX Xavier | Jetson Orin NX 16 GB (Waveshare Super kit, OpenZeka) | Türkiye'de stokta bulunan en iyi Orin. Dil modeli Kimi bulutunda çalıştığı için 16 GB yetiyor. |
| Derinlik kamerası | OAK-D Lite (pasif stereo) | Orbbec Gemini 336 (IR projektörlü aktif stereo, IMU) | OAK-D Türkiye'de ya stoksuz ya çok pahalı. Gemini düz duvarda ve karanlıkta da derinlik veriyor. Jiroskopu odometriye giriyor. |
| Mikrofon | XVF3800 | ReSpeaker Mic Array v2.0 (XVF-3000) | XVF3800 Türkiye'de satılmıyor. |
| Pil | 4S3P NMC 26650, JBD BMS | 4S2P LiFePO4 32700, Daly akıllı BMS | Çocuk odası için termal olarak çok daha güvenli bir kimya. JBD Türkiye'de satılmıyor, Daly satılıyor. |
| Uçurum sensörü | VL53L4CD | VL53L1X | VL53L4CD stokta yok. Köşe sensörleriyle aynı parça oluyor. |

## Ölçüler ve yükseklik bantları

Yukarıdan bakınca robot Ø 360 mm bir daire. Yandan bakınca, aşağıdan yukarıya şu parçalar var (`core/sim2d.robot_bands`, simülatörün temas modeli de bu bantları kullanıyor):

| Bant | Yükseklik | Ölçü | İçinde |
|---|---|---|---|
| Şasi | Z0–15 | Ø 330 | Tekerler, sarhoş tekerler, taban plakası, dock kontak şeritleri |
| Tampon halkası | Z15–110 | Ø 360 (basılınca 340) | 4 mikro anahtar, 4 köşe ToF penceresi (Z100) |
| Gövde | Z110–345 | Ø 340 | Pil, Orin, Teensy, sürücü, Gemini 336 (ön pencere, Z270, 15° aşağı bakar) |
| Boyun | Z345–360 | Ø 80 | Kablolar |
| Kafa | Z360–520 | 120 × 220 mm (derinlik × en) | Ekran (merkezi Z440, 8° geriye yatık), yüz kamerası (Z505, 12° yukarı), gizlilik sürgüsü |
| LiDAR | Z520–561 | Ø 56 | RPLIDAR C1, lazer düzlemi Z550 |

Toplam yükseklik **561 mm**. Mikrofon dizisi omuz plakasında, boynun arkasında, Z348'de duruyor.

## Şasi

**Seçim: ortada iki tahrikli teker, önde ve arkada birer sarhoş teker (bilyalı).** Tekerler 80 × 10 mm, eksenleri y ±150 mm'de. Motorlar Pololu 37D 70:1. Sarhoş tekerler 1" bilyalı, merkez çizgisi üzerinde x ±110 mm'de.

**Neden 4 tahrikli teker değil?**
- **Dönüşte kayma.** Dört tahrikli tekerle dönmek, tekerleri yana sürterek olur (kaymalı yönlendirme). R1'in paleti de aynı sorunu yaşadı: dönüşler odometrede %45'e varan hatayla çıkıyordu (`true_turn_slip` 1,45). Yana sürtünme parkeyi çizer, halıda takılır, motor akımını artırır.
- **Yerinde dönme.** İki tahrikli teker tam ortadayken robot kendi merkezi çevresinde döner. Yuvarlak gövde de dönerken kendi dairesinin dışına taşmaz. Bu yüzden Nav2'nin planlayıcısına dönüş payı gerekmiyor (`nav2.yaml`). Simülatörde 70 ve 80 cm'lik kapılardan geçiyor; R1'de 70 cm'lik kapı kapalıydı. 60 cm'de de geçiyor (2/2), ama haritanın 5 cm'lik ızgarasında yalnız bir hücre pay kalıyor (README, "Engel algılama").
- **Odometri.** İki tekerde dönüş geometriden hesaplanır. Ayrıca Gemini'nin jiroskopu yön bilgisini düzeltiyor (`base_node`). SLAM'e daha temiz bir başlangıç tahmini gidiyor.
- **Basitlik.** 2 motor, 1 sürücü, 2 enkoder. Dört tekerde bunların hepsi iki katı olurdu.

**Dikkat edilmesi gereken: dört temas noktası.** İki teker ve iki sarhoş teker rijit bir şasideyse, düz olmayan zeminde biri havada kalabilir. Tahrikli bir teker havaya kalkarsa robot çekişini kaybeder. Bu yüzden **sarhoş tekerlerden biri yaylı olmalı**: yaklaşık 5 mm strok, ağırlığın küçük bir kısmını taşıyacak bir yay. Böylece tahrikli tekerler hep yerde kalır. Diğer çözüm, tekerleri sallanır bir kiriş üzerine koymak; o daha karmaşık.

**Devrilme.** Ağırlık merkezi alçakta: pil ve sürücü şasinin hemen üstünde, Orin gövdenin altında. Ağırlık merkezi yaklaşık Z150 alınırsa, öne veya arkaya devrilme ancak 0,7 g'nin üstündeki bir frenlemeyle olur. Bu, robotun üretebileceği frenlemenin çok üstünde. Tahmindir, ilk prototipte tartılıp ölçülmeli.

**Hız.** En çok 0,30 m/s doğrusal ve 1,5 rad/s dönüş (`core/kinematics.DriveLimits`). Nav2 1,2 rad/s kullanıyor. 150 rpm'lik motor 80 mm tekerle yüksüzde 0,63 m/s yapar; yani motorların yarı hızında bile bol pay kalıyor.

## Kafa ve yüz

- **Ekran:** Waveshare 5,5" AMOLED, 1920 × 1080, HDMI, USB dokunmatik. Kafada yatay duruyor ve dumanlı akrilik bir kapağın arkasında.
- **Yüz yazılımı:** `web/face.html` ve `web/face.js` dosyaları, `face_bridge_node` tarafından `http://127.0.0.1:8765/` adresinde sunuluyor. Kafadaki tarayıcıyı `tools/kiosk.sh` açıyor: ekranı ve dokunmatiği 90° çeviriyor, ekranın kararmasını kapatıyor.
- **Yüz olayları:** dinliyor, düşünüyor, konuşuyor, pil azaldı ve mikrofon kapalı için ayrı yüzler var. Olayları `core/face_mood.py` üretiyor.
- **Dokunma:**
  - Dokunmak ya da basılı tutmak "soru soracağım" demek.
  - Parmağı ekranda gezdirmek okşamak demek.
  - Luna dock'ta uyurken ya da mikrofon kapalıyken dokunma robota gitmez. Yüz yalnız tepki verir: uyurken bir göz aralanır, mikrofon kapalıysa yüz "hayır" diye sallanır.
- **Yüz kamerası:** Raspberry Pi Camera v2 (IMX219, 62° × 49°). Ekranın üstünde, 12° yukarı bakıyor; ayaktaki ve oturan çocuk ikisi de kadraja giriyor. Orin'e CSI ile bağlı.
  - `face_track_node` YuNet yüz algılayıcıyla yüzü buluyor. Yüzün yönü gözlere gidiyor.
  - Yüz kadrajın kenarına kaçmadan (±22°) gövde de yavaşça o yöne dönüyor, en çok 30°/s.
  - Görüntü kaydedilmiyor, robottan çıkmıyor.
- **Neden iki göz kamerası değil?** Gözler ekranda çizildiği için kameranın gözün yerinde olması gerekmiyor. Göz teması hissi, gözlerin çocuğun yüzüne dönmesinden geliyor.

## Algılama

- **RPLIDAR C1** kafanın tepesinde, dönme ekseninin tam üstünde. SLAM ve Nav2 için kullanılıyor.
  - Tarama düzlemi Z550. Bunun altında kalan masa, sandalye oturağı ve oyuncak gibi şeyler bu düzlemde görünmüyor.
- **Gemini 336**, gövdenin önünde Z270'te, 15° aşağı bakıyor.
  - Derinlik: 90° × 65°, Nav2'nin voksel katmanına ve çarpışma izleyicisine gidiyor. Yerden 3 cm ile 58 cm arası engel sayılıyor.
  - Renk kamerası: dock etiketini okuyor.
  - IMU'nun jiroskopu: yön bilgisi.
  - Simülatördeki kör bölge: zemini gövdenin 0,41 m önünden başlayarak görüyor. Daha yakınını köşe ToF'ları ve tampon kapatıyor.
- **4 köşe VL53L1X**, tampon halkasının pencerelerinde, Z100'de, **düz ileri ve düz geri** bakıyor.
  - İlk taslakta yanları da kapsasın diye 30° dışa çevrilmişlerdi. Simülatörde bunun bir zararı çıktı: `tof_range_node` her okumayı koninin bütün genişliğinde bir yay olarak haritaya işliyor. Yan duvarı eğik gören bir koni, yayın uçlarını duvarın 14–20 cm önüne düşürüyor ve 68 cm'lik bir geçidi kapatıyordu.
  - Düz bakınca yandaki duvar 0,70 m'lik menzilin dışında kalıyor. Bu ancak dışa dönüklük 7°'nin altındayken sağlanıyor.
  - Yuvarlak gövde yerinde dönerken yanlara taşmadığı için yan kapsama gerekmiyor. Önü ayrıca Gemini görüyor (90°).
- **Tampon halkası:** ön yarım halka ±45°'deki iki mikro anahtara (FL, FR) yaslanıyor, arka yarım halka da aynı şekilde (BL, BR).
  - Tam önden ya da arkadan (±20° içinde) gelen çarpma iki anahtarı birden kapatıyor.
  - Halka, gövde sert bir şeye değmeden 10 mm önce kapanıyor.
  - Simülatördeki karşılığı: `core/sim2d.bumper_bits`.

## Uçurum

Tabanda 4 VL53L1X aşağı bakıyor. Pencereleri Z60'ta ve x ±154 mm'de, yani sarhoş tekerlerin (x ±110) 44 mm önünde ve arkasında. Okuma 100 mm'yi geçerse (zemin 60 mm + 40 mm pay) ya da sensör okuyamazsa `base_node` hareketi keser. Sensör okumazsa da uçurum sayılıyor.

**Duruş bütçesi (0,30 m/s'de):**
- Algılama gecikmesi en kötü ~65 ms: örnek 20 ms + MCU yayını 20 ms + iletim + `base_node`. Bu sürede robot ~20 mm yol alır.
- Fren mesafesi:
  - Hız döngüsü tekeri aktif durdurursa, 3 m/s² ile ~15 mm.
  - **Simülatördeki gibi 0,15 s gecikmeli bir motor modeliyse ~45 mm**; bu da toplamı 65 mm'ye çıkarır, 44 mm'yi geçer.
- **Sonuç:** sarhoş teker kenardan düşmeden durabilmek için hız döngüsünün aktif fren yapması gerekiyor.
- **İlk prototipte ölçülecek:** 0,30 m/s'de masa kenarına doğru giderken duruş mesafesi 40 mm'nin altında olmalı.
- Olmazsa iki çare var, ikisi birlikte de uygulanabilir:
  - Uçurum durdurmasını MCU'nun içine almak: Teensy, `base_node`'u beklemeden motoru keser. Gecikme 20 ms'ye iner.
  - Hızı 0,25 m/s'ye düşürmek.

## Ses

- **ReSpeaker Mic Array v2.0** omuz plakasında, dört mikrofonu yukarı bakıyor.
  - Gövdeye sabit: sesin geldiği yön, bir kez kalibre edildikten sonra doğrudan gövdeye göre açı olur (`doa_zero_deg`, `doa_direction`).
  - XVF-3000 yankı giderici, Luna'nın kendi sesini referans olarak kullanıyor. Bunun için hoparlör dizinin 3,5 mm çıkışından besleniyor: dizi → küçük amfi → hoparlör.
- **Ses zinciri R1'deki gibi** (README, "Ses zinciri"):
  - Konuşma whisper.cpp ile robotta metne çevriliyor.
  - Cevap Kimi bulutundan geliyor.
  - Piper ile Türkçe seslendiriliyor.
  - Orin NX'te whisper GPU'da (CUDA, sm_87) çalışıyor.

## Gizlilik

Kafanın üstünde, kameranın yanında **mekanik bir sürgü** var. Kapatınca:
- Yüz kamerasının merceğini örter.
- Mikrofon dizisinin USB 5 V hattını (VBUS) bir anahtarla keser. Dizi USB'den tamamen düşer. Yazılım bunu bir hata olarak değil "gizlilik" olarak okur (`audio_node`, `/luna/privacy`).
- Aynı anahtar, ekranın yanındaki kırmızı "mikrofon kapalı" LED'ini yakar.

**Sonuçları:**
- **Sürgü kapalıyken Luna ne duyar ne konuşur**, çünkü hoparlör de dizinin ses kartından besleniyor. Yüz çalışmaya devam eder ve mikrofon kapalı simgesini gösterir.
- Yazılım hiçbir zaman ALSA'nın "default" aygıtına geri düşmez. Dizi yokken başka bir mikrofonu açması mümkün değil.

Yazılımdaki susturma (`/luna/audio/mute`) ayrı bir şey: yalnız sesi atar, donanımı kapatmaz.

## Güç

**Paket:** 4S2P LiFePO4, 8 × 32700 hücre (3,2 V, 6 Ah). Toplam 12,8 V, 12 Ah, 154 Wh. Gerilim aralığı 10,0–14,6 V. Bütün hücreler aynı partiden alınmalı.
- **BMS:** Daly 4S LiFePO4 akıllı BMS, 60 A sürümü. UART'tan Teensy'ye bağlı. Bluetooth modülü takılmamalı, çünkü aynı portu kullanıyor.
- **Şarj durumu:** BMS'in kendi sayacından geliyor. LiFePO4'ün gerilim eğrisi %10–90 arasında neredeyse düz olduğu için gerilim yalnız yedek olarak kullanılıyor (`core/battery.py`).

**Ortalama tüketim, tahmin (~35 W):**

| Tüketici | Güç |
|---|---|
| Orin NX (15 W modu) | 15 W |
| Gemini 336 | 3 W |
| Ekran | 2,5 W |
| LiDAR | 1,2 W |
| Mikrofon dizisi ve amfi | 2 W |
| Teensy ve sensörler | 1 W |
| Kamera | 1 W |
| Motorlar (ortalama) | 5 W |
| Dönüşüm kayıpları | ~4 W |

**Süre:** kullanılabilir 140 Wh / 35 W ≈ **4 saat**. Dock'taki 14,6 V 4 A şarj cihazıyla dolum yaklaşık 3 saat sürer. Bu değerler `core/mock_plant.PlantConfig`'de de aynı.

**Güç dağıtımı:**
- Paket → 10 A sigorta → ana anahtar → üç kola ayrılır. Acil durdurma ana gücü kesmez, yalnız motorları durdurur ("Elektrik"); Orin açık kalır.
  - Motor sürücüsü (Pololu G2 18v18, 6,5–30 V): doğrudan paketten.
  - Orin taşıyıcı kartı (Waveshare, 9–19 V girişli): doğrudan paketten, 5 A sigortayla. LiFePO4'ün en düşük gerilimi (10 V) bile bu aralıkta, ayrı bir regülatöre gerek yok.
  - 5 V hattı: Pololu D24V50F5 (5 A). Ekran, LiDAR, Teensy, sensörler ve USB hub buradan beslenir.
- **Güç modu:** Orin şarjdayken, pil azken ya da sıcakken 10 W'a, diğer zamanlarda 15 W'a geçiyor (`core/battery.decide`). 25 W yalnız elle açılıyor.

## Elektrik

- **Pin haritası:** `firmware/luna_mcu/src/pins.h`.
  - R1'deki pinler korundu, yalnız servo pinleri (27–29) boşaldı.
  - Enkoderler 5 V ile besleniyor ve seviye dönüştürücüden (Pololu #2595) geçiyor.
- **Acil durdurma:** iki NC kontaklı mantar buton.
  - Kontak 1, sürücünün SLP hattını donanım olarak keser.
  - Kontak 2, Teensy'ye "basıldı" bilgisini verir (`ESTOP_SENSE`).
  - `DRV_SLP` hattında 10k'lık aşağı çeken direnç var; Teensy sıfırlanırken motorlar uyanık kalmaz.
- **Orin'in bağlantıları:**
  - HDMI: ekran.
  - CSI: yüz kamerası.
  - USB 3: Gemini 336.
  - USB 2, küçük bir hub üzerinden: Teensy, LiDAR'ın USB-UART çevirici, ekranın dokunmatiği, mikrofon dizisi. Dizinin VBUS hattı gizlilik anahtarından geçiyor.
  - Hub kendi 5 V'undan beslenmeli. Gemini doğrudan Orin'e bağlanmalı.
- **Kart sürümü:** Waveshare taşıyıcı kartın HDMI'lı sürümü (JETSON-ORIN-IO-BASE-H) alınmalı. DisplayPort'lu sürüm gelirse aktif bir DP→HDMI çevirici gerekir.

## Dock

Robot dock'a önden girer. Dock'un yapısı (`core/sim2d.dock_parts`, `core/dock_tag.py`):
- **Taban plakası:** 6 mm yüksekliğinde. Robot üstüne çıkar.
- **Teker hunisi:** iki teker için birer yuva. Girişte ±45 mm yanal ve ±14° açı hatasını tolere eder, robotu ±5 mm'ye ortalar.
- **Teker durdurucuları:** robotu oturma noktasında durdurur.
- **Kontaklar:** dock'ta yaylı üç pim (CHG+, P4 kimlik, GND). Robotun altındaki üç bakır şeride değerler. Şeritler 40 × 12 mm, adımları 16 mm; merkezleri dönme ekseninin 60 mm önünde ve 60 mm solunda, Z15'te.
  - Ön sarhoş teker merkez çizgisinde yürüdüğü için pimlerin üzerinden hiç geçmez.
  - Kontaklar alttayken tampon halkası her yönde kesintisiz kalabiliyor.
- **Etiket direği:** duvara dayalı, 36 cm yüksekliğinde. AprilTag 36h11, ID 0, siyah kare 80 mm, beyaz çerçevesiyle 100 mm. 104 mm'lik plaka üzerinde, merkezi 25 cm yükseklikte.
  - Etiket yüzeyi, oturma noktasındaki dönme ekseninin 0,45 m önünde.
  - Renk kamerası etiketi uzaktan oturma noktasına kadar görüyor.
  - Basılacak dosya: `docs/dock_tag36h11_id0_80mm.svg`.
- **Kimlik ve şarj izni:** R1'in Elektrik R01 devresiyle aynı.
  - Dock P4'ü 10k ile 5 V'a çeker, robot 10k ile GND'ye. Robot oturunca P4 2,50 V olur.
  - Robot `DOCK_INHIBIT` ile 2,2k ekleyince P4 0,76 V'a düşer ve dock kontakları ~10 ms içinde kapatır.
  - Robot oturmadıkça pimlerde gerilim yok. Bu, çocuk açısından önemli.
- **Şarj cihazı:** dock'un içinde, 14,6 V 4 A LiFePO4.

**Yaklaşma:**
- Nav2, robotu dock ekseni üzerinde 0,6 m geride bir noktaya getirir.
- Oradan `dock_node` etikete doğru sürer.
- Etikete doğru sürmek, başlangıçtaki yanal sapmayı oturma noktasına gelene kadar 0,45 / (0,6 + 0,45) ≈ 0,43 katına indirir. Bu yüzden devir koşulu bir koni: 0,6 m geride en çok 7,8 cm yanal sapma kabul ediliyor (`core/dock_nav.py`).
- Simülatörde 1,2 m geriden başlayan robot 15 cm yanal sapmayla bile oturup şarj oluyor, 20 cm'de oturamıyor.

## Yazılımda değişenler

- `urdf/luna.urdf`: baştan yazıldı. Eklem yok, tek bir sabit ağaç.
- `core/kinematics.py`: diferansiyel sürüş. Gemini jiroskobuyla yön düzeltmesi yapılıyor ve jiroskobun sapması robot dururken öğreniliyor.
- `head_node` ve kafa çerçeveleri kaldırıldı. `attention_node` artık gövdeyi döndürüyor, gözleri ekranda yönetiyor.
- `face_track_node` yeni: CSI kamera ve YuNet.
- `audio_node` ve `core/respeaker.py`: ReSpeaker dizisi ve gizlilik sürgüsü.
- Dock etiketi artık Gemini'nin renk kamerasıyla okunuyor. Simülatördeki kamera da aynı yerde.
- `core/sim2d.py`: yuvarlak gövde, halka tampon, R2 dock'u. Yeni dünyalar: 60, 70 ve 80 cm'lik kapı (`door60`, `door70`, `door80`).
- Nav2: ayak izi artık daire (`robot_radius` 0,18), planlayıcı payı 0,04.
- Firmware: servolar çıktı. Daly BMS, VL53L1X uçurum sensörleri ve 70:1 enkoder eklendi.
- `web/`: yeni ekran yüzü (`face-core.js`, `face.js`, `face.html`) ve `tools/kiosk.sh`.
- Docker: JetPack 6 (`l4t-jetpack:r36.4.0`). ROS apt'tan kuruluyor; Gemini sürücüsü, whisper.cpp (CUDA) ve Piper imajın içinde.

## Doğrulama durumu

**Doğrulananlar (25.10.2026):**
- Python birim testleri.
- Web yüzü testleri. Gerçek yüz köprüsünden Chromium'a uçtan uca ekran görüntüleri alındı.
- Firmware host testleri.
- Teensy derlemesi.
- x86 ROS Humble konteynerinde: `colcon build`, `colcon test` ve simülatör senaryoları.

Sayılar README'nin "R2 doğrulama" bölümünde.

**Doğrulanmayanlar:**
- **Hiçbir parça alınmadı ve hiçbir şey gerçek donanımda denenmedi.**
- Robot imajı Jetson'da derlenmedi.
- Ağırlık, ağırlık merkezi, fren mesafesi, yaylı sarhoş teker, halıdaki çekiş, Daly akım işareti, Gemini ve kamera kalibrasyonu ilk prototipte ölçülmeli.

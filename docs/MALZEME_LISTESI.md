# Luna R2 — malzeme listesi (Türkiye, 25.10.2026)

Bu listedeki her parça Türkiye'deki bir satıcıda listeleniyor. Fiyatlar 25.10.2026'da satıcı sayfalarından ve arama sonuçlarından alındı. Stok ve fiyat sık değişiyor, sipariş vermeden önce sayfayı kontrol edin.

- "KDV belirsiz" yazanlarda sayfa KDV'nin dahil olup olmadığını söylemiyor.
- "—" yazanların fiyatına bakılmadı. Bunlar her satıcıda bulunan küçük parçalar.
- Tasarımın gerekçeleri `docs/TASARIM_R2.md` belgesinde.

## Ana parçalar

| # | Parça | Adet | Satıcı | Fiyat (TL) | Not |
|---|---|---|---|---|---|
| 1 | **Waveshare Super Jetson Orin NX 16 GB yapay zekâ kiti** (taşıyıcı kart, modül, aktif soğutucu) | 1 | [OpenZeka](https://openzeka.com/urun/waveshare-super-jetson-orin-nx-16gb-yapay-zeka-kiti/) | 60.742,80 (KDV dahil; havale 58.920,52) | 5 adet stokta göründü. OpenZeka, NVIDIA'nın Türkiye distribütörü. Kitte SSD, Wi-Fi ve adaptör olup olmadığını ve kartın **HDMI'lı** sürüm olduğunu satıcıya sorun. |
| 2 | NVMe SSD, M.2 2280, 256 GB | 1 | Herhangi bir bilgisayar mağazası | — | 1. maddedeki kitte yoksa alınacak. |
| 3 | **Orbbec Gemini 336** derinlik kamerası | 1 | [Robot Sepeti](https://www.robotsepeti.com/stereo-depth-camera) | 33.359,89 (KDV belirsiz) | Tedarik 7–12 iş günü. Gemini 335 aynı sayfada 30.981,40 TL; IR geçiren filtresi olmadığı için güneşli odada daha zayıf. |
| 4 | **Slamtec RPLIDAR C1** | 1 | [Robot Sepeti](https://www.robotsepeti.com/slamtec-rplidar-c1-c1m1-r2-360-lazer-tarayici-12m-5k) | 7.591,47 (KDV dahil) | [Hepsiburada](https://www.hepsiburada.com/rplidar-c1-c1m1-r2-360-lidar-lazer-tarayici-12m-5k-pm-HBC0000A6UNRO)'da 8.000 TL, stokta. [Robotistan](https://www.robotistan.com/slamtec-rplidar-c1-lidar-laser-scanner-en) 4.563,29 TL ama stokta yok. |
| 5 | **Waveshare 5,5" HDMI AMOLED**, 1080 × 1920, kapasitif dokunmatik | 1 | [Hepsiburada](https://www.hepsiburada.com/waveshare-5-5inc-kapasitif-dokunmatik-amoled-ekran-modulu-1080-1920-piksel-hdmi-sertlestirilmis-cam-panel-pm-HBC00006QWSYN) | 9.561,82 | Stokta. [Robot Sepeti](https://www.robotsepeti.com/waveshare-55inch-kapasitif-dokunmatik-amoled-ekran-10801920-hdmi-16103) 5.950,73 + KDV ama stokta yok. |
| 6 | **Raspberry Pi Camera v2** (IMX219, 8 MP) ve Jetson için 22→15 pin CSI kablosu | 1 | [OpenZeka](https://openzeka.com/urun/raspberry-pi-kamera-v2/), [SAMM](https://market.samm.com/raspberry-pi-kamera-v2) | — | Çoğu satıcıda stok yok. NoIR sürümü SAMM'da stokta (56 adet); yüz bulmada çalışır ama renkler pembeleşir. |
| 7 | **ReSpeaker Mic Array v2.0** (XVF-3000, USB) | 1 | [Havya Tech](https://www.havyatech.com/products/respeaker-mic-array-v20) | 4.912,39 (indirimli; normali 6.877,35) | Stokta göründü. [Robotistan](https://www.robotistan.com/respeaker-mic-array-v20-eng) 6.472,64 TL, stokta yok. |
| 8 | **Teensy 4.1** | 1 | [SAMM](https://market.samm.com/en/sparkfun-teensy-41) | 2.870,11 | 50 adet stokta. Akakçe'de en ucuzu 2.796 TL. |
| 9 | **Pololu 37D 70:1** metal redüktörlü motor, 12 V, 64 CPR enkoderli (#4754) | 2 | [Robot Sepeti](https://www.robotsepeti.com/701-metal-gearmotor-37dx70l-mm-12v-with-64-cpr-encoder-helical-pinion) | 2 × 4.817,97 = 9.635,94 (KDV belirsiz) | 150 rpm, 27 kg·cm, çıkış milinde 4480 CPR. |
| 10 | **Pololu G2 18v18** çift motor sürücü | 1 | [Robot Sepeti](https://www.robotsepeti.com/pololu-g2-hp-dual-dc-motor-surucu-shield-18v18-arduino-uyumlu-pl-2515) | 3.911,10 (KDV belirsiz) | Kanal başına 18 A, 6,5–30 V, akım ölçüm çıkışı var. |
| 11 | **Daly 4S 12,8 V LiFePO4 akıllı BMS**, 60 A, UART | 1 | [Pilburada](https://www.pilburada.com/daly-4s-128v-60a-lifepo4-smart-bms-btuart485canbus-77189) | ≈ 3.164 (Hepsiburada'daki ilan) | Bluetooth modülü UART portuyla ortak; takmayın. |
| 12 | **LiFePO4 32700 hücre**, 3,2 V 6 Ah, 3C (Power-Xtra IFR32700) | 8 | [Pilburada](https://www.pilburada.com/power-xtra-ifr32700-3.2v-6000-mah-lifepo4-sarjli-pil-3c) | 8 × 176 = 1.408 | Hepsi aynı partiden olmalı; 4'lü ya da 50'li paket alın. Nikel şeritle punta kaynağı yapılır, lehim yapılmaz. |
| 13 | **14,6 V 4 A LiFePO4 şarj cihazı** (Power-Xtra 4S), dock'un içine | 1 | [Pilburada](https://www.pilburada.com/power-xtra-4s-14.6v-4a-lifepo4-pil-sarj-cihazi) | — | |
| 14 | **VL53L1X ToF sensör kartı** (4 köşe + 4 zemin) | 8 | [Robotistan](https://www.robotistan.com/vl53l1x-time-of-flight-distance-sensor), [Robot Sepeti (Pololu)](https://www.robotsepeti.com/pololu-vl53l1x-voltaj-regulatorlu-tof-uzaklik-sensoru-400cm-max-pl-3415), [Motorobit](https://www.motorobit.com/tof400c-vl53l1x-4m-distance-sensor-module) | ≈ 8 × 475–570 | XSHUT pini dışarı verilmiş kart seçilmeli. Pololu kartında seviye dönüştürücü ve regülatör hazır. |
| 15 | **Pololu D24V50F5**, 5 V 5 A düşürücü regülatör | 1 | [Robotistan](https://www.robotistan.com/pololu-5v-5a-step-down-voltage-regulator-d24v50f5), [Robot Sepeti](https://www.robotsepeti.com/pololu-5v-5a-voltaj-dusurucu-regulator-d24v50f5) | 2.247,99 | Robotistan'ın bir sayfası stokta yok diyor. |

**Yukarıdaki fiyatı bilinenlerin toplamı:** yaklaşık **143.500 TL**. VL53L1X'ler adet başı 520 TL alındı.

## Yürüyüş mekaniği

| # | Parça | Adet | Satıcı | Not |
|---|---|---|---|---|
| 16 | Pololu 80 × 10 mm teker çifti (PL-1431) | 1 çift | Robot Sepeti | 37D ile uyumlu olduğu yazıyor. Halıda çekişi ilk prototipte denenmeli. 90 mm'lik çift Türkiye'de bulunamadı. |
| 17 | Pololu 6 mm alüminyum montaj göbeği (çift) | 1 çift | [Robotistan](https://www.robotistan.com/wheel-hub), SAMM | 80 ve 90 mm Pololu tekerlerle uyumlu. |
| 18 | Pololu 37D alüminyum L braket (PL-1995) | 2 | [Robot Sepeti](https://www.robotsepeti.com/mounting-brackets-hubs) | |
| 19 | Sarhoş teker, 1" bilyalı (Pololu) | 2 | [Direnc.net](https://www.direnc.net/sarhos-teker-ball-caster-with-1-plastic-ball-old-version-pololu), [Robotistan](https://www.robotistan.com/ball-caster-with-1-plastic-ball-and-ball-bearings) | Biri yaylı takılacak (TASARIM_R2.md, "Şasi"). Yükseklik ara parçalarla teker eksenine göre ayarlanır. |

## Küçük parçalar

Hepsi Robotistan, Direnc.net, Robot Sepeti ve benzeri satıcılarda bulunuyor. Fiyatlarına bakılmadı; hepsi birlikte kabaca 3.000–6.000 TL tutar (tahmin).

- Acil durdurma: mantar buton, **iki NC kontaklı**, 22 mm.
- Tampon anahtarları: 4 × Omron D2F-01L (ya da muadili) mikro anahtar.
- Seviye dönüştürücü: Pololu 4 kanallı çift yönlü (#2595), enkoderler için.
- Sigorta ve anahtar: 10 A ve 5 A bıçak sigorta ve yuvaları, ana anahtar (en az 10 A DC).
- USB: kendi beslemeli 4 portlu USB 2.0 hub; kısa bir USB kablosu, VBUS hattı gizlilik anahtarından geçirilecek.
- Gizlilik sürgüsü için küçük bir sürgülü anahtar (2 kutup: dizinin VBUS'u ve kırmızı LED).
- Ses: 3 W hoparlör, PAM8403 amfi, 3,5 mm jak kablosu.
- Dock:
  - 3 yaylı pim (pogo, en az 5 A), robotun altına 3 bakır şerit.
  - P4 devresi: 10k ve 2,2k dirençler, AO3400A MOSFET, BAT54S, 100 nF.
  - Teker hunisi ve durdurucular: 3B baskı ya da kontrplak.
- Etiket: `docs/dock_tag36h11_id0_80mm.svg` mat kâğıda %100 ölçekte basılır ve 104 mm'lik plakaya yapıştırılır.
- Kablo ve bağlantı: XT30/XT60 güç konnektörleri, JST, silikon kablo, nikel şerit.
- Gövde: kabuk ve kafa 3B baskı ya da termoform; ekran için dumanlı akrilik kapak; tampon halkası için yaylar.

## Toplam

Yaklaşık **155.000–160.000 TL**. Bunun ~143.500 TL'si fiyatı bilinen parçalar; kalanı fiyatı bakılmamış mekanik ve küçük parçalar için tahmin. En büyük kalemler Orin kiti (%39) ve Gemini 336 (%21).

## Bulunamayanlar ve yerine konanlar

| R1'de / ilk planda | Türkiye'de | Yerine |
|---|---|---|
| Luxonis OAK-D Pro | Yalnız 360Teknoloji: sabit odaklı 36.990 TL, otomatik odaklı 82.262 TL; stok doğrulanmadı | Orbbec Gemini 336 |
| ReSpeaker XVF3800 | Hiçbir Türk satıcıda yok (yalnız Avrupa, 61–84 €) | ReSpeaker Mic Array v2.0 |
| JBD akıllı BMS | Türk satıcısı bulunamadı | Daly akıllı BMS ve LiFePO4 paket |
| VL53L4CD | Hiçbir Türk satıcıda yok | VL53L1X |
| BNO085 IMU | Türk satıcısı bulunamadı | Gemini 336'nın kendi IMU'su (ayrı IMU kartına gerek kalmadı) |
| USB yüz kamerası (Arducam, OV2710) | Bakılan modellerin hepsi stokta yok | Raspberry Pi Camera v2, CSI üzerinden |
| Pololu 90 × 10 mm teker | Bulunamadı | 80 × 10 mm |

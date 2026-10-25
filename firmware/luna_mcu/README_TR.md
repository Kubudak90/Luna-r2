# luna_mcu — Teensy 4.1 micro-ROS yazılımı (R2, 2026-10-25)

R1 yazılımından devam ediyor. R2'de değişenler:
- **Kafa servoları yok.** R2'nin kafası sabit; `/luna/mcu/head_us` aboneliği, servo pinleri ve servo regülatörlerinin EN hattı kaldırıldı. Pin 27, 28, 29 boşta.
- **Pil yönetimi: JBD yerine Daly akıllı BMS.** JBD Türkiye'de satılmıyor. Paket artık 4S2P LiFePO4.
- **Uçurum sensörleri: VL53L4CD yerine VL53L1X.** VL53L4CD Türkiye'de stokta yok. Böylece köşe ve zemin sensörlerinin sekizi de aynı parça ve aynı kütüphane.
- **Motorlar:** Pololu 37D 70:1. Çıkış milinde 4480 sayım/tur, 12 V'ta 150 rpm. İleri besleme katsayısı 15 rad/s.

## Ne var
- `lib/luna_core/`: Arduino'ya bağlı olmayan, saf C++ çekirdek. Hepsi bilgisayarda test ediliyor.
  - `daly_bms.*`: Daly BMS ile UART üzerinden konuşur.
    - İstek çerçevesi her komut için aynı biçimde: `A5 40 <komut> 08 00×8 <toplam>`. Örneğin 0x90 için toplam `7D`.
    - Okunan komutlar:
      - 0x90: gerilim, akım, SoC
      - 0x92: en sıcak ve en soğuk sensör
      - 0x93: FET durumu ve kalan Ah
      - 0x94: hücre sayısı ve çevrim
      - 0x95: hücre gerilimleri; 4 hücre iki çerçevede gelir
      - 0x98: alarm bitleri
    - Yanıtlar bayt bayt ayrıştırılır. Başlangıç baytı, BMS adresi (0x01), uzunluk ve sağlama toplamı denetlenir. Bozuk bir çerçeveden sonra, zaten okunmuş baytlar içindeki bir sonraki `A5`'ten yeniden senkron olunur.
    - `BatteryMonitor`: 3 s boyunca geçerli 0x90 çerçevesi gelmezse `present=false` olur. Öbür alanlar (sıcaklık, kalan Ah, hücreler, sağlık) yalnızca kendi yanıtları tazeyse doldurulur; değilse NaN, 0 ya da UNKNOWN kalır.
    - Sağlık (`power_supply_health`) yalnız Daly'nin 2. seviye (koruma) alarmlarından ve donanım arızalarından gelir. 1. seviye uyarılar dikkate alınmaz, çünkü gerilim, sıcaklık ve SoC zaten sayı olarak gidiyor ve AGX kendi eşiklerini uyguluyor.
  - `cliff.*`: VL53L1X kanalını metreye çevirir. Sensör yoksa, veri yolundan düşmüşse, örnek yoksa, durum≠0 ise ya da örnek 100 ms'den eskiyse **0.0** döner (0.0 = uçurum).
    - `CliffRetry`: düşen sensörü yeniden başlatma kuralı. 2 s'de en fazla bir deneme, sensör başına en fazla 5 deneme, yalnızca motorlar kapalıyken.
  - `debounce.*`, `dock_id.h`: P4 dock kimlik hattı ve `charge_enable` kirası. R1'den değişmedi (Elektrik R01).
  - `drive.*`: ileri besleme + PI hız döngüsü. Sayaç taşması sorun olmaz. Hedef ya da `pwm_cap` NaN/inf ise motorlar durur.
- `src/main.cpp`:
  - **Uçurum sensörleri:** 4 adet VL53L1X Wire1 üzerinde (SDA1 17 / SCL1 16).
    - XSHUT sırasıyla açılıp sensörler 0x40–0x43 adreslerine taşınır.
    - Kısa mod, 20 ms bütçe, 50 Hz. Zemin pencereden 60 mm aşağıda.
    - Her okumadan önce adres yoklanır. Yanıt vermeyen sensör "ölü" sayılır ve motorlar dururken yeniden başlatılır.
  - **Köşe sensörleri:** 4 adet VL53L1X Wire üzerinde, 0x30–0x33. R1'deki gibi. 16×4 ROI hâlâ ayarlanmıyor (bkz. ana README, "Engel algılama").
  - **BMS:** Serial1 9600 baud, 125 ms'de bir istek. 0x90 4 Hz, diğer beş komut sırayla, her biri 1,25 s'de bir. `cell_voltage` doluyor. `power_supply_technology` = LIFE.
  - **Agent bağlantısı ve watchdog:** R1'deki gibi. Agent yokken motorlar kapalı, şarj serbest. WDOG1 0,5 s.
  - **Executor:** 3 tutamaç (zamanlayıcı + `wheel_target` + `charge_enable`).
- `host_test/`: bilgisayarda çalışan birim testleri (ASan + UBSan açık).

## Doğrulananlar
- **Birim testleri:** `./host_test/run.sh` → **19 test, 188 kontrol, 0 hata** (Linux, GCC, `-Werror`, 25.10.2026).
  - Yeni Daly testleri: istek baytları, her komutun çözümü, hücre çerçevelerinin sırası ve eksik çerçeve, bozuk çerçeveler, gürültü ve kesik yanıttan sonra yeniden senkron, alarm → sağlık eşlemesi, alanların tazelik kuralı, BMS yokken güvenli varsayılan.
  - Sürüş testleri 4480 sayım/tura göre güncellendi.
- **Teensy 4.1 derlemesi BAŞARILI**, **0 uyarı** (`pio run -e teensy41`, PlatformIO 6.1.18, Linux, 25.10.2026, micro-ROS Humble kaynaktan derlendi, 249 s).
  - Bellek: FLASH kod 117 520 B + veri 15 544 B, RAM1 değişkenler 37 280 B. R1'den biraz küçük: servo ve VL53L4CD kütüphaneleri çıktı.

## Derleme
- Platform: `teensy@5.1.0` (Teensyduino 1.60, GCC 11.3). Neden sabit olduğu `platformio.ini`'de yazıyor.
- Kütüphaneler: micro_ros_platformio `cfee17f`, Pololu VL53L1X 1.3.1, QuadEncoder `59e6208`. STM32duino VL53L4CD kaldırıldı.
- Klasör yolunda boşluk veya ASCII dışı karakter olmamalı. micro_ros_platformio bağımlılıkları indirirken git komutu bölünüyor (R1'de görüldü). Gerekirse klasörü örneğin `~/luna_mcu` altına kopyalayıp `pio run -e teensy41` orada çalıştırın.

## Hâlâ doğrulanmayanlar (donanım gerekiyor)
- **Hiçbir şey gerçek kartta denenmedi.**
- **Daly akım işareti:** Kaynaklara göre 30000'in üstü deşarj. Bazı Daly yazılımlarında ters olabilir. Şarj cihazı takılıyken `/luna/mcu/battery` akımı **pozitif** olmalı; değilse `daly_bms.cpp`'deki tek satırdaki işaret çevrilir.
- **Daly UART seviyesi:** Teensy 5 V toleranslı değil. Bağlamadan önce BMS'in TX hattı ölçülmeli. UART portu Bluetooth modülüyle ortak, modül takılı olmamalı.
- **Daly hücre çerçevesi sayısı:** UART'ta ⌈hücre/3⌉ çerçeve bekleniyor. Bazı kartlar 16 çerçeve gönderebilir. Fazlası hata sayılır, ilk iki çerçeve yine kullanılır.
- **Halı ve zeminde VL53L1X durum kodları.** Durum 1/2 gereksiz durmalara yol açabilir; kısa mod ve 20 ms bütçe tezgâhta denenmeli.
- PID katsayıları ve ileri besleme (37D 70:1 ile).
- micro-ROS agent bağlantısı ve yeniden bağlanma.
- **Watchdog tezgâh testi:** kasıtlı bir sonsuz döngüyle 0,5 s sonra sıfırlandığı görülmeli.
- **Sıfırlama sırasında DRV_SLP:** hatta 10k aşağı çeken direnç olmalı (pins.h'te yazıyor). Olmazsa sıfırlama anında motor sürücüsü uyanık kalabilir.
- **P4 ve DOCK_INHIBIT:** R1 ile aynı devre. R2 dock'unda yay pimlerinin altta olmasıyla değişmiyor, ama tezgâhta yeniden ölçülmeli.

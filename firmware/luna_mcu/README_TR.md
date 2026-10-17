# luna_mcu — Teensy 4.1 micro-ROS yazılımı (2026-10-17)

## Ne var
- `lib/luna_core/`: Arduino'ya bağlı olmayan, saf C++ çekirdek. Hepsi Mac'te test ediliyor.
  - `jbd_bms.*`: JBD BMS ile konuşur.
    - İstek çerçevelerini üretir: 0x03 temel bilgi (`DD A5 03 00 FF FD 77`) ve 0x04 hücre voltajları (`DD A5 04 00 FF FC 77`).
    - Gelen yanıtı bayt bayt ayrıştırır. Sağlama toplamını, bitiş baytını, durum baytını ve uzunluğu denetler; bozuk veriden sonra yeniden senkron olur.
    - Çıkardıkları: voltaj, akım, kalan/nominal kapasite (→ SoC), NTC sıcaklıkları (en sıcağı `temperature` olur), koruma bayrakları (→ `power_supply_health`) ve hücre voltajları.
    - `BatteryMonitor`: 3 s boyunca geçerli 0x03 çerçevesi gelmezse `present=false` olur. Hücre voltajları yalnızca kendi çerçeveleri tazeyse eklenir; tek başlarına `present` yapmaz.
  - `cliff.*`: VL53L4CD kanalını metreye çevirir. Sensör yoksa ya da veri yolundan düşmüşse, örnek yoksa, durum≠0 ise veya örnek 100 ms'den eskiyse **0.0** döner (0.0 = uçurum).
    - `CliffRetry`: düşen sensörü yeniden başlatma kuralı. 2 s'de en fazla bir deneme, sensör başına en fazla 5 deneme, ve yalnızca motorlar kapalıyken (başlatma ~25 ms bloklar).
  - `debounce.*`: dock voltajı için eşik + histerezis + bekleme. 15,0 V'ta açılır, 14,5 V altında kapanır; her iki geçiş için de değer 100 ms boyunca karşı tarafta kalmalı.
  - `drive.*`: ileri besleme + PI hız döngüsü. Hız 32-bit sayaç farkından hesaplanır, sayaç taşması sorun olmaz. Hedef veya `pwm_cap` NaN/inf ise motorlar durur.
- `src/main.cpp`:
  - **Uçurum sensörleri:** 4 adet VL53L4CD Wire1 üzerinde (SDA1 17 / SCL1 16). XSHUT sırasıyla açılıp 0x40–0x43 adreslerine taşınır. Okuma ve `/luna/mcu/cliff` yayını 50 Hz. Düşen sensör, motorlar dururken yeniden başlatılmayı dener.
  - **BMS:** Serial1 üzerinden (9600 baud) 0x03 ve 0x04 sırayla sorgulanır, her biri 2 Hz. `BatteryState.cell_voltage` doluyor. Saat agent ile senkronsa (`rmw_uros_sync_session`, 60 s'de bir) `header.stamp` da doluyor.
  - **Agent bağlantısı:** micro-ROS'un yeniden bağlanma kalıbı kullanılıyor (WAITING → AVAILABLE → CONNECTED → DISCONNECTED).
    - Agent'a 500 ms'de bir ping atılır.
    - Agent yokken motorlar kapalı tutulur. Agent geri gelince bütün ROS varlıkları yeniden kurulur.
  - **Watchdog:** donanım watchdog'u (WDOG1, 0,5 s).
    - Bağlıyken yalnızca 10 ms'lik kontrol zamanlayıcısı besler.
    - Bağlı değilken `loop()` besler; bu sırada motorlar zaten kapalı.
    - Döngü takılırsa Teensy sıfırlanır.
  - **Dock algılama:** `/luna/mcu/dock_sense` artık debounce'lu (100 Hz örnekleme, 50 Hz yayın).
  - **Uyarılar:** `rcl_publish` dönüş değerleri artık sayılıyor; derleme uyarısı kalmadı.
- `host_test/`: Mac'te çalışan birim testleri (ASan + UBSan açık).

## Doğrulananlar
- **Birim testleri:** `./host_test/run.sh` → 17 test, 219 kontrol, 0 hata (Apple clang, `-Werror`).
- **Teensy 4.1 derlemesi BAŞARILI**, kendi kodumuzda **0 uyarı** (`pio run -e teensy41`, 2026-10-17).
  - Bellek: FLASH kod 121 KB + veri 15 KB, RAM1 değişkenler 38 KB.
  - Platform `teensy@5.1.0` (Teensyduino 1.60, GCC 11.3) olarak sabitlendi. `teensy` 5.2.0 ve sonrası GCC 15 ile geliyor; micro-ROS Humble `rcutils` bu sürümle derlenmiyor (`clock_gettime` örtük bildirim hatası).
  - Kütüphaneler: micro_ros_platformio `cfee17f`, VL53L1X 1.3.1, STM32duino VL53L4CD 1.0.5, QuadEncoder `59e6208`.
  - Derleme ASCII karakterli geçici bir kopyada yapıldı. **Bu klasörün kendi yolunda derleme başarısız** (2026-10-17 denendi): micro_ros_platformio bağımlılıkları indirirken `ament_cmake clone failed … Çok fazla argüman` hatası veriyor; yoldaki boşluklar (`Mary’nin Klasörü`, `Luna Robot`, `Yazılım R01 …`) git komutunu bölüyor. Derlemek için `firmware/luna_mcu` klasörünü boşluksuz ve ASCII bir yola kopyalayın (örn. `~/luna_mcu`) ve `pio run -e teensy41` orada çalıştırın.
- **Enkoder pinleri:** 30/31 ve 36/37, QuadEncoder kütüphanesinin `ARDUINO_TEENSY41` tablosunda var; XBAR girişleri (23/22, 16/17) çakışmıyor. Bu yüzden `pins.h` değişmedi.
- **Güvenli varsayılanlar:** BMS yokken `present=false` ve **voltage=0.0** gönderilir, NaN asla gönderilmez.
  - AGX tarafında `kinematics.pwm_cap(NaN)` artık 0 döndürüyor (`luna_ws/.../core/kinematics.py`, test eklendi; AGX testleri 41/41 geçiyor).
  - Firmware de NaN `pwm_cap` gelirse duruyor.

## Tasarım kararları
- **Neden WDT_T4 kütüphanesi değil:** Kütüphanenin WDT3 yolunda pencere değeri hiç sıfırlanamıyor (en az 1/32 s). Pencere açılmadan, yani 10 ms'de bir besleme hemen sıfırlamaya yol açardı. Bunun yerine WDOG1 doğrudan yazmaçlarla kuruldu. Bu watchdog'un en kısa süresi 0,5 s.
- **Sensörü yeniden başlatırken alınan önlemler:** Önce diğer ölü sensörlerin XSHUT'u LOW çekiliyor; böylece 0x29'da yalnızca yeniden başlatılan sensör bulunuyor. Sürücü nesnesi de sıfırdan kuruluyor. Aksi halde `InitSensor`, adres zaten verilmiş sanıp adres atamayı atlar ve olmayan adresten okumaya çalışırken sonsuza kadar takılır.

## Bu Mac'e kurulanlar (kullanıcı onayıyla)
- pipx ile PlatformIO 6.2.0 (Python 3.12).
- Homebrew ile `cmake` ve `binutils`.
- `~/.platformio/penv` → `~/.local/pipx/venvs/platformio` sembolik bağlantısı. micro-ROS betiği bu yolu arıyor.

## Hâlâ doğrulanmayanlar (donanım gerekiyor)
- **Hiçbir şey gerçek kartta denenmedi.** Tezgâhta ölçülmesi gerekenler:
  - halı ve zeminde VL53L4CD durum kodları (durum 1/2 gereksiz durmalara yol açabilir);
  - JBD UART seviyesi (Teensy 5 V toleranslı değil) ve gerçek çerçeveler;
  - PID katsayıları;
  - micro-ROS agent bağlantısı ve yeniden bağlanma.
- **Watchdog tezgâh testi:** Kasıtlı bir sonsuz döngüyle 0,5 s sonra sıfırlandığı görülmeli.
- **Sıfırlama sırasında DRV_SLP:** Teensy sıfırlanırken pinler sürülmez. `DRV_SLP` hattında aşağı çeken bir direnç olduğu doğrulanmalı (yoksa ~10k ile GND'ye eklenmeli); aksi halde sıfırlama anında motor sürücüsü uyanık kalabilir.
- **Dock eşiği:** 15 V eşiği iskeletten geliyor. Sabit akımla şarj sırasında kontak voltajı 15 V'un altına düşüyorsa `dock_sense` yanlışlıkla false olur. Bu voltaj ölçülmeli.
- stm32duino sürücüsünün NACK'te sonsuz tekrar denemesi tamamen kalkmadı: yoklama ile okuma arasında sensör düşerse döngü takılır. Bu durumda artık watchdog Teensy'yi sıfırlıyor.

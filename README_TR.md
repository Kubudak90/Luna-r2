# Luna robot yazılımı · R01 (16.10.2026)

İlk robot yazılım iskeleti. Robot tarafında ROS 2 Humble, Teensy 4.1 tarafında micro-ROS, yüz tarafında tarayıcı adaptörü var. Donanım henüz alınmadığı için yığın, **sahte MCU** (`mock_mcu_node`) ile donanımsız çalışacak şekilde kuruldu. Gerçek kart geldiğinde aynı topic'ler Teensy'den gelir.

## Bu ortamda doğrulananlar

| Kontrol | Sonuç |
|---|---|
| Çekirdek mantık birim testleri: kinematik, kafa, güvenlik kapısı, batarya politikası, dock durum makinesi, yüz eşlemesi | **20/20 geçti** (`python3 -m unittest discover -s test`) |
| Yüz köprüsünün SSE sunucusu, gerçek HTTP bağlantısıyla uçtan uca (rclpy taklit edildi) | **2/2 geçti** |
| Tarayıcı adaptörü, **gerçek** Telefon Yüzü R01 çekirdeğiyle: çarpma → şaşırma, dock → uyku, ayrılma → uyanma | **3/3 geçti** (`cd web && npm test`) |
| Bütün Python dosyalarının sözdizimi, URDF ve package.xml | geçti |

## Henüz doğrulanmayanlar

- **ROS 2 yığını gerçekten çalıştırılmadı.** Bu Mac'te ROS kurulu değil; `colcon build` ve `ros2 launch` denenmedi.
- **Docker imajı kurulmadı.** Jetson ya da arm64 L4T ortamı gerekiyor.
- **Teensy yazılımı derlenmedi.** PlatformIO ve micro-ROS araç zinciri burada yok.
- **Hiçbir gerçek donanımla denenmedi.**

## Yapı

```
luna_ws/src/luna_robot/
  luna_robot/core/      ROS'tan bağımsız, test edilen mantık
    kinematics.py         paletli (skid-steer) kinematik, odometri, 12 V motor için PWM sınırı
    head.py               pan ±60° / tilt ±12° → servo µs, hız sınırı, yüz için bakış
    safety.py             acil durdurma, tampon, uçurum, komut zaman aşımı kapısı
    battery.py            4S3P SoC tahmini; dock arama / kritik / şarj kararları ve AGX güç modu
    dock.py               SEEK → ALIGN → APPROACH → CONTACT → CHARGING → DOCKED; geri çekilme ve 3 deneme
    face_mood.py          robot olayları → yüz olayları (tap/stroke/curious/sleep/wake/gaze)
  luna_robot/*_node.py  ince ROS 2 düğümleri: base, head, dock, face_bridge, mock_mcu
  launch/luna.launch.py   mock:=true (varsayılan) | mock:=false lidar:=true
  config/luna.yaml        CALIBRATE işaretli değerler gerçek robotta ölçülecek
  urdf/luna.urdf          R05 CAD'den sadeleştirilmiş çerçeveler
firmware/luna_mcu/       Teensy 4.1 micro-ROS iskeleti (pins.h pin haritası)
web/robot-link.js        yüz PWA'sını robota bağlayan EventSource adaptörü
docker/                  dustynv/ros:humble-ros-base-l4t-r35.4.1 tabanlı imaj
```

## Topic sözleşmesi

Teensy ile AGX arasındaki topic'lerin tek kaynağı `luna_robot/topics.py`. `firmware/luna_mcu/src/topics.h` bu dosyayla birebir aynı tutulmalı.

| Yön | Topic | Tip |
|---|---|---|
| MCU → | `/luna/mcu/wheels` | Float32MultiArray [sol rad, sağ rad, sol rad/s, sağ rad/s] |
| MCU → | `/luna/mcu/bumpers` · `/estop` · `/dock_sense` | UInt8 bit maskesi · Bool · Bool (robot beslenen dock'a oturdu, P4 kimliği) |
| MCU → | `/luna/mcu/cliff` · `/corners` | Float32MultiArray (m) |
| MCU → | `/luna/mcu/battery` | sensor_msgs/BatteryState (BMS UART) |
| → MCU | `/luna/mcu/wheel_target` | [sol rad/s, sağ rad/s, pwm_cap] |
| → MCU | `/luna/mcu/head_us` | [pan µs, tilt µs] |
| → MCU | `/luna/mcu/charge_enable` | Bool: false = DOCK_INHIBIT, dock kontakları söndürür. `false` 1 s geçerli; `dock_node` 20 Hz yayımlar. Mesaj yoksa şarj serbest |
| AGX | `/cmd_vel`, `/luna/dock/request` (dock/undock/cancel), `/luna/head/target` | |

Hız önceliği: dock komutu teleop'tan önce gelir. Her iki komut da güvenlik kapısından geçer. Kapı açılışta kapalıdır: MCU'dan ilk mesaj gelene kadar acil durdurma basılı sayılır.

## Güvenlik katmanları

1. **Donanım:** acil durdurma kontağı motor sürücünün SLP hattını ve servo regülatörlerinin EN hattını doğrudan keser.
2. **Teensy:** acil durdurma basılıysa, sürücü hata (FLT) pini düşükse ya da 300 ms boyunca hız komutu gelmezse motorlar durur.
3. **AGX güvenlik kapısı:**
   - Ön tampon basılıyken ileri, arka tampon basılıyken geri gidilmez.
   - Bir uçurum algılanırsa o yöne gidilmez.
   - Hem önde hem arkada zemin yoksa (robot kaldırılmış olabilir) her hareket durur.
4. **PWM sınırı:** pwm_cap = 12 V / paket gerilimi. 16.8 V'luk dolu pakette 12 V'luk motor anma geriliminin üstüne çıkılmaz.

**Firmware'de iki iş bilerek eksik bırakıldı:**
- **VL53L4CD zemin sensörleri:** okunana kadar firmware zemini 0 m bildirir. Güvenlik kapısı bunu "zemin yok" sayar ve robot yürümez.
- **JBD BMS ayrıştırması:** yazılana kadar batarya gerilimi 0 V bildirilir, bu yüzden pwm_cap = 0 olur.

İki iş tamamlanmadan gerçek robot hareket etmez; yarım firmware'le kaza olmasın diye böyle bırakıldı.

## Çalıştırma (Jetson üzerinde, Docker ile)

```bash
docker compose -f docker/compose.yaml --profile mock up --build luna-mock
```

Mock koşusunda başlangıç SoC'u %24. Bu, %25'lik dock eşiğinin altında, yani robot kendiliğinden 1.2 m ilerideki dock'u arar:
- AprilTag görünümü taklit edilir; robot hizalanır, yaklaşır ve son 60 mm'de sürünerek girer.
- Kontak oluşunca şarj başlar ve yüz uyur.
- Yüz olayları `http://127.0.0.1:8765/events` adresinden akar.

Telefon yüzünü robota bağlamak için `web/robot-link.js` dosyası yüz PWA'sına eklenir:

```js
import { connectRobot } from './robot-link.js';
connectRobot(face, { url: 'http://127.0.0.1:8765/events' });
```

Telefon Yüzü R01 klasörü değiştirilmedi.

## Bilinen sınırlar ve sıradaki adımlar

- **ROS 2 sürümü:** Xavier'de ROS 2 Humble yalnız Tier 3 düzeyinde destekleniyor ve konteynerde çalışıyor. Jazzy için Xavier'e hazır imaj yok.
- **AprilTag düğümü:** bu pakette yok. `apriltag_ros` bağlanıp etiket ölçümü `/luna/dock/tag` topic'ine (PointStamped: x = yatay sapma, z = mesafe) yayımlanacak.
- **SLAM ve navigasyon:** bağlanmadı. Sıradaki adaylar slam_toolbox ve Nav2; LiDAR verisi `laser` çerçevesinde hazır.
- **USB kameralar:** Arducam UVC kameralar için `usb_cam` ya da `v4l2_camera` eklenecek.
- **Ses:** XVF3800'ün ses ve yön bilgisi bağlanmadı; yüzdeki `gaze` olayı buna da bağlanabilir.
- **Ağır yapay zekâ işleri:** bulut Kimi 2.8 için ağ istemcisi henüz yok. Wi-Fi kartı donanım listesinde (K05).
- **Kalibre edilecekler** (`config/luna.yaml`):
  - skid-steer kayma katsayısı
  - servoların gerçek orta noktası
  - zemin sensörünün düz zemindeki okuması
  - PID kazançları
  - motor ileri besleme katsayısı
- **Kablolama eşlemesi:** CAD'deki `_L` parçalar robotun kendi **sağ** tarafındadır (CAD'de ön −Y). Motor kanal 1, robotun sol paleti, yani CAD `REF_MOTOR_R`. Ayrıntı `urdf/luna.urdf` ve `pins.h` içinde.

## Docker doğrulaması (16.10.2026)

Bu bölüm yukarıdaki "Henüz doğrulanmayanlar" listesinin ilk maddesini günceller: ROS 2 yığını artık derlenip mock ile çalıştırıldı.

**Ortam:** Mac (Apple Silicon), colima VM ve resmi `ros:humble-ros-base` arm64 imajı (Ubuntu 22.04, Python 3.10). Bu Mac'te Docker Desktop açılmadı. Paket boş bir `/ws` çalışma alanına kopyalanıp orada derlendi. Jetson imajı (`docker/Dockerfile`, dustynv L4T) **kurulmadı**.

| Kontrol | Sonuç |
|---|---|
| `rosdep check`, `colcon build` | geçti; bütün bağımlılıklar imajda var |
| `colcon test` (pytest) ve `python3 -m unittest discover -s test` | **23/23 geçti** (konteynerde Py 3.10, Mac'te Py 3.14) |
| `cd web && npm test` (Mac) | **3/3 geçti** |
| `ros2 launch luna_robot luna.launch.py` (mock:=true) | 6 süreç başladı; robot_state_publisher URDF'deki 11 parçayı okudu |
| `/luna/dock/state` | ALIGN → APPROACH (0.4 s) → CONTACT (22.1 s) → **CHARGING** (23.1 s); 40 s sonunda hâlâ CHARGING |
| `/luna/power/action` | unknown → seek_dock → charge |
| `/odom` | x: 0 → **1.1999 m** |
| `curl -N http://127.0.0.1:8765/events` (konteyner içinden) | `curious`, `stroke`, **`sleep`** |
| Ctrl-C ile kapanış | 6/6 süreç temiz kapanıyor (çıkış kodu 0), terminalde de terminalsiz çalıştırmada da. Ayrıntı aşağıda, 2. hata |

SEEK durumu topic'te görünmez: etiket başlangıçta zaten görüş alanında olduğundan dock düğümü aynı tikte ALIGN'a geçer. Etiket görünmediğinde SEEK yayımlanıyor; düzeltmeden önceki koşuda geri çekilmeden sonra görüldü.

**Bulunan ve düzeltilen hatalar**
1. **Mock'ta dock kontağı titriyordu, robot şarja hiç geçmiyordu.** Durum CONTACT'e giriyor, 50 ms sonra BACKOFF'a düşüyor, üç denemeden sonra FAULT'ta kalıyordu; `sleep` olayı da hiç gelmedi. Nedeni şuydu: mock, kontağı yalnız burun duvarın içine geçtiği tikte (`dist <= 0`) bildiriyordu. Duvar modeli ise robotu bir önceki konuma geri itiyordu, bu yüzden DOCK_SENSE bir tik true, sonra yine false oluyordu. Düzeltme:
   - Mock'un fizik modeli saf `core/mock_plant.py` dosyasına taşındı; `mock_mcu_node` artık ince bir sarmalayıcı.
   - Yaylı kontaklar, sert duruştan 3 mm önce değer (`contact_travel_m`).
   - Yeni test `DockingLoop`, gerçek `Dock` durum makinesini bu modelle kapalı döngüde koşturuyor. Test düzeltmeden önce kalıyor, sonra geçiyor.
2. **Ctrl-C'de düğümler traceback basıp ölüyordu**, launch da bunu "process has died" diye bildiriyordu. Üç adımda düzeltildi.
   - Önce sinyallerin yolu: terminalde Ctrl-C, SIGINT'i ön plandaki süreç grubunun tamamına, yani launch'la birlikte bütün düğümlere gönderir. stdin terminal değilse launch her düğüme bir SIGINT daha yollar; terminalde yollamaz (`ros2 launch --noninteractive`'in varsayılanı `not sys.stdin.isatty()`). Terminalsiz çalıştırma demek: `-t`'siz `docker run`, betikler, servisler. Aşağıda "grup SIGINT", launch terminalsizken bütün gruba SIGINT göndermek demek; her düğüm iki SIGINT alır. "pty Ctrl-C" ise launch'ı bir pty'de çalıştırıp pty'ye `\x03` yazmak; gerçek terminaldeki gibi her süreç tek SIGINT alır.
   - **1. adım (16.10.2026):** yeni `luna_robot/entry.py` dosyasındaki ortak `run()`. Kapanış sinyallerini ve kapanış sırasında yayınla yarışan RCLError'u yutar; çalışırken çıkan gerçek hataları yine yükseltir. Eski kodla yapılan 25 kapanışta 125 düğüm çıkışının 121'i 1 koduyla bitti. Yeni kodla ilk 7 kapanışta bir kez bir düğüm askıda kaldı, bir kez de `mock_mcu_node` segfault (-11) verdi. Sonraki 67 kapanışın hepsi temiz bitti. Bu iki olayın nedeni bulunamadı; Humble'daki rclpy kapanış yarışlarından biri olabilir. Gerçek terminalde Ctrl-C bu adımdan sonra temizdi: 17.10.2026'da pty Ctrl-C ile 10 kapanışın hepsinde 6/6.
   - **2. adım (17.10.2026), Python düğümleri:** grup SIGINT'te 4 kapanışın hepsinde 5 Python düğümü de traceback basıp -2 ile ölüyordu. İkinci SIGINT `run()`'ın `finally` bloğundaki temizlik sırasında gelince KeyboardInterrupt yakalanmıyordu. Düzeltme: `entry.run()` artık `rclpy.init()`'ten önce Python'un KeyboardInterrupt'ını kapatıyor (`SIG_IGN`). Ctrl-C'yi rclpy'nin kendi sinyal işleyicisi karşılıyor: context'i kapatıyor, `spin()` de `ExternalShutdownException` ile dönüyor. Yeni test: `test/test_entry.py`.
     - Yalnız `finally`'yi korumak yetmedi, çünkü KeyboardInterrupt rastgele yerlerde patlıyordu. Bir kez `spin()` RCLError ile bittikten sonra, hata gerçek mi diye bakılırken geldi (`dock_node`, 4 kapanışta 1). Bir kez de rclpy'nin mesaj dönüştürmesinin içinde patladı; `RuntimeError: Unable to convert call argument...` olarak çıkıp düğümü 1 koduyla düşürdü (`mock_mcu_node`, 19 kapanışta 1).
     - Yok sayma bir Python fonksiyonuyla değil `SIG_IGN` ile yapılıyor; `finally` de rclpy'nin işleyicisinin yerine yeniden `SIG_IGN` koyuyor. Python, yorumlayıcı kapanırken Python işleyicilerini SIG_DFL'e çeviriyor. O sırada gelen SIGINT, DDS nesneleri yok edilirken düğümü traceback'siz -2 ile öldürüyordu (Python no-op işleyicili ara sürümde 19 kapanışta 16 Python düğümü).
     - 17.10.2026'da Docker'da: `colcon test` ve `python3 -m unittest discover -s test` **41/41** geçti (Py 3.10). Grup SIGINT'le 24 kapanışta (canlı koşunun sonu dahil) 120 Python düğüm çıkışının hepsi temiz bitti; traceback, askıda kalma ya da segfault görülmedi. SIGINT yalnız `ros2 launch`'a gönderildiğinde 3 kapanışın, pty Ctrl-C'de 20 kapanışın hepsinde 6/6 süreç temiz kapandı.
     - Bedeli: bir callback içinde takılı kalmış düğüm artık Ctrl-C ile kesilmiyor. launch onu sırayla SIGTERM ve SIGKILL ile en geç ~10 s'de kapatır; elle çalıştırılan düğüm için Ctrl-\ kullanılır.
   - **3. adım (17.10.2026), `robot_state_publisher`:** C++ RSP grup SIGINT'te ara sıra -2 ile ölüyordu (24 kapanışta 5, sonraki 20 kapanışta 2); pty Ctrl-C'de 20 kapanışta hiç ölmedi. rclcpp kapanırken kendi SIGINT işleyicisini kaldırıp eskisini (SIG_DFL) geri koyuyor; launch'ın ikinci SIGINT'i de süreci öldürüyordu. Düzeltme: `luna.launch.py` RSP'yi `sh -c 'trap "" INT; exec "$0" "$@"'` önekiyle başlatıyor (`IGNORE_LATE_SIGINT`). Süreç SIGINT'i yok sayarak açılıyor; rclcpp kendi işleyicisini yine üstüne kuruyor, kapanınca da yerine SIG_IGN'i koyuyor. Böylece geç gelen ikinci SIGINT zararsız kalıyor. `sh` kendini `exec` ile RSP'ye bıraktığı için pid, süreç grubu ve log etiketi (`exec_name`) aynı kalıyor.
     - 17.10.2026'da Docker'da yeni dosyayla 6/6 süreç şu kapanışların hepsinde temiz bitti: grup SIGINT 20/20, pty Ctrl-C 10/10, yalnız launch'a SIGINT 3/3. Aynı önekle başlatılan ve SIGINT'i hiç dinlemeyen bir test süreci ~5 s sonra launch'tan SIGTERM aldı, yani kademeli kapatma çalışıyor. `unittest` ve `colcon test` 41/41 geçti.
     - Bedeli: RSP, rclcpp işleyiciyi kurmadan önceki (açılış) ve kaldırdıktan sonraki (son temizlik) kısa anlarda SIGINT'e tepki vermez. O anlarda takılırsa launch ~5 s sonra SIGTERM, ardından SIGKILL yollar. Açık `prefix` verildiği için `launch-prefix` (ör. gdb) bu düğüme artık uygulanmaz.
     - `setsid` öneki denendi ve seçilmedi. RSP kendi oturumuna geçince terminaldeki Ctrl-C ona hiç ulaşmıyor; launch da terminalde sinyal yollamadığı için her Ctrl-C'de 5 s bekleyip RSP'yi SIGTERM ile kapattı (3/3). launch'ın süreç grubuna SIGKILL gidince de RSP sahipsiz kaldı.
     - Önek, `mock:=false`'taki `micro_ros_agent`'a ve lidar sürücüsüne uygulanmadı; ikisi bu ortamda yok, denenmedi.
   - **Açık:**
     - pty Ctrl-C ile yapılan 40 kapanışın birinde `face_bridge_node` tek SIGINT'le 5 s içinde çıkmadı; launch SIGTERM yolladı, düğüm ardından temiz kapandı. Nedeni araştırılmadı.
     - `ros2 launch`'a yalnız SIGTERM gönderilirse Humble launch düğümlere sinyal yollamadan çıkıyor, düğümler sahipsiz kalıyor (launch'ın kendi uyarısı: "can result in orphaned processes"). Ardından yapılan başlatmada `face_bridge_node` 8765 portu dolu diye düştü. Servis olarak çalıştırırken sinyal bütün süreç grubuna gitmeli; systemd'de `KillSignal=SIGINT` ve varsayılan `KillMode=control-group` bunu yapar (denenmedi).

**Doğrulanmayanlar**
- Jetson/L4T imajı ve GPU tarafı; komutlar aynı, ama yalnız resmi arm64 imajında denendi.
- `mock:=false` (`micro_ros_agent` ve Teensy yok) ile `lidar:=true` (`sllidar_ros2` yok).
- Tarayıcıdaki yüz canlı akışa bağlanmadı; akış yalnız `curl` ile okundu.

**Gözlemler**
- 1.2 m'lik yaklaşma yaklaşık 21.7 s sürüyor, `approach_timeout` ise 25 s. Etiketi yaklaşık 1.4 m'den uzakta yakalayan robot, yaklaşmayı bitiremeden geri çekilir. Nav2 robotu dock'un yaklaşık 1 m önüne getirecekse bu yeterli.
- Dock durum makinesi, CONTACT ya da CHARGING'de tek bir DOCK_SENSE=false örneğinde geri çekiliyordu; firmware bu sinyali debounce etmeden gönderiyor. **Sonradan düzeltildi:** artık 0.2 s kesintisiz kontak kaybından sonra geri çekiliyor (`sense_loss_s`). Firmware değişmedi.
- Yüz akışı bağlanan istemciye mevcut durumu göndermiyordu; robot zaten şarjdayken bağlanan yüz `sleep` almıyordu. **Sonradan düzeltildi:** yeni ya da yeniden bağlanan istemci, canlı olaylardan önce mevcut durumu alıyor (`MoodMapper.snapshot`). Robot dock'taysa `sleep` gider. Değilse önce `wake`, bakış sıfır değilse ardından `gaze` gider. `wake`, bağlantı koparken uykuda kalmış yüzü uyandırır; uyanık yüzde yalnız boşta kalma sayacını sıfırlar.
- Bu iki düzeltme 17.10.2026'da Docker'da yeniden koşuldu. `colcon test` ve `python3 -m unittest discover -s test` **35/35** geçti (Py 3.10). Canlı koşuda robot ~23 s'de CHARGING'e geçti. O sırada bağlanan `curl`, `retry:` satırının hemen ardından `sleep` aldı. Dock'tan ayrılma isteğinden sonra yeniden bağlanan `curl` ise `wake` aldı.

**Tekrarlamak için** (Mac, depo kökü "Yazılım R01 2026-10-16"):

```bash
colima start
docker --context colima run --rm -it -v "$PWD/luna_ws/src/luna_robot:/src/luna_robot:ro" ros:humble-ros-base bash
```

Konteynerde:

```bash
source /opt/ros/humble/setup.bash && mkdir -p /ws/src && cp -r /src/luna_robot /ws/src/ && cd /ws
colcon build && colcon test && colcon test-result --verbose
source install/setup.bash && ros2 launch luna_robot luna.launch.py
```

Docker Desktop bozukken `docker pull`, kimlik yardımcısında (`credsStore`) takılabilir. O durumda imaj, içinde yalnız `{}` olan bir `config.json` bulunan boş bir dizinle çekilir: `DOCKER_CONFIG=<o dizin> DOCKER_HOST=unix://$HOME/.colima/default/docker.sock docker pull ros:humble-ros-base`.

## Dock kimliği (P4) ve şarj izni (17.10.2026)

Elektrik R01 (`../Elektrik R01 2026-10-17/README_TR.md` §3 ve §5) dock algılamayı kontak geriliminden P4 kimlik hattına taşıdı. Bu depoya iki parça halinde girdi.

**1. Firmware yaması** (`firmware_yamasi.diff`, olduğu gibi uygulandı):
- `lib/luna_core/src/dock_id.h`: P4 penceresi. Robot oturunca P4 2.50 V, inhibit'te 0.76 V.
- `pins.h`: `DOCK_ID_ADC = 41`, `DOCK_INHIBIT = 35`.
- `main.cpp`: `dock_sense` P4 skoruyla, 100 ms debounce'la hesaplanıyor.
- `/luna/mcu/dock_sense` yine Bool; anlamı artık "robot beslenen dock'a oturdu".

**2. Faz 2: şarj izni.**
- **Topic:** yeni `/luna/mcu/charge_enable` (Bool, AGX → MCU), `topics.py` ve `topics.h`'te.
- **Firmware:** `DOCK_INHIBIT = !data` uygular. `false` yalnız 1 s geçerli; agent koparsa pin hemen LOW olur. Varsayılan her durumda "şarj serbest".
- **`core/dock.py`:** `Dock.charge_enable`, UNDOCK ve BACKOFF'ta false. Bu iki durumda robot önce 0.25 s yerinde durur (`release_s`), sonra eskisi gibi 2.5 s geri gider. Böylece dock kontakları akım altında ayrılmadan önce söner. BACKOFF da dahil, çünkü CONTACT'ta şarj akımı görülmezse (ör. BMS yok) robot oturmuş ve kontaklar enerjili hâlde geri çekilir.
- **`dock_node`:** değeri her tikte yayımlar.
- **Mock:** `mock_mcu_node` aboneliği `DockPlant.charge_enable`'a bağlar; `false`'ta şarj akımı keser. Gerçek firmware'deki 1 s sınırı mock'ta yok, son değer kalır.

**Doğrulama (17.10.2026):**

| Kontrol | Sonuç |
|---|---|
| `firmware/luna_mcu/host_test/run.sh` | **19 test / 246 kontrol** (yama sonrası 18/235, ardından `charge_enable lease` testi) |
| `pio run -e teensy41` (ASCII scratch kopya) | **SUCCESS**, kendi kodumuzda uyarı yok; FLASH kodu 121 320 B, RAM1 değişken 38 048 B |
| `PYTHONPATH=. python3 -m unittest discover -s test` (Mac, Py 3.14) | **41/41** (iki eski undock testi yerine iki yeni test) |
| Docker `ros:humble-ros-base`, `colcon test` (Py 3.10) | **41/41** |
| Docker canlı mock koşusu (CHARGING'den 3 s sonra `undock`) | `charge_enable` false ile şarj akımı aynı tikte kesildi; ilk geri tekerlek hızı **0.22 s** sonra; IDLE'da `charge_enable` yeniden true. `charge_enable` ~19.5 Hz geldi |

Yeni testlerin ikisi de iki bozuk sürümde düşüyor: `charge_enable` hep true iken ve `release_s = 0` iken.

**Gözlem (değiştirilmedi):** Mock SoC'u %24 olduğu için robot IDLE'a geçer geçmez `auto_dock` yüzünden yeniden dock'a yöneliyor. Düşük SoC'de kullanıcının `undock` isteği, pil politikası tarafından hemen geri alınıyor.

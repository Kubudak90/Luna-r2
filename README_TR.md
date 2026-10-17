# Luna robot yazılımı · R01 (16.10.2026)

İlk robot yazılım iskeleti. Robot tarafında ROS 2 Humble, Teensy 4.1 tarafında micro-ROS, yüz tarafında tarayıcı adaptörü var. Donanım henüz alınmadığı için yığın, **sahte MCU** (`mock_mcu_node`) ile donanımsız çalışacak şekilde kuruldu. Gerçek kart geldiğinde aynı topic'ler Teensy'den gelir.

## Bu ortamda doğrulananlar

| Kontrol | Sonuç |
|---|---|
| Çekirdek mantık birim testleri: kinematik, kafa, güvenlik kapısı, batarya politikası, dock durum makinesi, yüz eşlemesi | **20/20 geçti** (`python3 -m unittest discover -s test`) |
| Yüz köprüsünün SSE sunucusu, gerçek HTTP bağlantısıyla uçtan uca (rclpy taklit edildi) | **2/2 geçti** |
| Tarayıcı adaptörü, **gerçek** Telefon Yüzü R01 çekirdeğiyle: çarpma → şaşırma, dock → uyku, ayrılma → uyanma | **3/3 geçti** (`cd web && npm test`) |
| Bütün Python dosyalarının sözdizimi, URDF ve package.xml | geçti |
| Kafa kameraları ve AprilTag ile dock (17.10.2026, dal `kamera-apriltag`): sentetik görüntü taraması ve `tags:=sim` tam yığın | geçti; gerçek kamerayla denenmedi. Ayrıntı: "Kamera ve AprilTag ile dock" |

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
    dock_tag.py           dock AprilTag'inin base_link konumu → /luna/dock/tag (offset, mesafe); iki kamera birleştirme
    frames.py             dönüşümler + URDF eklem zinciri (TF'in ROS'suz eşi, testler ve sim için)
    camera_model.py       B0630 pinhole modeli (1280×720, 82°)
    tag36h11.py           tag36h11 bit desenleri (AprilRobotics kütüphanesiyle aynı yerleşim)
    tag_scene.py          sentetik kamera görüntüsü (numpy; sim kamera ve tarama)
  luna_robot/*_node.py  ince ROS 2 düğümleri: base, head, dock, dock_tag, face_bridge, mock_mcu, sim_camera
  launch/luna.launch.py   mock:=true (varsayılan) | mock:=false lidar:=true; tags:=mock|sim|usb
  config/luna.yaml        CALIBRATE işaretli değerler gerçek robotta ölçülecek
  config/cameras.yaml     usb_cam, iki Arducam B0630 (1280×720 MJPG, 15 fps)
  config/apriltag.yaml    apriltag_ros, tag36h11 ID 0, 48 mm
  config/camera_info/     NOMİNAL kamera iç parametreleri (kalibre edilecek)
  urdf/luna.urdf          R05 CAD'den sadeleştirilmiş çerçeveler; kameralar R06 modelinden, optik çerçeveler dahil
  tools/                  tag_sweep.py (sentetik görüntü taraması), dock_watch.py, make_tag_svg.py
docs/dock_tag36h11_id0_48mm.svg  dock etiketi baskı dosyası (%100 ölçek)
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
| AGX | `/cmd_vel`, `/luna/dock/request` (dock/undock/cancel), `/luna/head/target` (pan + sola, tilt + aşağı) | |
| AGX | `/luna/dock/tag` | PointStamped: x = etiketin gövdeye göre yönü (sağ +, ±35° = ±1), z = ağız–burun mesafesi (m) |
| AGX | `/joint_states` | head_node → robot_state_publisher: head_pan, head_tilt (rad); TF kameraları kafayla taşır |

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
- **AprilTag düğümü:** eklendi (`kamera-apriltag` dalı); ayrıntı aşağıda "Kamera ve AprilTag ile dock" bölümünde.
- **SLAM ve navigasyon:** bağlanmadı. Sıradaki adaylar slam_toolbox ve Nav2; LiDAR verisi `laser` çerçevesinde hazır.
- **USB kameralar:** `usb_cam` ile yapılandırıldı (`config/cameras.yaml`); gerçek kamerayla denenmedi.
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

## Kamera ve AprilTag ile dock (17.10.2026, dal `kamera-apriltag`)

Bu bölüm, mock'un sahte etiketinin yerine gerçek bir etiket hattı koyar: iki kafa kamerası → `apriltag_ros` → `dock_tag_node` → `/luna/dock/tag` → `dock_node`. Kafa döndüğünde de çalışır, çünkü etiket TF ile gövdeye (`base_link`) çevrilir.

### Etiket

- **Aile ve kimlik:** tag36h11, ID 0.
- **Baskı ölçüsü:** siyah kare **48 mm** (8 hücre × 6 mm). Bir hücrelik beyaz halkayla birlikte 60 mm eder.
- **Plaka:** REF_R02_TAG, 64 × 64 mm. Etiket ortalanınca her kenarda 2 mm kesim payı kalır.
- **Neden 48 mm:** Plakaya beyaz halkasıyla sığabilecek en büyük etiket 51.2 mm olurdu, ama o zaman kesim payı hiç kalmaz. 6 mm'lik hücre hem yazdırması kolay hem de pay bırakıyor.
- **`apriltag_ros` ayarı:** `size: 0.048`. Bu değer siyah karenin kenarıdır; beyaz halka dahil değildir.
- **Baskı dosyası:** `docs/dock_tag36h11_id0_48mm.svg`. %100 ölçekle yazdırın ve siyah karenin kenarını cetvelle ölçüp 48 mm olduğunu doğrulayın. Kesik çizgi 64 mm'lik plaka sınırıdır.
- **Desenin doğruluğu:** Bit deseni AprilRobotics `tag36h11.c` dosyasından alındı. Gerçek dedektör sentetik görüntülerde bu deseni ID 0 olarak çözdü.

### Geometri (R02 CAD, oturmuş poz, `base_link` eksenleri)

| Nokta | x | Kaynak |
|---|---|---|
| Etiket yüzü | 0.371 m, merkez yüksekliği 0.382 m | REF_R02_TAG Y−376..−371, Z350..414 |
| Alıcı ağzı | 0.170 m | Y−170 |
| Burun ucu | 0.154 m | R02_PF2_D01 Y−154; etiketin 0.217 m önünde |
| Sol / sağ kamera optik merkezi | 0.034 m, y ±0.062, z 0.475 | R06 kamera modeli; giriş göz bebeği lens önünün 3 mm gerisinde |

`/luna/dock/tag` şunları taşır:
- **`x` (offset):** etiketin gövdeye göre yönü. Dönme merkezi `base_link`'tir; sağ pozitiftir. ±35° değeri ±1'e karşılık gelir ve değer ±1'de kırpılır. Mock ve `Dock.k_turn` aynı ölçekle ayarlı olduğu için durum makinesinde değişiklik gerekmedi.
- **`z` (mesafe):** alıcı ağzından burun ucuna olan mesafe. Burun ağza değdiğinde 0 olur; robot tam oturduğunda **−0.016 m** olur, çünkü burun ağzın 16 mm içine girer. Robot dock'la aynı hizadayken bu değer kesindir; açılı yaklaşırken yaklaşıktır.
- **Reddedilen tespitler:** etiket yüksekliği 0.382 ± 0.10 m dışındaysa ya da `base_link`'e uzaklık 0.25–3.0 m aralığının dışındaysa ölçüm yayımlanmaz.

### Kameralar

- **Sürücü:** `usb_cam`, 1280×720 MJPG, 15 fps (`config/cameras.yaml`).
  - Arducam B0630, YUY2 biçimini en çok 640×480'de veriyor; 720p ve 1080p için MJPG gerekiyor. `usb_cam` MJPG'yi kendisi çözüyor (`mjpeg2rgb`, ffmpeg).
  - 15 fps yeterli: `dock_node` etiketi 0.5 s görmezse kayıp sayıyor. Düşük kare hızı AprilTag'in işlemci yükünü de yarıya indiriyor.
- **İç parametreler:** `config/camera_info/*.yaml` dosyaları **nominal**. Üreticinin verdiği 82° yatay açıdan hesaplandı: fx = 736 px, bozulma 0.
  - Lensin bozulması üreticiye göre −13 %'e kadar çıkıyor. `camera_calibration` ile kalibre edilene kadar görüntü kenarlarına yakın ölçümlere güvenilmemeli.
  - `apriltag_ros` şimdilik düzeltilmemiş (`image_raw`) görüntüyü alıyor.
- **Odak:** B0630'un M12 lensi sabit odaklı; üreticiye göre 1 m ile sonsuz arası net.
  - Dock sırasında etiket 0.37–1.4 m arasında görünüyor. Sentetik testte 3 px bulanıklıkla bile robot oturmuş pozdayken tespit edildi, ama gerçek kamerada bu denenmedi.
  - Kaba hesap (f 3.7 mm, F2.5, karışıklık dairesi 2 piksel = 5.8 µm): hiperfokal mesafe ≈ 0.94 m. Lens elle ~0.6 m'ye odaklanırsa 0.37–1.65 m arası net kalır.
- **Cihaz adları:** her kamera iki düğüm açar (`/dev/videoN` ve meta veri için N+1) ve numaralar takılma sırasına göre değişir. Kablolama bitince `config/cameras.yaml` ve `docker/compose.yaml` içinde `/dev/v4l/by-path/...` adları kullanılmalı.

### Kafa dönüşü (pan) telafisi

- `head_node` artık `/joint_states` yayımlıyor: `head_pan` ve `head_tilt` (rad). `robot_state_publisher` bunlarla TF'i günceller ve kameralar kafayla birlikte döner. Servolarda geri bildirim olmadığı için yayımlanan değer ölçüm değil, hız sınırlı komuttur.
- **Yön kuralı (URDF eksenleri):** + pan robotun soluna, + tilt aşağıya. Kalibrasyonda `pan_direction` ve `tilt_direction` buna göre ayarlanmalı. Yanlış işaret, telafiyi ters çevirir.
- `dock_tag_node` etiketi görüntünün zaman damgasında `base_link`'e çevirir. Böylece offset ve mesafe kafanın nereye baktığından bağımsız olur.
- **Sınır:** Telafi, etiket kameralardan en az birinin görüntüsünde kaldıkça çalışır. Yatay yarım açı 41°. Robot etikete dönmüşken kafa yaklaşık ±35°'ye kadar çevrilebilir; daha fazlasında hizalanırken etiket görüntüden çıkar.

### launch

```bash
ros2 launch luna_robot luna.launch.py                # tags:=mock — mock etiketi sahteler, kamera paketi gerekmez
ros2 launch luna_robot luna.launch.py tags:=sim      # sim kamera → apriltag_ros → dock_tag_node (donanımsız)
ros2 launch luna_robot luna.launch.py mock:=false    # tags:=usb — iki usb_cam → apriltag_ros → dock_tag_node
```

`tags:=sim` modunda:
- mock MCU etiketi sahtelemez, yalnız gerçek pozunu `/luna/mock/truth` topic'ine yayımlar.
- `sim_camera_node` bu pozdan ve TF'teki kafa pozundan iki kameranın 1280×720 görüntüsünü üretir: ışın izlemeyle, piksel başına 3×3 örnekle ve gürültü ekleyerek.
- Kamera başına bir `apriltag_ros` düğümü çalışır. Etiket TF çerçeveleri `dock_tag_left` ve `dock_tag_right`'tır.

### Doğrulananlar (17.10.2026)

**Ortam:** Mac (Apple Silicon), colima ve yerel `luna-dev:humble-tag` imajı.
- İmaj `docker/Dockerfile.dev` ile kuruldu: `ros:humble-ros-base` üstüne `ros-humble-apriltag-ros` 3.4.0 ve `ros-humble-usb-cam` 0.8.1. İndirme 209 MB oldu; imaj 2.15 GB, tabana göre +1.03 GB.
- Konteyner `--shm-size=512m` ve `ROS_LOCALHOST_ONLY=1` ile çalıştırıldı.
- Aşağıdaki sayılar VM boşken alındı; kardeş oturumların konteynerlerinden ROS ağı olarak da yalıtılmıştı (`ROS_LOCALHOST_ONLY=1`, ayrı domain). Kardeş konteynerler VM'yi doldurduğunda (yük ortalaması 20–27, 4 vCPU) mock'un kendi sahte etiketiyle bile dock yavaşladı: CHARGING 26.9 s'de geldi, boşta 22.7 s. Bir koşuda 25 s'lik yaklaşma süresi aşıldı ve robot geri çekildi. O koşular sayılmadı.

| Kontrol | Sonuç |
|---|---|
| Birim testleri, `python3 -m unittest discover -s test` | Mac'te (Py 3.14) **63/63**, 1'i atlandı (numpy yok). Konteynerde (Py 3.10) ve `colcon test`'te **63/63**. `rosdep check` geçti |
| `cd web && npm test` | **3/3**. Worktree'den çalıştırınca test, yüz çekirdeğini `../../Telefon Yüzü R01 2026-10-07` yolunda bulamıyor. Ana çalışma kopyasında ya da bu yolun doğru olduğu bir kopyada çalıştırılmalı |
| Firmware host testleri (`firmware/luna_mcu/host_test/run.sh`) | **219 kontrol, 0 hata**, 17 test |
| `tags:=usb` duman testi (kamera cihazı yok) | `usb_cam` ayarları okudu: kamera adı, 15 fps, `package://luna_robot/config/camera_info/...` URL'si. Ardından konteynerde `/sys/class/video4linux` olmadığı için beklendiği gibi düştü. `apriltag` düğümleri `tag.frames = [dock_tag_left]` ve `size = 0.048` ile açıldı. SIGINT'te diğer 9 süreç temiz kapandı |
| Sentetik görüntü taraması (`tools/tag_sweep.py`), gerçek `apriltag_ros` ile | aşağıda |
| Tam yığın, `tags:=sim` | aşağıda |

**Sentetik görüntü taraması.**
- **Sahne:** sol kameranın bilinen bir pozdan göreceği 1280×720 görüntü.
- **Kamera pozunun kaynağı:** `core/frames.UrdfChain`.
- **Ölçümün kaynağı:** `robot_state_publisher` → TF → `dock_tag_node` hattı. Kamera pozu ve ölçüm farklı yollardan geldiği için iki URDF yorumu da birbirine karşı denenmiş oldu.
- **Yöntem:** her vakada 8 kare gönderildi.
- **Karşılaştırma:** `/luna/dock/tag` gerçek offset ve mesafeyle, `/luna/dock/tag_position` etiketin gerçek `base_link` konumuyla karşılaştırıldı.

| Grup (720p, `decimate: 2`, seçilen ayar) | Tespit | En büyük hata |
|---|---|---|
| Menzil 0–1.0 m (oturmuş poz dahil), hizalı, kafa düz | 72/72 | konum 8 mm, offset 0.002 |
| Menzil 1.25–1.5 m | 16/16 | konum 10 mm |
| Menzil 1.75–2.0 m | 16/16 | konum 22 mm |
| Menzil 2.25 m / 2.5 m / 3.0 m | 5/8 / 0/8 / 0/8 | konum 50 mm (2.25 m) |
| Yanal 0.05–0.3 m ve yaw −25..+20° | 48/48 | offset 0.003; konum 58 mm (1.5 m'de 0.3 m yanda, yaw −10°) |
| Kafa pan ±20°, ±35° | 32/32 | konum 5 mm, offset 0.001 |
| Robot dönük, kafa etikete çevrili (yaw 25° / pan −30°, yaw 45° / pan −50°) | 24/24 | konum 6 mm, offset 0.001 (45°'de beklendiği gibi +1'de kırpıldı) |
| Tilt +10° (oturmuş), −10°, +12° | 24/24 | konum 4 mm |
| Bulanıklık σ 1.5 ve 3 px (oturmuş), 1.5 px (0.3 m), 1 px (1.0 / 1.5 m) | 40/40 | konum 28 mm (1.5 m, σ 1 px) |

- **Desen ve hat:** Dedektör sentetik deseni her vakada ID 0 olarak çözdü. 2.0 m'ye kadar bütün gruplarda **272/272** kare yayına ulaştı.
- **Gecikme:** Görüntünün yayımlanmasından `/luna/dock/tag`'ın alınmasına kadar ortalama 37 ms. Bu değer Mac VM'de ölçüldü; Xavier'in değeri değildir.
- **`decimate: 1` ile karşılaştırma:** 2.25 m ve 2.5 m'de 8/8, gecikme ~70 ms.
- **1920×1080, `decimate: 2` ile karşılaştırma:** 2.5 m'de 8/8, konum hatası 13 mm, gecikme ~45 ms. 1080p, USB bant genişliği ve MJPG çözme yükü getiriyor; Xavier'de denenmedi.
- **3.0 m:** Etiket her iki ayarda da görüldü ama yayımlanmadı, çünkü `base_link`'e uzaklık `max_range_m` (3.0 m) sınırının üstünde.
- **Seçim:** 720p ve `decimate: 2`. Dock yaklaşması zaten ≤1.4 m'den başlıyor (`approach_timeout`). Daha uzaktan yakalamak gerekirse önce `decimate: 1` denenmeli.

**Tam yığın, `ros2 launch luna_robot luna.launch.py tags:=sim`, mock MCU, 1.2 m'den otomatik dock.** Etiket yalnız sim kamera → `apriltag_ros` ×2 → `dock_tag_node` hattından geliyor. Gerçek son poz `/luna/mock/truth`'tan alındı.

| Senaryo | Durumlar | `/luna/dock/tag` | Gerçek son poz |
|---|---|---|---|
| `tags:=mock` (karşılaştırma) | ALIGN → APPROACH 0.1 s → CONTACT 21.7 s → **CHARGING 22.7 s** | 49 Hz (mock) | x −0.1 mm, y 0.0 mm |
| `tags:=sim`, düz | ALIGN 0.2 s → APPROACH 0.2 s → CONTACT 22.1 s → **CHARGING 23.1 s** | 19.7 Hz, en uzun boşluk 0.29 s | y −0.4 mm, yaw +0.02° |
| `tags:=sim`, kafa +30° (`dock_watch.py --pan 30`) | SEEK → ALIGN 0.7 s → APPROACH 1.3 s → CONTACT 23.3 s → **CHARGING 24.3 s** | 13.9 Hz, offset −0.06..+0.11 | y −0.2 mm |
| `tags:=sim`, robot 20° sola dönük başlıyor (`start_yaw_deg: 20`) | SEEK → ALIGN 0.2 s → APPROACH 1.3 s → CONTACT 23.2 s → **CHARGING 24.2 s** | 19.7 Hz, offset +0.64'ten 0'a | y −0.7 mm, yaw +0.07° |

- SIGINT'le her `tags:=sim` koşusunda 10/10, `tags:=mock`'ta 6/6 süreç temiz kapandı. Dock sırasında `dock_tag_node` hiç TF hatası vermedi. Yalnız bir koşuda açılışta ve kapanışta birer uyarı çıktı: TF tamponu, ilk karenin zaman damgasından 7 ms sonra doluyordu; kapanışta da `head_node` önce durmuştu.
- Kafa +30°'deyken etiket görüntüde 30° sağda duruyordu, yine de yayımlanan offset sıfır çevresinde kaldı. Telafi böylece doğrulandı.
- Geçersiz `mock:=false tags:=sim` önce bir hata verip sahipsiz düğümler bırakıyordu: `OpaqueFunction` yükseldiğinde öndeki düğümler başlamıştı. Bu düğümlerden ikinci bir `dock_node` sonraki koşuda kendi durum makinesini çalıştırdı. Argüman denetimi artık hiçbir düğüm başlamadan yapılıyor; denendi, hiçbir süreç başlamadı.
- Görülen bir başlangıç yarışı: Yeniden derlemeden sonraki ilk `tags:=sim` koşusunda dock, etiket hattı ısınmadan başladı. Robot SEEK'te 16.7 s dönüp etiketi yeniden buldu ve yine CHARGING'e ulaştı (40.6 s). Aynı senaryonun ikinci koşusu yukarıdaki satırdır.
- İşlemci kullanımı, Mac VM, tek çekirdek oranıyla: `apriltag` düğümü başına ~%27, `sim_camera_node` ~%20, diğer düğümler ~%7. Xavier'de ölçülmedi.

### Bulunan ve düzeltilen sorunlar

- **TF'de kamera yoktu.**
  - `head_pan` ve `head_tilt` revolute eklemlerdir; `robot_state_publisher` bunları `/joint_states` gelmeden yayımlamaz.
  - Yığında bu topic'i yayımlayan yoktu, bu yüzden `base_link`'ten kafaya ve kameralara giden TF zinciri hiç oluşmuyordu.
  - Düzeltme: `head_node` artık `/joint_states` yayımlıyor.
- **Humble'da `TransformListener(spin_thread=True)` aynı düğümü ikinci bir yürütücüye ekliyor.**
  - Belirti: Bir geri çağrının içinde zaman aşımlı `lookup_transform` beklenirken `/tf` mesajları işlenmedi. Hata "Lookup would require extrapolation into the future" idi.
  - Etki: İlk taramada tespitlerin yaklaşık yarısı kayboldu; menzil grubunda 112 karenin yalnız 61'i yayımlandı. Bu sayı yalnız gösterge niteliğinde. O koşu (14:37–14:48 UTC) sırasında iki kardeş oturumun konteynerleri de aynı DDS alanındaydı (domain 0) ve kendi mock yığınlarını yayımlıyordu. Nedenin kendisi `tf2_ros` kaynağında doğrulandı; düzeltmeden sonraki bütün ölçümler yalıtılmış koşuldu (`ROS_LOCALHOST_ONLY=1`, domain 37).
  - Neden: `apriltag_ros` önce `detections`'ı, ardından etiketin `/tf`'ini yayımlıyor.
  - Düzeltme: `dock_tag_node` artık beklemiyor. Tespitler en çok `tf_wait_s` (0.2 s) bir kuyrukta bekliyor; kuyruk, TF geldikçe 10 ms'lik bir zamanlayıcıyla boşalıyor. Zamanlayıcı yalnız kuyrukta tespit varken çalışıyor.
- **Süreçler arası 720p görüntü Fast DDS'te 2.4 s'ye kadar takılıyordu.** Bu, robotu da ilgilendiren en önemli bulgu.
  - Belirti: Tespit kendisi ~5–80 ms sürüyordu, ama tespitler kimi zaman 1–2.4 s gecikmeyle geliyordu. Dock eski bir açıyla dönüp salınıyor, sonra FAULT'a düşüyordu.
  - Neden: Humble'ın paylaşımlı bellek segmenti 512 KB. 0.9 MB'lık görüntü UDP parçalarına bölünüyor; loopback'te kaybolan parçalar güvenilir QoS'ta yeniden gönderilmeyi bekliyor.
  - Best-effort QoS ile gecikme 8–59 ms'ye indi, ama karelerin %45–70'i kayboldu.
  - Düzeltme: `config/fastdds_large_images.xml`, 4 MB'lık SHM segmentli bir profil. Ölçüm (80 kare): kayıp 0, gecikme ortalama 11–67 ms, en çok 149 ms.
  - Uygulanışı: `luna.launch.py` profili kamera hattı açıkken (`tags:=sim|usb`) ve kullanıcı başka bir profil vermediyse uyguluyor. Konteynerin `/dev/shm`'i en az 256 MB olmalı; `docker/compose.yaml`'a `shm_size: 512mb` eklendi.
  - Robotta `usb_cam` rgb8 yayımlıyor; 720p bir kare 2.7 MB olduğu için sorun orada daha da büyük olurdu. Bu düzeltme Jetson'da denenmedi.
  - Daha iyi bir çözüm, `usb_cam` ile `apriltag_ros`'u aynı bileşen konteynerinde süreç içi iletişimle çalıştırmak olurdu; bu yapılmadı.
- **Python'dan büyük görüntü yayımlamak yavaştı.**
  - Humble'ın mesaj sınıfı, `Image.data`'ya `bytes` verilince 921 600 baytın her birini Python'da tek tek denetliyor.
  - Etki: Sim kamera kamera başına yalnız ~1.3 kare/s üretebiliyordu.
  - Düzeltme: veri `array.array('B', ...)` olarak veriliyor. Önceden üretilmiş gürültü kareleri de bu süreyi kısalttı.

### Doğrulanmayanlar

- **Gerçek kameralar.** `usb_cam` bir cihazla hiç çalıştırılmadı: MJPG çözme, 15 fps, pozlama ve `/dev/video*` adları denenmedi. B0630'un UVC denetimlerinin (`v4l2-ctl`) adları da bilinmiyor; `config/cameras.yaml` bunları kameranın kendi değerinde bırakıyor (`-1`).
- **Kalibrasyon.** Lensin gerçek iç parametreleri ve bozulması bilinmiyor; düzeltilmiş (`image_rect`) görüntü yok. Sentetik görüntüler ideal pinhole ile üretildi. Bu yüzden yukarıdaki doğruluk sayıları, gerçek lensle elde edilecek doğruluğun üst sınırıdır.
- **Görüntü koşulları.** Gerçek ışık, parlama, hareket bulanıklığı, baskı kalitesi ve plakanın rengi denenmedi. Gerçek odak bulanıklığı da denenmedi; sentetik bulanıklık yalnız Gauss bulanıklığı.
- **Jetson.** `docker/Dockerfile`'a apriltag ve usb_cam kaynakları eklendi, ama Jetson imajı **derlenmedi**. dustynv imajında `cv_bridge`, `image_transport` ve `camera_info_manager` bulunduğu varsayıldı. Xavier'de iki kamera, MJPG çözme ve AprilTag'in işlemci yükü ölçülmedi.
- **Kafa servolarının yönü.** + pan'ın gerçekten sola döndürdüğü kalibrasyonda doğrulanmalı (`pan_direction`).

### Bilinen sınırlar

- **Dock eksenine hizalanma yok.** Durum makinesi etikete yöneliyor, dock eksenine oturmuyor.
  - Robot eksenin 15 cm yanından başlarsa dock'a açılı varıyor ve mock'un 2 cm'lik ağız toleransını kaçırıyor. 3 denemeden sonra FAULT'a düşüyor (test: `test_start_beside_the_dock_axis_is_not_docked_by_bearing_alone`).
  - 5 cm yandan başlarsa ve 20° dönük başlarsa dock'luyor.
  - Bu sınır etiket hattından değil, `core/dock.py`'deki durum makinesinden geliyor. Çözüm için etiketin yönü (yaw) ya da dock ekseni üstünde önde bir hedef nokta kullanılabilir. Küçük etikette yaw belirsizliği (planar ambiguity) bunu zorlaştırıyor.
- **Kafa çok dönükse etiket kaybolur.** Kafa ±35°'ye kadar çevrilmişken dock'luyor; 40°'de etiket görüntüden çıkıyor ve dock başarısız oluyor (birim testi, B0630 720p modeli).
- **Menzil.** `decimate: 2` ile etiket güvenilir olarak 2.0 m'ye kadar görülüyor (2.25 m'de 5/8).
- **`max_age_s` = 0.3 s.** İki kameranın tespitleri yalnız bu kadar yakın zamanlıysa birleştiriliyor.

### Tekrarlamak için

Mac'te, depo kökü "Yazılım R01 2026-10-16":

```bash
docker --context colima build -t luna-dev:humble-tag -f docker/Dockerfile.dev docker
docker --context colima run --rm -it -v "$PWD/luna_ws/src/luna_robot:/src/luna_robot:ro" luna-dev:humble-tag bash
```

Konteynerde:

```bash
source /opt/ros/humble/setup.bash && export ROS_LOCALHOST_ONLY=1
mkdir -p /ws/src && cp -r /src/luna_robot /ws/src/ && cd /ws && colcon build && source install/setup.bash
cd /ws/src/luna_robot
ros2 launch tools/sweep.launch.py decimate:=2.0 &
python3 tools/tag_sweep.py --out /tmp/sweep.json         # sentetik görüntü taraması
ros2 launch luna_robot luna.launch.py tags:=sim &          # (taramayı kapattıktan sonra) tam yığın
python3 tools/dock_watch.py --seconds 45 [--pan 30]
```

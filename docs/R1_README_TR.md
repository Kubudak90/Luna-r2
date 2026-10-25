> **Bu, R1'in belgesi (16–25.10.2026), tarihsel kayıt olarak duruyor.** R2'de palet, kafa servoları, iki göz kamerası, OAK-D Lite, XVF3800 ve JBD BMS yok; bunlardan söz eden bölümler R2 için geçerli değil. Ses zinciri, bulut asistanı, Kimi yönlendiricisi ve SLAM/Nav2 doğrulamalarının ayrıntıları burada. R2: `README_TR.md`, `docs/TASARIM_R2.md`.

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
    dock_nav.py           "dock'a git": Nav2 ile dock öncesi poza, sonra dock_node'a devir (aşağıda)
    sim2d.py              2.5B oda simülatörü: LiDAR düzlemi, köşe ToF konileri, yükseklik bantlı engeller ve temas, tamponlar, dock ve etiket, kayan paletler
    sim_worlds.py         simülatör odaları: room, toy, table, chair, dock, door (engel parkuru)
    depth_camera.py       OAK-D Lite derinlik simülasyonu (numpy ışın izleme, MinZ, z² gürültü)
    tof.py                köşe VL53L1X → Range / nokta yayı dönüşümü, kullanılabilir menzil
    robot_model.py        URDF'ten sensör pozları (LiDAR, OAK-D, ToF): pozların tek yeri URDF
    face_mood.py          robot olayları → yüz olayları (tap/stroke/curious/sleep/wake/gaze); dinliyor/konuşuyor
    dock_tag.py           dock AprilTag'inin base_link konumu → /luna/dock/tag (offset, mesafe); iki kamera birleştirme
    frames.py             dönüşümler + URDF eklem zinciri (TF'in ROS'suz eşi, testler ve sim için)
    camera_model.py       B0630 pinhole modeli (1280×720, 82°)
    tag36h11.py           tag36h11 bit desenleri (AprilRobotics kütüphanesiyle aynı yerleşim)
    tag_scene.py          sentetik kamera görüntüsü (numpy; sim kamera ve tarama)
    xvf3800.py            ReSpeaker XVF3800 USB kontrol protokolü (DoA), ALSA kart bulma
    audio.py              mikrofon PCM'i: kanal seçme, seviye, konuşmayı kayıt parçalarına (WAV) bölme
    attention.py          ses yönü → gözler hemen, kafa yavaşça o yöne; dock'ta ve şarjda kurallı
    assistant.py          sağlayıcıdan bağımsız bulut asistanı istemcisi (Kimi varsayılan), çevrimdışı yedek
    speech.py             cevabı seslendirme: metni temizleme, cümle kuyruğu, takılabilir TTS
  luna_robot/*_node.py  ince ROS 2 düğümleri: base, head, dock, dock_tag, face_bridge, mock_mcu, sim_camera, sim2d, dock_nav,
                          tof_range, sim_depth, audio, attention, assistant, speech
  launch/luna.launch.py   mock:=true (varsayılan) | sim:=true | mock:=false lidar:=true; tags:=mock|sim|usb (sim:=true'da tags boş/mock); voice:= doa:= cloud:=
  launch/luna_nav.launch.py  luna + slam_toolbox + Nav2 + dock_nav (slam.launch.py, nav2.launch.py); obstacle_sensors:=all|lidar world:= depth:=sim|oakd|off
  config/luna.yaml        CALIBRATE işaretli değerler gerçek robotta ölçülecek
  config/cameras.yaml     usb_cam, iki Arducam B0630 (1280×720 MJPG, 15 fps)
  config/apriltag.yaml    apriltag_ros, tag36h11 ID 0, 48 mm
  config/camera_info/     NOMİNAL kamera iç parametreleri (kalibre edilecek)
  config/slam_*.yaml, nav2.yaml  harita/yerelleştirme ve navigasyon ayarları
  config/nav2_obstacle_sensors.yaml  obstacle_sensors:=all: kamera (voxel) ve ToF katmanları, collision_monitor kaynakları
  config/oakd.yaml        depthai_ros_driver ayarları (denenmedi)
  urdf/luna.urdf          R05 CAD'den sadeleştirilmiş çerçeveler; kameralar R06 modelinden, optik çerçeveler dahil
  tools/                  tag_sweep.py (sentetik görüntü taraması), dock_watch.py, make_tag_svg.py, sim_nav_check.py (simülatörde uçtan uca senaryo, Docker'da)
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
| AGX | `/luna/head/state` | Vector3 [pan°, tilt°], hız sınırından sonraki gerçek komut |
| AGX | `/luna/tof/{fl,fr,bl,br}` · `/luna/tof/*/points` | sensor_msgs/Range · PointCloud2 yay; `tof_*` çerçevelerinde, köşe ToF'lerinden (`tof_range_node`) |
| AGX | `/oak/points` | PointCloud2, OAK-D Lite (`depthai_ros_driver`) ya da `sim_depth_node`, optik çerçevede |
| Sim | `/luna/sim/contact` | String "t ad tür bantlar": simülatördeki her engel teması (tampon / çarpma), yalnız test için |
| Ses | `/luna/audio/doa` | Float32MultiArray [`base_link`'e göre yön° (+ = sol), konuşma 0/1 (susturulmuşken ya da kafa hareket ederken 0), kartın ham açısı°, kafa hareket ediyor 0/1, kafaya göre yön°] |
| Ses | `/luna/audio/mute` → `/luna/audio/muted` | Bool komut (true = sustur) → Bool durum (1 Hz ve değişince) |
| Ses | `/luna/audio/utterance` | String: biten konuşmanın WAV yolu (16 kHz mono), konuşma→metin eklentisi için |
| Ses | `/luna/face/gaze` | Vector3 [x, y] −1..1: sese bakan gözler; tazeyken kafa bakışının önüne geçer |
| Ses | `/luna/assistant/request` → `/luna/assistant/reply` | String → String (seslendirilecek metin) |
| Ses | `/luna/assistant/status` · `/luna/voice/speaking` | String: idle / thinking / ok / offline:<neden> · Bool |

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
- **SLAM ve navigasyon:** `slam-nav2` dalında eklendi; ayrıntı ve sınırlar aşağıda, "SLAM, Nav2 ve dock'a git" bölümünde.
- **USB kameralar:** `usb_cam` ile yapılandırıldı (`config/cameras.yaml`); gerçek kamerayla denenmedi.
- **Ses ve bulut asistanı:** `ses-bulut` dalında eklendi; neyin doğrulandığı ve neyin açık kaldığı aşağıda, "Ses ve bulut asistanı" bölümünde. Wi-Fi kartı donanım listesinde (K05).
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

## SLAM, Nav2 ve dock'a git (`slam-nav2` dalı, 17.10.2026)

**Ne eklendi**
- **slam_toolbox** (`config/slam_mapping.yaml`, `config/slam_localization.yaml`, `launch/slam.launch.py`): RPLIDAR C1 için. Tarama `laser` çerçevesinde (URDF'te z 0.58 m), menzil 12 m, 10 Hz. İki kip var: harita çıkarma ve kayıtlı haritada yerelleştirme. Yerelleştirme, haritalamanın başladığı noktada (harita orijini) başlar. Haritalamaya dock'tan başlanırsa robot dock'ta açılıp hemen konumunu bilir; başka yerde açılırsa poz `/initialpose` ile verilir.
- **Nav2** (`config/nav2.yaml`, `launch/nav2.launch.py`):
  - Ayak izi 0.340 m (x, palet boyu) × 0.378 m (y, R06 palet bandı).
  - En çok 0.35 m/s ve 1.2 rad/s, yani `DriveLimits` ile aynı.
  - Kontrolcü Regulated Pure Pursuit: yerinde dönerek yön alır, geri gitmez, engel yakınında yavaşlar.
  - collision_monitor ayrıca yavaşlama kutusu ve "çarpışmaya ≥ 1.2 s" sınırı uygular. "Dur" bölgesi bilerek yok: dönmeyi ve geri gitmeyi de keser, robotu duvar dibinde kilitleyebilir.
  - Yerelleştirme ve `/map` slam_toolbox'tan gelir; AMCL kullanılmıyor.
- **Hız yolu:** controller ve recovery davranışları → `cmd_vel_nav` → velocity_smoother → `cmd_vel_smoothed` → collision_monitor → **`/cmd_vel`** → base_node (dock > `/cmd_vel` önceliği, ardından güvenlik kapısı: E-stop, tampon, uçurum, 0.3 s zaman aşımı) → MCU. Nav2 `/luna/mcu/wheel_target` ya da `/luna/dock/cmd_vel` yayımlamaz. `test/test_nav_config.py` bunu yapılandırma dosyalarında, `tools/sim_nav_check.py` de çalışan ROS grafiğinde denetler.
- **Dock'a git** (`core/dock_nav.py` + `dock_nav_node.py`, istek `/luna/nav/dock`: `go` | `cancel`, durum `/luna/nav/dock_state`):
  - Batarya politikası `seek_dock`/`critical` dediğinde kendiliğinden de başlar. Bu yüzden `luna_nav.launch.py`, dock_node'un kendi `auto_dock`'unu kapatır; böylece robot bulunduğu yerde dönerek etiket aramaz.
  - Akış: IDLE → WAIT (yerelleşme ya da UNDOCK beklenir) → NAVIGATE (Nav2, dock öncesi poza) → HANDOVER (`/luna/dock/request` = `dock`) → DOCKING → DOCKED.
  - Dock öncesi poz, dock pozunun 0.6 m önünde ve dock eksenindedir. dock_node noktaya dümen kırarak yaklaştığı için girişteki açı hatası ≈ yanal sapma / mesafe olur. Bu yüzden devir yalnız eksen etrafındaki bir konide yapılır (0.6 m'de ≤ 7.8 cm yanal sapma). Konide değilse Nav2 yeniden sürer. Simülatörde dock_node 0.6 m'den ≤ 11 cm yanal sapmayla kenetlendi, 13 cm'de başaramadı.
  - **Dock pozu:** robot dock'a kenetlenip CHARGING'e geçtiğinde öğrenilir ve `<harita>.dock.yaml` dosyasına yazılır. Elle kaydetmek için robot dock'tayken `/luna/nav/save_dock` (std_srvs/Trigger) çağrılır.
  - **Dock ekseni:** robotun yönünden değil, dock'un arkasındaki duvardan alınır; LiDAR'ın önündeki noktalara doğru uydurulur. Huni yamuk robotu da kabul ettiği için ilk denemede eksen robot yönünden alınmıştı. Ölçüm 5° kaymıştı, dock öncesi poz 12.7 cm yana düştü ve kenetlenme başarısız oldu. Duvar düz değilse (köşe, mobilya) robotun yönüne geri dönülür.
  - Kayıtlı pozdan 0.5 m'den fazla sapan bir şarj pozu öğrenilmez; yerelleşme kaymış olabilir, uyarı yazılır.
  - Haritalama kipi başlarken eski dock dosyası `.old` olarak kenara alınır, çünkü yeni haritanın çerçevesi başkadır. Dock pozu bilinmiyorsa `go`, eskisi gibi dock_node'un etiket aramasına düşer.
  - **Hata durumları:**
    - Nav2 başarısız olursa 3 deneme yapılır.
    - dock_node FAULT verirse robot önce Nav2 BackUp ile 0.4 m geri çekilir, sonra bir kez daha dock öncesi poza gider. FAULT'ta robot dock yüzüne dayalı kalıyor; bu konum maliyet haritasında "çarpışma" sayıldığı için Nav2 oradan ne plan yapabiliyor ne dönebiliyordu.
    - Açılışta Nav2 henüz hazır değilse bu bir deneme sayılmaz; 60 s'ye kadar beklenir.
    - Başarısızlıktan ya da kullanıcı iptalinden sonra batarya kaynaklı yeniden deneme 120 s bekler.
- **2B simülatör** (`core/sim2d.py`, `sim2d_node.py`, `luna.launch.py sim:=true`): mock MCU ile aynı topic'leri yayımlar.
  - Oda 5 × 4 m. İçinde yüksek bir dolap, bir kolon ve kuzey duvarda dock var. Dock gövdesi tarama düzleminin altında kalır.
  - `/scan`, C1 gibi 500 ışın ve ±1 cm gürültüyle üretilir.
  - Paletler odometrinin varsaydığından fazla kayar (gerçek kayma katsayısı 1.42, base 1.35 sanıyor; doğrusal %2). SLAM'in düzeltecek bir hatası olsun diye böyle yapıldı.
  - Tamponlar, dock burnuna yamuk çarpma, etiketin görüş alanı, görüş açısı ve dolabın arkasında kalması modellenir.
  - Gerçek konum `/luna/sim/truth`'ta yayımlanır, `/luna/sim/estop` ile E-stop'a basılır. Bunlar yalnız testler içindir.

**Çalıştırma**
```bash
ros2 launch luna_robot luna_nav.launch.py                               # simülatör, harita çıkar
ros2 launch luna_robot luna_nav.launch.py slam_mode:=localization sim_soc:=0.24   # kayıtlı harita, düşük batarya
ros2 launch luna_robot luna_nav.launch.py sim:=false mock:=false lidar:=true slam_mode:=localization   # robot
ros2 topic pub --once /luna/nav/dock std_msgs/String "{data: go}"
```
Haritayı kaydetmek (varsayılan yol `~/.luna/maps/home`; `map:=` ile değişir):
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /root/.luna/maps/home}"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: /root/.luna/maps/home}}"
```

**Bu ortamda doğrulananlar**

| Kontrol | Sonuç |
|---|---|
| Birim testleri (Mac, Py 3.14) | 114 testin **108'i geçti**, 6'sı PyYAML olmadığı için atlandı. Yeniler: `test_dock_nav.py` (dock'a git karar mantığı, duvar ekseni, koni, geri çekilme), `test_sim2d.py`, `test_nav_config.py` |
| Birim testleri (Docker, Py 3.10, PyYAML var) | `python3 -m unittest` ve `colcon test` **114/114**, atlanan yok; `rosdep check` temiz |
| Eski mock yığını (`luna.launch.py`, değişen launch dosyasıyla) | 6 süreç, `/odom` x = 1.1999 m, dock CHARGING, politika `charge`, `auto_dock` = True (bool); Ctrl-C'de 6/6 temiz |
| `cd web && npm test` | **3/3**, asıl checkout'ta. `web/` bu dalda değişmedi. Worktree'de test, `../../Telefon Yüzü R01…` göreli yolunu bulamadığı için çalışmıyor |
| Firmware host testleri (`./host_test/run.sh`) | **17 test, 219 kontrol** geçti |
| Docker: haritalama senaryosu (`tools/sim_nav_check.py mapping`) | 5 koşunun 4'ünde **17/17**, birinde 16/17 (yalnız `.pgm` kaydı; aşağıda). Kenetlenme 5/5 |
| Docker: kayıtlı haritada yerelleştirme + düşük batarya | 5 koşunun 5'inde **12/12** |

Haritalama senaryosu (ayrı colima VM'si, 4 CPU):
- Hız yolu denetimi: `/luna/mcu/wheel_target`'ı yalnız `luna_base`, `/cmd_vel`'i yalnız `collision_monitor`, `/luna/dock/cmd_vel`'i yalnız `luna_dock` yayımlıyor.
- Dolabın arkasındaki keşif hedefine sürüş sırasında E-stop'a basıldı. Nav2 komut göndermeyi sürdürürken (47 `/cmd_vel` mesajı) base_node'un güvenlik kapısı tekerlek hedefini **0.03 s'de** sıfırladı. Robot, motor gecikmesiyle 0.42 s'de durdu. Bırakınca hedefe ulaşıldı.
- SLAM pozu gerçek konuma göre 0.2–1.2 cm, ≤ 1.4°.
- `go` isteğinden **21 s** sonra CHARGING. Devirde gerçek konum dock ekseninden 3.4 cm yanda ve 0.573 m önde; son konum dock'ta 0 mm (3.6° yamuk, huni kabul ediyor).
- Duvardan öğrenilen dock pozu gerçeğe 0.2 cm ve 0.8° yakın. Pose graph ve `.pgm/.yaml` haritası kaydedildi.

Yerelleştirme senaryosu: robot %24 SoC ile açıldı, batarya politikası `seek_dock` dedi ve dock_nav kendiliğinden yola çıktı. Devirde dock ekseninden 2.4 cm yandaydı. **26 s** sonra CHARGING, son konum 0 mm, ardından politika `charge`'a geçti. İki senaryonun sonunda Ctrl-C ile 15/15 süreç temiz kapandı.

**Doğrulanmayanlar**
- Gerçek RPLIDAR C1, `sllidar_ros2` sürücüsü ve onun açı yönü/sırası. Simülatör `angle_min = -π` ile yayımlıyor.
- Gerçek dock hunisinin kabul ettiği yanal sapma ve açı. Simülatörde ±2 cm ve ±0.2 rad varsayıldı; koni (7.5°) ve 0.6 m buna göre seçildi. Gerçek huni ölçülünce `handover_entry` ayarlanmalı.
- Jetson/L4T imajı: `docker/Dockerfile` Nav2 ve slam_toolbox içermiyor. dustynv imajlarında ROS kaynaktan derlendiği için bu ikisinin de kaynaktan derlenmesi gerekecek (Xavier'de uzun sürer). Denemeler resmi arm64 `ros:humble-ros-base` + apt ile yapıldı.
- Uzun süreli koşu, gerçek ev haritası ve döngü kapatma; simülatördeki oda küçük.

**Bulunan sorunlar ve gözlemler**
- **slam_toolbox `transform_timeout` 0.2 s yetmedi.** Aşırı yüklü VM'de Python base_node odom TF'ini 0.3–0.8 s gecikmeyle yayımladı. slam_toolbox, taramanın TF'ini en çok `transform_timeout` kadar bekliyor; süre dolunca tarama düşüyor. Bunu tf2'nin yanıltıcı "earlier than all the data in the transform cache" mesajıyla bildiriyor. Hiç tarama işlenmeyince `map→odom` damgası donuyor ve Nav2 duruyor. 0.5 s'ye çıkarıldı (`scan_queue_size: 3`); Nav2 TF toleransları da 0.5 s yapıldı. Xavier'de de kamera ve ses yükü varken base_node'un TF gecikmesi ölçülmeli.
- **Haritalama kipinde Nav2, haritanın o anki sınırları dışındaki bir hedefe plan yapamıyor** ("goal off the global costmap"). Başlangıçta dolap, x > 2.5 m'yi tamamen gölgeliyordu. Haritalama sırasında dock'a gidilecekse dock'un çevresi önce haritaya girmiş olmalı; yerelleştirme kipinde sorun yok.
- **Ortak colima VM'sindeki ROS konteynerleri birbirini görüyor.** Hepsi DDS alanı 0'da ve aynı docker köprüsünde. Kamera ve ses oturumlarının konteynerleriyle ~30 s aynı ROS grafiği paylaşıldı. Bu koşularda `ROS_LOCALHOST_ONLY=1` ve `ROS_DOMAIN_ID=73` kullanıldı; diğer oturumlara da haber verildi. Ortak VM 4 çekirdekti ve diğer oturumlar ~8 çekirdeklik iş yürütüyordu; orada Nav2 zaman aşımlarına düştü. Kullanıcı onayıyla ayrı bir colima profili (`luna-nav`, 4 CPU / 6 GB) açıldı ve son koşular orada yapıldı.
- Kapanış: aşırı yükte `controller_server`, `planner_server` ve `slam_toolbox` launch'ın SIGTERM/SIGKILL kademesine kaldı (-9); bir kez de `collision_monitor` kapanırken -11 verdi. Ayrı VM'de 15/15 temiz.
- Ayrı VM'de, düzeltmelerden önceki bir koşuda SLAM `map→odom`'u bir kez kalıcı olarak dondurdu ve o koşu dock'a varamadı. Sonraki 10 koşuda (5 haritalama + 5 yerelleştirme) izleyici betik `map→odom`'un 3 s'den fazla eskidiğini hiç görmedi. Nedeni kesin bulunamadı; aşırı yükte tarama düşmesiyle aynı belirti. Gerçek robotta `map→odom` tazeliği izlenmeli: dock_nav 2 s'den eski pozu "yerelleşmemiş" sayar.
- `/slam_toolbox/save_map` (`.pgm/.yaml`) 5 kayıttan birinde "Failed to spin map subscription" ile kaydetmedi: içindeki map_saver `/map`'i 2 s içinde alamadı. Yerelleştirmenin kullandığı pose graph (`serialize_map`) her seferinde kaydedildi. `.pgm` yalnız bakmak içindir; olmazsa komut tekrarlanır.
- Kapanış, ayrı VM'de 10 koşuda: 8'inde 15/15 temiz. Bir koşuda `robot_state_publisher` kapanırken -11 verdi, bir koşuda da `planner_server` deaktivasyonda takılıp launch tarafından -9 ile kapatıldı. Python düğümlerinin hepsi her seferinde temiz kapandı.

**Tekrarlamak için** (Mac, depo kökü). `luna-nav:humble` imajı, `ros:humble-ros-base` üstüne şu komutla kurulup `docker commit` ile saklanan yerel imajdır (~166 MB indirme, 488 MB içerik). İmaj varsayılan colima VM'sinde duruyor; deneme için açılan `luna-nav` VM'si iş bitince silindi.
```bash
apt-get update && apt-get install -y --no-install-recommends ros-humble-slam-toolbox ros-humble-nav2-controller ros-humble-nav2-planner ros-humble-nav2-navfn-planner ros-humble-nav2-behaviors ros-humble-nav2-bt-navigator ros-humble-nav2-lifecycle-manager ros-humble-nav2-velocity-smoother ros-humble-nav2-collision-monitor ros-humble-nav2-regulated-pure-pursuit-controller ros-humble-nav2-costmap-2d ros-humble-nav2-msgs ros-humble-nav2-map-server
```
Ortak VM başka işlerle doluysa ayrı bir profil açılır ve imaj ağ kullanmadan kopyalanır:
```bash
colima start --profile luna-nav --cpu 4 --memory 6
DOCKER_HOST=unix://$HOME/.colima/default/docker.sock docker save luna-nav:humble | DOCKER_HOST=unix://$HOME/.colima/luna-nav/docker.sock docker load
docker context use colima
DOCKER_HOST=unix://$HOME/.colima/luna-nav/docker.sock docker run -d --name luna-nav-run --init -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=73 -v "$PWD/luna_ws/src/luna_robot:/src/luna_robot:ro" luna-nav:humble sleep infinity
```
`colima start --profile`, Docker'ın genel bağlamını yeni VM'ye çevirir; `docker context use colima` diğer oturumlar için onu geri alır. Konteynerde:
```bash
source /opt/ros/humble/setup.bash && mkdir -p /ws/src && cp -r /src/luna_robot /ws/src/ && cd /ws && colcon build && source install/setup.bash
set -m; ros2 launch luna_robot luna_nav.launch.py map:=/ws/maps/room > /ws/mapping.log 2>&1 &
python3 /ws/src/luna_robot/tools/sim_nav_check.py mapping --map /ws/maps/room
```
Ardından launch Ctrl-C (SIGINT) ile kapatılır ve `slam_mode:=localization sim_soc:=0.24` ile `... localization` koşulur. `set -m` gerekli: iş kontrolü kapalı kabukta arka plana atılan süreç SIGINT'i yok sayar ve launch Ctrl-C ile kapanmaz.

## Ses ve bulut asistanı (17.10.2026, `ses-bulut` dalı)

Bu bölüm XVF3800 mikrofon dizisini, ses yönüne bakmayı, bulut asistanı istemcisini ve hoparlör çıkışını anlatır. Hiçbiri gerçek donanımla ya da gerçek bulut çağrısıyla denenmedi; aşağıdaki tablolar neyin nerede doğrulandığını ayrı ayrı gösterir.

### Akış

```
XVF3800 (kafa üstü, R07) ──USB kontrol──> audio_node ──/luna/audio/doa──> attention_node ──/luna/head/target──> head_node
        │                                    │                                 └──/luna/face/gaze──> face_bridge ──> yüz
        └──USB ses (ALSA)──> audio_node ──/luna/audio/utterance (WAV)──> [konuşma→metin eklentisi, YOK]
[konuşma→metin] ──/luna/assistant/request──> assistant_node ──/luna/assistant/reply──> speech_node ──> XVF3800 hoparlör çıkışı
                                                   └──/luna/assistant/status──> face_bridge (thinking)
speech_node ──/luna/voice/speaking──> attention_node (kendi sesi sayılmaz) ve face_bridge (konuşuyor)
```

- **Ses yönü (DoA):** Seeed'in resmî Python örneğindeki USB kontrol aktarımıyla okunur; `xvf_host` ikilisi gerekmez ve indirilmedi. Kullanılan değerler: VID `0x2886`, PID `0x001A`; `DOA_VALUE` = kaynak 20, komut 18; 4 bayt yanıt = [açı 0–359°, konuşma 0/1]; yanıttan önce 1 durum baytı gelir. Kaynak: Seeed wiki, "reSpeaker XVF3800 Control with Python" ve "I2C command list" sayfaları (17.10.2026'da okundu). Durum baytının anlamı (0 = başarılı, 64 = tekrar dene) yalnız I2C sayfasında yazıyor; USB için varsayıldı.
- **Açının yönü belgelenmemiş.** Kartın 0°'sinin nereye baktığı ve açının hangi yöne arttığı Seeed/XMOS sayfalarında yok. Bu yüzden `doa_zero_deg` ve `doa_direction` **CALIBRATE** işaretli: yüzün tam önünden konuşulup okunan açı `doa_zero_deg`'e yazılır. Sol taraftan konuşulduğunda açı pan yönünün tersine artıyorsa `doa_direction: -1` yapılır.
- **Kafaya göre → `base_link`** (`core/head_frame.py`): kart kafanın içinde, logolu yüz yukarıda, USB-C robotun soluna (+y) bakıyor ve pan/tilt ile birlikte dönüyor (R07). `audio_node`, DoA'yı `head_tilt_link → base_link` dönüşümüyle (URDF) çevirir; `/luna/head/state`'ten okuduğu pan ve tilt açılarını kullanır. Bakışa ve kafa dönüşüne giden yön artık tabana göre.
  - İşaretler: pan + = sol (URDF `head_pan`, +z). Tilt + = burun yukarı (`head.py`); URDF'teki `head_tilt` ekseni +y olduğu için oradaki eklem açısı −tilt'e eşit.
  - Açılar ±60° / ±12° sınırına kırpılır; sonuç (−180°, 180°] aralığına sarılır.
  - Düz bir dizi yüksekliği göremez: sesin kendi düzlemindeki izdüşümünün açısını ölçer. Kafa eğikken bu açı sesin yüksekliğine bağlı. Bu yüzden konuşmacının tabana göre yüksekliği varsayılır: `speaker_elevation_deg` 25° (CALIBRATE), 1–2 m uzakta oturan ya da ayakta duran bir yetişkin.
  - Gerçek yükseklik 0–40° arasındaysa tilt ±12°'de bile yön hatası 6°'nin altında kalıyor (test). Dikkatin ölü bölgesi 12°.
- **Kafa hareket ederken dinleme durur** (`core/audio.MicGate`): tilt servosu bir mikrofona yaklaşık 49 mm uzakta (R07).
  - Kafa komutu 2°/s'den hızlı değişirken ve durduktan sonraki 0.3 s boyunca (CALIBRATE) konuşma bayrağı 0 yayımlanır.
  - Bu sürede yeni bir kayıt parçası başlamaz; başlamış olan biter, çünkü o sırada konuşan bir insan var.
  - Dikkat, o sırada duyduğunu dondurur: hareket süresi ne konuşma ne de sessizlik sayılır.
  - `/luna/head/state` servonun ölçülen konumu değil, hız sınırından geçmiş komuttur. Gerçek servo gecikmesi bilinmiyor; settle süresi bu yüzden payla seçildi.
- **Susturma yazılımdan:** mute ve reset düğmeleri kafanın içinde kalıyor (R07). `ros2 topic pub --once /luna/audio/mute std_msgs/msg/Bool "{data: true}"` mikrofonları susturur.
  - Susturulmuşken DoA okunmaz ve yayımlanmaz. Kayıt atılır, yarım kalan konuşma da dahil. Luna sese bakmaz, yüz de "dinliyor" göstermez.
  - Kontrol arayüzü açıksa kartın kendi mute pini de yükseltilir. Seeed'e göre bu, mikrofonları donanımda susturur ve kırmızı mute LED'ini yakar (GPO servisi 20, `GPO_WRITE_VALUE` 1, {pin 30, 1}). Kart yeniden bağlanınca durum tekrar uygulanır.
  - Bu komut Seeed'in sayfasında yalnız I2C için yazıyor; USB'de denenmedi. Yazılım susturması donanım susturmasından bağımsız çalışır.
  - Açılışta durum `start_muted` ile seçilir (varsayılan: açık).
- **LiDAR tablası yansıması (yalnız not):** arka iki mikrofon, sabit LiDAR tablası P16'nın 30 mm altında (R07). Arkadan ve üstten gelen seslerde erken yansıma DoA'yı saptırabilir. Şimdilik yazılımda bir şey yapılmadı; R07 tablanın altına ince akustik köpük öneriyor.
- **Bakış ve kafa dönüşü** (`core/attention.py`):
  - Gözler sese hemen döner, kafa 30°/s ile yavaşça izler. `head_node`'un kendi hız sınırı 90°/s.
  - Kafa en fazla ±50° döner; mekanik sınır ±60°. Tilt +4° (CALIBRATE); mekanik sınır ±12°. Bu sınırlar hiçbir ayarla aşılamaz: `max_pan_deg` 80 verilse bile en fazla 60° istenir.
  - Kafanın baktığı yöne 12°'den yakın bir ses için yalnız gözler döner.
  - Luna 0.3 s'den kısa sesleri ve yönü dağınık sesleri izlemez. Ses kesildikten sonra kişiye 4 s daha bakar, sonra kafa yavaşça ortaya döner.
  - Kafa yalnız dock `IDLE` iken döner. Dock sırasında kameraların öne bakması gerekir: o sırada yalnız gözler döner; önceden dönmüş kafa önce ortaya getirilir.
  - Şarjda ya da dock'ta (`CHARGING`/`DOCKED`) Luna uyur ve sese hiç tepki vermez.
  - Luna konuşurken kendi sesini konuşmacı saymaz ve karşısındakine bakmaya devam eder.
  - Gözlerin bakışı kafa bakışıyla aynı ölçekte (pan/60, −tilt/12). Bu yüzden dikkat bıraktığında yüzde sıçrama olmaz.
- **Yüz:** yüz çekirdeğinin yeni olaya ihtiyacı yok; yalnız mevcut olaylar kullanılıyor (`core/face_mood.VoiceMood`).
  - Dinlerken ve bulut cevabını beklerken `curious` gider; yüzde 1.6 s sürer ve her 1.5 s yeniden gönderilir.
  - Konuşurken `stroke` (mutlu) gider; yüzde 2.8 s sürer ve her 2.5 s yeniden gönderilir.
  - Dock'ta hiçbir ses olayı gönderilmez. Bu olaylar yüz çekirdeğinde uykuyu bozar; dock'ta uyku ve uyanmaya dock karar verir.
  - Sese bakan gözler (`/luna/face/gaze`) 0.5 s tazeyken kafa bakışının önüne geçer.
- **Mikrofon kaydı** (`capture: false`, varsayılan kapalı): `arecord` ile 16 kHz, 2 kanal okunur. Dizinin konuşma bayrağına göre (yoksa enerji eşiğine göre) konuşmalar WAV'a bölünür ve `/tmp/luna_utterances`'ta yalnız son 5'i tutulur. Evdeki insanların sesi kaydedildiği için bilerek kapalı.
- **Hoparlör:** 40 mm 4 Ω hoparlör XVF3800'ün amfili hoparlör çıkışına bağlanacak; ses aynı ALSA kartına çalınır. Bu çalma, dizinin yankı engelleyicisinin referansıdır (Seeed sayfası: JST hoparlör çıkışı "5 W amplified speakers"; empedans ve güç yazmıyor).
- **TTS takılabilir:** varsayılan `tts_engine: none`. Ses çıkmaz, ama konuşma durumu cümle uzunluğu kadar sürer. `tts_engine: command` ile herhangi bir komut bağlanır, örnek: `espeak-ng -v tr --stdout {text}` | `aplay -q -D {device}`. Komut bir kez argv'ye bölünür ve metin tek argüman olarak verilir; kabuk kullanılmaz, cevap metni komut çalıştıramaz. espeak-ng ya da piper imaja **kurulmadı**.

### Bulut asistanı ve anahtar

> **25.10.2026 güncellemesi: varsayılan sağlayıcı artık Kimi Code üyeliği.** Üyelikte API dahil; anahtar [Kimi Code Console](https://www.kimi.com/code/console)'dan alınır. Model `kimi-for-coding`; bu K2.8 Preview'dir. Düşünme kapalı istek, düşünmesiz K2.8 Preview'e gider; sesli cevapta ilk kelime en çabuk bunda gelir. Hızlı sürüm `kimi-for-coding-highspeed` (K2.7 Code Highspeed) yalnız düşünme açıkken hızlıdır, kotayı 3 kat harcar ve Pro+/Allegretto+ planı ister; düşünme kapalıyken o da K2.8 Preview'e düşer. Adres `https://api.kimi.com/coding/v1`; belgeye göre yurt dışı için `https://api.kimi.ai/coding/v1`. Kaynak: kimi.com/code/docs/en/kimi-code/models.html (25.10.2026). Anahtar `KIMI_API_KEY` ortam değişkeninde ya da `~/.config/luna/kimi_api_key` dosyasında olmalı (`chmod 600`). Aşağıdaki kurallar (onay, anahtarın depoda olmaması, maskeleme, yeniden deneme) aynen geçerli. Açık platform (`api.moonshot.ai`, `kimi-k2.6`) `luna.yaml`'da yorum olarak duruyor; K2 serisi Aralık 2026 sonunda kaldırılacak. **Kimi Code API'siyle gerçek bir çağrı yapılmadı** (Luna için anahtar yok). Düşünmeyi kapatan alanın bu uç noktada da `{"thinking": {"type": "disabled"}}` olduğu ve hata gövdelerinin biçimi doğrulanmadı.

- **Sağlayıcıdan bağımsız:** OpenAI uyumlu `POST {base_url}/chat/completions` kullanılıyor. Varsayılan sağlayıcı Kimi (Moonshot AI): `https://api.moonshot.ai/v1`, `Authorization: Bearer <anahtar>`. Kaynak: resmî Kimi API belgeleri (platform.moonshot.ai → platform.kimi.ai), 17.10.2026'da yalnız okundu.
- **"Kimi 2.8" resmî model listesinde yok.** 17.10.2026 itibarıyla listede `kimi-k3`, `kimi-k2.7-code`, `kimi-k2.7-code-highspeed` ve `kimi-k2.6` var.
  - Varsayılan `kimi-k2.6`: düşünme modu kapatılabiliyor (`{"thinking": {"type": "disabled"}}`), bu da sesli cevabı hızlandırır.
  - Model adı `config/luna.yaml`'dan değiştirilir. Kullanıcı hangi modeli kastettiğini doğrulamalı.
  - K2.6'da `temperature` ve `top_p` API tarafından sabit; başka değer hata verir, bu yüzden gönderilmiyor.
- **Onay ve anahtar:** robottan bulut isteği çıkması için iki koşulun ikisi de gerekir.
  1. `cloud_enabled: true` ya da `cloud:=true` (sahibin onayı; varsayılan kapalı).
  2. Anahtar `MOONSHOT_API_KEY` ortam değişkeninde ya da `~/.config/luna/moonshot_api_key` dosyasında olmalı (`chmod 600`).
  - Bir git deposunun içindeki anahtar dosyası **reddedilir**. Başkalarının okuyabildiği dosya uyarıyla kullanılır.
  - Anahtar loglara ve cevaplara yazılmaz; sunucu hatası anahtarı içerse bile `***` ile maskelenir.
  - `.gitignore`'a `.env`, `*.key`, `*api_key*` eklendi.
  - Docker'da anahtar `compose.yaml`'a yazılmaz; ana makinedeki ortam değişkeni aktarılır.
- **Zaman aşımı ve yeniden deneme:**
  - Tek deneme en fazla 15 s, toplam en fazla 30 s.
  - 2 yeniden deneme yapılır, bekleme 1 → 2 s; `Retry-After` başlığına uyulur.
  - Yeniden denenen hatalar: 429 `engine_overloaded`/`rate_limit_reached`, 500/502/503/504 ve ağ hataları.
  - Yeniden denenmeyen hatalar: 429 `exceeded_current_quota`, 400, 401/403, 404 ve içerik filtresi.
  - Üst üste 3 başarısız sorudan sonra ağ 60 s hiç denenmez.
- **Çevrimdışı yedek:** her hata kısa bir Türkçe cevapla seslendirilir; Luna susmaz. Örnek: "İnternete ulaşamıyorum. Birazdan tekrar sorar mısın?"
- **Gizlilik:** soru ve cevap metinleri loglanmaz, yalnız uzunlukları ve sonuç (`ok` / `offline:<neden>`) loglanır. Geçmiş en fazla 6 soru-cevap tutulur ve yalnız bellekte kalır.
- **Konuşmayı metne çevirme (STT) yok.** `/luna/audio/utterance` WAV yollarını yayımlıyor, `/luna/assistant/request` de metin bekliyor. Aradaki eklenti (ör. AGX'te whisper ya da bir bulut STT) yazılmadı. Şimdilik soru elle yayımlanır:
  `ros2 topic pub -w 1 --once /luna/assistant/request std_msgs/msg/String "{data: 'Merhaba Luna'}"`

### Bu dalda doğrulananlar

| Kontrol | Nerede | Sonuç |
|---|---|---|
| `python3 -m unittest discover -s test` (eski 41 + yeni 68) | Mac, Py 3.14 | **109/109 geçti** |
| Yeni testler: XVF3800 protokolü sahte USB cihazına karşı, PCM/konuşma bölme, kafa→`base_link` dönüşümü (sarma, ±60°/±12° kırpma, eğik kafada gidiş-dönüş, yükseklik varsayımının hata sınırı), susturma ve hareket kapısı, dikkat (sınırlar, hız, ölü bölge, dock/şarj kuralları, kendi sesi, takılan zamanlayıcı, harekette donma), yüz ses olayları, asistan (onay, anahtar, istek biçimi, yeniden deneme, `Retry-After`, toplam süre, devre kesici, anahtar maskeleme), TTS (kabuksuz argv) | Mac | geçti; ağ ve ses donanımı kullanılmadı |
| Testler gerçekten hata yakalıyor mu (bilerek bozulan 21 davranış: hız sınırı, dock kuralı, ölü bölge, uyku kuralı, dock'ta uyandırma, kayıt sonu, kendi sesi, onay kapısı, maskeleme, git reddi, kota tekrarı, toplam süre, kabuk birleştirme; eklem kırpması, tilt işareti, tilt'in yok sayılması, harekette donma, bekletme, settle süresi, susturma, mute pini) | Mac | **21/21'i test kırdı** |
| `cd web && npm test` (+1 yeni test: gerçek yüz çekirdeğinde `curious` → meraklı, `stroke` → mutlu) | Mac | **4/4 geçti** (aşağıdaki nota bakın) |
| `firmware/luna_mcu/host_test/run.sh` | Mac, clang | **17 test / 219 kontrol geçti** (firmware değişmedi) |
| `colcon build`, `colcon test` ve `unittest` | Docker, resmî `ros:humble-ros-base` arm64 (Py 3.10), mevcut imaj; yeni imaj ya da paket çekilmedi | **109/109 geçti** |
| `ros2 launch luna_robot luna.launch.py doa:=mock` (mock MCU dock'ta değil, `start_soc` 0.60) | Docker, `--network none`, `ROS_DOMAIN_ID=47` | 10 süreç başladı; Ctrl-C (SIGINT) ile **10/10 temiz kapandı** (çıkış 0) |
| Sahte sesler (+35° ve −30°) → `/luna/head/target` | aynı koşu | kafa pan −30…+35°, tilt 0…+4°; hiçbir komut ±60° / ±12° dışına çıkmadı |
| İlk konuşma örneğinden ilk göz bakışına | aynı koşu | 0.39–0.65 s (0.3 s eşik + yön ortalaması) |
| Sahte ses, kafaya göre ölçülüp `base_link`'e çevrildi | aynı koşu | konuşma örneklerinde yön tam −30.0° / +35.0° (sahte sesin yüksekliği varsayılanla aynı) |
| Kafa hareket ederken | aynı koşu | 38 DoA mesajının hiçbirinde konuşma bayrağı yok |
| `/luna/audio/mute` true → false | aynı koşu | durum false → true → false; susturulmuşken 0 DoA mesajı, açıldıktan sonra yeniden yayın |
| Yüz akışı (`curl /events`) | aynı koşu | `wake`, `curious` (dinliyor), `gaze`, `stroke` (konuşuyor) |
| `/luna/assistant/request` → cevap (onay yok) | aynı koşu | durum `thinking` → `offline:disabled`, 0.2 s'de Türkçe yedek cevap; `speaking` 4.9 s true |
| `cloud:=true`, anahtar yok | Docker | `offline:no_key`; ağ hiç denenmedi (konteynerin ağı da yoktu) |

**Kapanış gözlemi (çözülmedi):** Docker'da ses yığınıyla yapılan 11 Ctrl-C kapanışının 10'unda 10/10 süreç temiz bitti. Birinde launch, SIGINT ve SIGTERM'den 10 s sonra 8 düğümü SIGKILL (−9) ile kapattı. Ölenler arasında bu dalda değişmeyen base, dock ve mock_mcu da vardı; makinenin yükü o sırada çok yüksekti (load average 22–27). Ardından yapılan 5 ayrı kapanış denemesinin hepsi temiz bitti: 3'ü ses düğümleriyle, 2'si `voice:=false` ile. Neden bulunamadı; R01'deki seyrek Humble kapanış takılmalarıyla aynı olabilir.

**Web testi notu:** `web/robot-link.test.mjs` yüz çekirdeğini `../../Telefon Yüzü R01 2026-10-07/...` göreli yoluyla arıyor. Bu yol yalnız ana klasörde çözülüyor; `.worktrees/ses-bulut` içinde test "module not found" ile duruyor. Bu yüzden `web/` bir karalama klasörüne kopyalandı, yanına yüz klasörüne bağlantı kondu ve testler orada koşuldu. Kod hatası değil, worktree konumundan kaynaklanıyor.

**Docker notu:** ilk canlı denemeler kardeş oturumların konteynerleriyle aynı Docker köprü ağında ve `ROS_DOMAIN_ID` 0'da yapıldı. `ros2 node list` öbür konteynerin düğümlerini (`/camera_left/apriltag`, `/luna_dock_tag`) gösterdi; iki yığın birbirinin topic'lerini görmüş olabilir (17.10.2026 17:38–17:44). İlgili oturuma haber verildi. Sonraki bütün koşular `--network none` ve ayrı domain ile yapıldı. Aynı makinede birden çok ROS konteyneri çalışacaksa her biri ayrı `ROS_DOMAIN_ID` ya da `ROS_LOCALHOST_ONLY=1` ile başlatılmalı.

### Bu dalda doğrulanmayanlar

- **Gerçek XVF3800 ile hiçbir şey denenmedi** (kart henüz alınmadı). USB kontrol protokolü yalnız sahte cihaza karşı test edildi.
  - USB üzerinden donanım susturması (GPO 30) da denenmedi.
  - Konuşmacı yüksekliği varsayımı (25°), 2°/s hareket eşiği ve 0.3 s settle süresi tahmin; servo gürültüsünün ve LiDAR yansımasının gerçek etkisi ölçülmedi.
  - DoA açısının sıfırı ve yönü bilinmiyor (CALIBRATE).
  - USB'de durum baytının anlamı varsayım.
  - Kartın kontrol isteklerine ses akışı açıkken cevap verdiği denenmedi.
  - Seeed örneği `sudo` ile çalışıyor: konteyner dışında root'suz kullanım için bir udev kuralı gerekebilir.
- **Mikrofon kaydı hiç çalıştırılmadı.** Bu Mac'te `arecord` yok, resmî ROS imajında da yok. Hangi kanalın işlenmiş (ASR) ses olduğu USB firmware'i için belgelenmemiş (`capture_channel`, CALIBRATE).
- **Gerçek bulut çağrısı yapılmadı:** anahtar yok ve onay verilmedi. Kimi istek biçimi resmî belgelere göre yazıldı ve sahte taşıyıcıyla test edildi. Gerçek yanıt, hata gövdeleri ve gecikme denenmedi. "Kimi 2.8" model adı bulunamadı.
- **Hoparlörden ses çıkmadı.** TTS motoru kurulu değil (`tts_engine: none`). XVF3800'ün hoparlör çıkışının 4 Ω 3 W hoparlörü ne kadar sürdüğü belgelenmemiş.
- **Kafa dönüş hızı canlıda temiz ölçülemedi.** Kod her adımı dikkat düğümünün kendi saatine göre 30°/s ile sınırlıyor; birim testleri, takılan zamanlayıcı testi dahil, bunu doğruluyor. Docker'daki ölçüm ise mesajın alındığı ana göre yapıldı. Makinede aynı anda 3 konteyner vardı (load average 15–28, 4 vCPU), mesaj aralıkları 1.27 s'ye kadar uzadı ve 0.5 s'lik pencerelerde 37–44°/s göründü. Bu sapmanın teslim gecikmesinden mi geldiği, sakin bir makinede ya da zaman damgalı mesajla yeniden ölçülmeli.
- **Jetson imajı kurulmadı.** `docker/Dockerfile` (alsa-utils, python3-usb) ve `compose.yaml` (`/dev/snd`, `/dev/bus/usb`, anahtar aktarımı) değişiklikleri derlenmedi.
- **Telefondaki gerçek yüz canlı akışa bağlanmadı;** akış yalnız `curl` ile okundu.

### Açık işler

- **Konuşmayı metne çevirme eklentisi:** `/luna/audio/utterance` → `/luna/assistant/request`. Adaylar: AGX'te whisper, ya da sağlayıcının STT'si (ayrı onay ister). *25.10.2026: robotta whisper.cpp ile eklendi, "Ses zinciri" bölümü.*
- **Uyandırma sözcüğü ya da bas-konuş:** şu an her konuşma "dinliyor" sayılıyor. Buluta yalnız Luna'ya yöneltilen sorular gitmeli. *25.10.2026: dokun-konuş eklendi; uyandırma sözcüğü hâlâ yok.*
- **Mute durumu dışarıdan görünmüyor:** kart düğmesinin (GPI X1D09) okunması belgelerde çelişkili (Seeed sayfası iki farklı okuma veriyor), okunmadı. Robotun kendisi de susturulduğunu yüzde göstermiyor; kırmızı LED kafa içinde kalıyor, kabuktaki LED deliklerinden görünüp görünmeyeceği bilinmiyor.
- **Söz kesme (barge-in):** Luna konuşurken kişi konuşmaya başlarsa susmuyor. *25.10.2026: yüze dokununca susuyor; sesle söz kesme hâlâ yok.*
- **`/luna/head/target` tek sahipli değil:** kamera/takip düğümleri de kafayı sürecekse bir öncelik düzenleyici gerekir. Şu an yalnız dikkat düğümü yayımlıyor, o da yalnız kafayı kullandığı sürece.

## Birleştirme sonrası doğrulama (17.10.2026)

`kamera-apriltag`, `slam-nav2` ve `ses-bulut` dalları `main`'e birleştirildi (`4dcd138`, `0ad059b`, `a7494ec`).

| Kontrol | Sonuç |
|---|---|
| Mac `unittest` (Py 3.14) | 204 OK, 7 atlandı (numpy/PyYAML yok) |
| Docker `luna-nav:humble`, `colcon test` (Py 3.10) | **204 test, 0 hata, 0 atlanan** |
| Web (`npm test`) · firmware host testleri | 4/4 · 19 test / 246 kontrol |
| Canlı mock yığını (`mock:=true doa:=mock`, SoC %24) | Kendiliğinden dock'a girdi, **56,7 s'de CHARGING** (bir BACKOFF ile). Bağlanan yüze `sleep`. Ses düğümleri ayakta. Ctrl-C (süreç grubuna SIGINT) 2 s'de temiz |
| SLAM haritalama senaryosu (`sim_nav_check.py mapping`, colima 8 CPU) | **17/17.** E-stop Nav2 sürerken durdurdu, keşif hedefi, dock'a git → devir → CHARGING 32.7 s, dockta 0 mm; dock pozu duvardan 1.4 cm / 0.3°; harita ve poz grafı kaydedildi |
| Kayıtlı haritada yerelleştirme + düşük batarya (`localization`, `sim_soc:=0.24`) | **12/12.** Düşük batarya dock'a gidişi kendisi başlattı, CHARGING 25.4 s, dockta 0 mm |

- **4 CPU'luk VM'de geçmiyor:** colima 4 CPU / 8 GB iken iki senaryo da işlemci yetmezliğinden düştü (TF "geleceğe uzanma", controller heartbeat; yük 23–29). Birleştirilmemiş `slam-nav2` dalı da orada geçmedi. VM 8 CPU'ya çıkarılınca (`colima start --cpu 8 --memory 8`) yük 2–4'e indi ve iki senaryo da tam geçti. Nav2 + SLAM + Python 2B simülatör için en az 8 çekirdek gerekiyor; Jetson'da gerçek sürüş ayrıca ölçülmeli.
- **Deneme betiği notları:** etkileşimsiz bir kabukta `ros2 launch … &` SIGINT'i yok sayarak başlar; `set -m` açılıp SIGINT süreç grubuna (`kill -INT -PID`) gönderilmeli. Haritalama launch'ına `map:=/ws/maps/room` verilmezse dock pozu `~/.luna/maps/home.dock.yaml`'a yazılır ve senaryonun kayıt kontrolü düşer.

## Engel algılama: LiDAR düzleminin altı (`engel-algilama` dalı, 18.10.2026)

**Sorun.** RPLIDAR C1 direkte, 0.59 m yükseklikte tarıyor (R08: Z589.8). SLAM ve dock için doğru yer (360° açık görüş), ama bu düzlemin altındaki hiçbir şey Nav2'ye ulaşmıyordu. Sehpa tablası, sandalye oturağı ve yerdeki oyuncak görünmüyordu. Onları yalnız tamponlar durduruyordu, o da dar bir bantta. Bu dalda önce simülatörde sorun gösterildi, sonra köşe ToF'leri ve yeni ön derinlik kamerası (Luxonis OAK-D Lite, sabit odak) Nav2'ye bağlandı. Aynı parkur iki yapılandırmayla koşuldu. Kamera satın alınmadı; donanım tarafı denenmedi.

**Ne eklendi**
- **2.5B simülatör** (`core/sim2d.py`): Duvarlar tam yükseklikte. Diğer engeller yükseklik bandı olan kutu (`Block`) ya da dik silindir (`Cylinder`).
  - LiDAR yalnız tarama düzlemini kesen engelleri görür.
  - Köşe ToF'leri koni boyunca 9 × 5 ışınla modellenir; zemin dahil en yakın dönüş alınır.
  - Temas, robotun R07 CAD'den çıkarılan yükseklik bantlarıyla hesaplanır: palet/şasi Z0–125, P24 tampon şeritleri Z125–149, gövde Z125–345, boyun, kafa Z430–520, LiDAR direği ve C1 (tepe Z601.3, R08), OAK-D.
  - Yalnız tampon şeritlerine değen engel tampon bitini kaldırır. Diğer her temas robotun fark etmediği bir **çarpma** olarak `/luna/sim/contact`'a yazılır (yalnız test için).
- **Dünyalar** (`core/sim_worlds.py`, `world:=`):
  - `room`: eski SLAM/dock odası.
  - `toy`: yolda 0.20 × 0.15 × 0.08 m oyuncak kutusu.
  - `table`: 1.0 × 0.5 m sehpa. Tabla Z0.40–0.45, dört Ø0.04 ayak. Paletler ayakların arasından geçer, kafa tablanın altına sığmaz.
  - `chair`: oturak Z0.42–0.45, dört Ø0.025 ayak. Arkalık Z0.45–0.90 tek yanda; düzlemde görünen tek parça o.
  - `dock`: `room` + dock yolunda küçük, alçak eşyalar. Tarama `room` ile aynı, kayıtlı harita geçerli kalır.
  - `door`: odayı bölen duvarın 0.70 m'lik kapısını 0.30 m'lik bir puf kapatıyor. Hedefe başka yol yok.
- **URDF** (tek yer, R08 CAD'den, 18.10.2026): `laser` (Z0.5898, yaw −90°), `oakd_link` (x 0.105, z 0.365, 15° aşağı), depthai adlarıyla üç optik çerçeve (`oak_rgb/left/right_camera_optical_frame`), `tof_fl/fr/bl/br` ve zemin ToF'leri `tof_down_*`. `core/robot_model.py` (mevcut `frames.UrdfChain` üstüne ince katman) bu pozları simülatöre, `dock_nav_node`'a ve testlere okur; Nav2 ve SLAM TF'ten alır. `depth:=oakd` ile sürücü optik çerçeveleri EEPROM kalibrasyonundan `oakd_link` altına yayımlar; `luna.launch.py` o zaman URDF'teki kopyalarını robot_state_publisher'a vermez, böylece hiçbir çerçevenin iki ebeveyni olmaz.
- **Simüle OAK-D Lite** (`core/depth_camera.py`, `sim_depth_node`, `depth:=sim`):
  - 64 × 48 ışın, 5 Hz, 73° × 58°, MinZ 0.35 m.
  - Stereo gürültüsü σz = 0.0046·z² (0.15 px alt piksel, f ≈ 432 px, 75 mm taban).
  - Gerçek konumdan zemin + duvar + engel ışın izleme yapar. `oak_rgb_camera_optical_frame`'de `/oak/points` yayımlar (gerçek sürücüde derinlik RGB'ye hizalı, `i_align_depth`).
- **`tof_range_node`**: `/luna/mcu/corners`'tan her sensör için iki çıktı üretir:
  - `sensor_msgs/Range` (`/luna/tof/*`): collision_monitor için.
  - Aynı okumanın 27°'lik nokta yayı (`/luna/tof/*/points`): maliyet haritaları için.
  - ≥ 0.70 m okuma "daha yakında bir şey yok" demektir, yalnız temizler. 0 ya da NaN okuma yayımlanmaz.
- **Nav2** (`config/nav2_obstacle_sensors.yaml`, `obstacle_sensors:=all`, varsayılan): yerel ve global haritaya iki katman, collision_monitor'a beş kaynak eklenir.
  - `voxel_layer` (kamera): Z0.03–0.62 m arası işaretler, 2 m'ye kadar. Temizleme 3B'dir ve zemin noktalarını da kullanır; alçak kutunun üstünden zemine giden ışın kutuyu silmez.
  - `tof_layer` (ObstacleLayer, ToF yayları).
  - collision_monitor'a `pointcloud` + 4 × `range` kaynağı.
  - `obstacle_sensors:=lidar` eski davranıştır (yalnız `/scan`). İki kipte düğümler aynıdır; yalnız parametre katmanı değişir.
  - İki kipte de: planlayıcının ayak izi dolgusu 0.14 (yerinde dönüşe 5 cm pay) ve kurtarmalarda robotun çevresini koruyan davranış ağacı. Neden ve kanıt aşağıda, "Dock yolunda ayakkabı kutusu".
- **launch**: `luna_nav.launch.py obstacle_sensors:=all|lidar world:=… depth:=sim|oakd|off`. `depth:=oakd`, `depthai_ros_driver`'ı `sllidar_ros2` gibi isteğe bağlı içerir; paket yoksa uyarıyla atlar. Ayarları `config/oakd.yaml`'dadır ve denenmedi.
- **`tools/sim_nav_check.py course --world … --label …`**: tek Nav2 hedefi verir (`dock`'ta "dock'a git"). Temasları, her engele en yakın plan görünüm açıklığını, süreyi ve sonucu ölçer, tablo için bir `RESULT {json}` satırı basar.

**A/B sonuçları** (Docker `luna-nav:humble`, colima 8 CPU / 8 GB, `ROS_LOCALHOST_ONLY=1`, her satır ayrı bir launch)

Son kodla (R08 pozları, tur başı tarama damgası, slam bekçisi, planlayıcı dolgusu 0.14, kurtarmalarda yakın engelleri koruyan ağaç) 19.10.2026'da koşuldu: dock tüm sensörler 09:09–09:26, diğerleri 09:26–09:40, açık kapılar 09:44–09:47 (`door70` / `door75` / `door80` dünyaları: aynı duvar, engelsiz kapı). Her koşudan önce Mac yükünün 12'nin altına inmesi beklendi; bu sette Mac yükü 2–7'ydi. Her koşuda Fast DDS paylaşımlı bellek artıkları temizlendi.

Sütunlar:
- **Süre:** Nav2 hedefinden sonuca kadar; dock'ta `go`'dan CHARGING + DOCKED'a kadar.
- **Açıklık:** robotun ayak izi ile engel arasındaki en küçük plan görünüm mesafesi, gerçek konumdan.

| Parkur | Sensörler | Sonuç | Süre | Temas (fark edilmeyen) | En küçük açıklık |
|---|---|---|---|---|---|
| Oyuncak kutusu (Z0.08) | yalnız LiDAR | **varamadı**, kutuya dayanıp kaldı | 182 s zaman aşımı | **3 (3)**, paletler | 0 |
| | tümü | **ulaştı** | 16.5 s | **0** | kutu 32.0 cm |
| Sehpa (tabla Z0.40–0.45) | yalnız LiDAR | **varamadı**, altına girmeye çalıştı | 182 s zaman aşımı | **3 (3)**, kafa ↔ tabla | 0 |
| | tümü | **ulaştı**, yanından dolaştı | 19.3 s | **0** | tabla 17.4 cm, ayak 21.9 cm |
| Sandalye (oturak Z0.42–0.45) | yalnız LiDAR | ulaştı (şans, aşağıda) | 14.3 s | 0 | oturak **3.9 cm**, ayak 6.3 cm |
| | tümü | **ulaştı** | 19.4 s | **0** | oturak 12.5 cm, ayak 16.6 cm |
| Kapı, puf yolu kapatıyor (Z0.30) | yalnız LiDAR | Nav2 vazgeçti (ABORTED), ama pufu görmeden: kapıyı planlayamadı (aşağıda) | 26.6 s | 0 | puf 27.8 cm |
| | tümü | **Nav2 temiz vazgeçti** (ABORTED) | 28.2 s | **0** | puf 135.7 cm |
| Dock yolunda eşyalar (kayıtlı harita) | yalnız LiDAR | CHARGING + DOCKED (şans, aşağıda) | 38.0 s | 0 | ayakkabı kutusu 31.7 cm, oyuncak araba 22.8 cm, top **8.0 cm** |
| | tümü | **CHARGING + DOCKED**, 7 koşunun 6'sında; düşen koşu yük altında TF donması (aşağıda) | 42.6–133.1 s | **0** (7 koşuda) | ayakkabı kutusu ≥ 28.9 cm, oyuncak araba ≥ 22.3 cm, top ≥ 16.3 cm |
| Açık kapı 0.70 m (engelsiz) | yalnız LiDAR / tümü | **geçemedi**: Nav2 vazgeçti (ABORTED), planlayıcı kapıyı kapalı görüyor | 38.2 s / 38.2 s | 0 / 0 | – |
| Açık kapı 0.75 m | yalnız LiDAR / tümü | **ulaştı** | 16.0 s / 15.9 s | 0 / 0 | – |
| Açık kapı 0.80 m | yalnız LiDAR / tümü | **ulaştı** | 16.4 s / 16.1 s | 0 / 0 | – |
- Tüm sensörlerle bu setteki her koşuda temas 0. Yalnız LiDAR'la oyuncak kutusunda ve sehpada 3'er temas oldu; 6'sını da robot fark etmedi (paletler ya da kafa).
- **Yalnız LiDAR'daki üç "iyileşme" algılamadan gelmiyor.** Planlayıcı dolgusu yolları değiştirdi:
  - Sandalyede LiDAR yalnız arkalığı ve arka ayakları görüyor. Planlayıcı onlardan 0.31 m uzak durunca robot görmediği oturağın 3.9 cm yanından geçti.
  - Kapıda 0.70 m'lik geçit planlayıcıya kapalı. Robot kapıya 0.77 m kala "failed to create plan" ile durdu ve görünmeyen pufa hiç varmadı.
  - Dock'ta planlayıcı baştan güney yolunu seçti, ayakkabı kutusuna hiç yaklaşmadı. Görmediği topun 8 cm yanından geçti.
- **Önceki set (dolgu 0.01, 00:20–02:12, Mac yükü 7–37):** yalnız LiDAR'la 18 temas (14'ü fark edilmeyen), tüm sensörlerle dock dışında 0.
  - Kapı, tüm sensörler: ilk iki koşuda senaryo betiği hedefi `bt_navigator` hâlâ etkinleşirken gönderdi ve hedef kayboldu. Bu bir betik yarışıydı; betik artık Nav2'nin "aktif" demesini bekliyor.
  - Dock, tüm sensörler: Mac yükü 7–9 iken robot 144.7 s'de kenetlendi ama ayakkabı kutusuna 2 kez değdi. Yük 13–15 iken 4 kez değdi ve kenetlenmedi. Yük 6.6 iken 0 temas, ama yalnız 1.9 cm açıklık vardı. R08 setinde 7.1 cm, R07 setinde 6.4 cm'ydi. Önce yükten sanıldı. Kök neden ve çözüm aşağıda, "Dock yolunda ayakkabı kutusu".
- **Eski–yeni farkı (tek satır):** Yalnız LiDAR'da temas sayıları: R07 pozlarıyla 18 (14'ü fark edilmeyen), ilk R08 setinde 16 (12), damga + bekçi setinde 18 (14), dolgu 0.14 ile 6 (6). Tüm sensörlerle temas önceki setlerde yalnız dock'ta vardı, bu sette hiç yok.
- Dock eşyaları, yalnız LiDAR'lı robotun gerçekten izlediği yola konuldu. Eşyalar dock_node'un son 0.6 m'lik yaklaşma bölgesinin dışında.

**Eski senaryolar (son kod)**

| Kontrol | Yalnız LiDAR | Tümü |
|---|---|---|
| Haritalama (`sim_nav_check.py mapping`): duvardan dock ekseni öğrenme dahil | **17/17** | **17/17** |
| Kayıtlı haritada yerelleştirme + düşük batarya (`localization`, `sim_soc:=0.24`) | **12/12** (betik düzeltmesinden sonra; aşağıda); ayrıca art arda 6/6 deneme | **12/12** (düzeltmeden önce de, sonra da); ayrıca art arda 6/6 deneme |
| Birim testleri | Mac (Py 3.14) 260 test, 22'si numpy/PyYAML olmadığı için atlandı · Docker `colcon test` **260/260** | |

- Haritalama satırı 09:26–09:29'daki set (dolgu 0.14). Yerelleştirme son kez 09:42–09:43'te koşuldu.
- **Betik yarışı, açılış kontrolü:** dolgu 0.14 setinde yalnız LiDAR'lı yerelleştirme 11/12 kaldı. Düşen kontrol "start on the saved map": 1.8 cm, 5.0°; sınır 4.6° (0.08 rad). Bekçinin zorla dondurma denemesinde de aynı kontrol 5.4° ile düşmüştü.
  - Mac yükü düşükken Nav2 launch'ın 4. saniyesinde etkinleşti. `dock_nav` hedefi hemen gönderdi; kontrol çalıştığında robot zaten dönüyordu.
  - Betik SLAM pozunu en son TF'ten, gerçek konumu ise en son simülatör mesajından alıyordu. İkisinin arasındaki 0.08 s, 1 rad/s dönüşte tek başına 4.6° eder.
  - Önceki setlerde yük yüksekti, Nav2 geç açılıyordu ve kontrol robot kalkmadan yapılıyordu.
  - `tools/sim_nav_check.py` artık SLAM pozunu gerçek konum mesajının damgasındaki TF'ten alıyor. Düzeltmeden sonra iki kipte de 12/12.
  - Açılışta yalnız LiDAR 0.7 cm, 1.6°; tüm sensörler 1.3 cm, 4.4°. Robot ikisinde de hareket halindeydi. Damga hizalaması zaman kaymasını kaldırdı, ama dönen robotta SLAM'in kendi açı hatası sınıra yakın kalıyor.
  - **Açık:** bu kontrol açılış yerelleşmesini değil, kalkmış robotu ölçüyor. Sağlam çözüm, senaryoda `dock_nav`'ın kontrol bitene kadar beklemesi.

**Yerelleştirme kararsızlığı: kök neden ve çözüm (19.10.2026)**

R08'den sonra yalnız LiDAR'lı yerelleştirme üç kez düşmüştü (5/12, 8/12, 7/12). Ayrıntılı ölçüm:
- **Çerçeve değil.** Aynı kodla `laser_joint` R07 değerine (z 0.58, yaw 0) döndürüldü, harita o çerçeveyle yeniden çıkarıldı ve 5 deneme koşuldu: 4/5 geçti. R08 değerleriyle de 4/5 geçti. Sonraki R08 denemeleri 6/6 ve 6/6 geçti. Tüm sensörlü kipte de düşüşler oldu (6 denemede 3). Sorun ne LiDAR'ın dönmesinde ne de tek bir kipte.
- **Ne oluyor?** Her denemede bir gözlemci `map→odom`'un yaşını saniyede bir yazdı. İki ayrı belirti görüldü:
  1. **Tam kilitlenme** (38 denemede 4): `map→odom` damgası bir anda donuyor ve bir daha ilerlemiyor. slam_toolbox'ın bütün iş parçacıkları futex'te uyuyor, işlemci harcamıyor. Parametre servisi 15 s'de cevap vermiyor, yani yürütücüsü kilitli. Bu sırada `/scan` 9 Hz akmaya devam ediyor.
  2. **1–2 s'lik takılma** (38 denemede 2): damga bir an geride kalıyor. slam_toolbox `map→odom`'u "son işlenen tarama + `transform_timeout` (0.5 s)" damgasıyla yayımladığı için Nav2 denetleyicisi "Transform data too old" deyip duruyor.
- **Elenen olasılıklar:** odometri TF'inde 2 s'lik boşluk kilitlenme yapmadı (base_node SIGSTOP ile durduruldu, SLAM 1 s'de toparlandı). tf2_ros'un MessageFilter'ında, Buffer'ında ve CreateTimerROS'ta ters kilit sırası yok (Humble kaynağı okundu). slam_toolbox'ın kendi kilit sırası tutarlı.
- **Kök neden (simülatörde):** sim2d taramayı yayımladığı an ("şimdi") damgalıyordu. Odometri TF'i her zaman birkaç milisaniye eskiydi. slam_toolbox'ın tf2 MessageFilter'ı bu yüzden **her tarama için** bir "TF hazır olunca haber ver" isteği ve bir zaman aşımı zamanlayıcısı kurdu. Dört kilitlenmeden ikisi açılışta, filtrenin "queue is full" düşürmesinden birkaç saniye sonra (5. ve 7. s) geldi. İkisi sürüş sırasında geldi (26. ve 31. s; biri VM yükü 17 iken). ROS'ta `LaserScan` damgası ilk ışının zamanıdır; gerçek C1 sürücüsü de (sllidar_ros2, `grabScanDataHq`'dan önce alınan zaman) taramayı turun başında damgalıyor. Yani tarama yaklaşık 0.1 s eski gelir ve TF'i hazırdır. Simülatör artık taramayı 0.1 s önceye damgalıyor ve robotun o anki konumundan üretiyor. Kilidin slam_toolbox / tf2 içindeki tam yeri hata ayıklayıcı olmadığı için (kurulmadı) bulunamadı.
- **Çözüm:**
  1. Tarama damgası tur başında (`sim2d_node.py`).
  2. Yerelleştirmede `transform_timeout` 0.5 → 1.5 s (`config/slam_localization.yaml`): 1–2 s'lik takılmalar Nav2'yi artık durdurmuyor.
  3. **`slam_watchdog_node`** güvenlik ağı: tarama akarken `map→odom` 4 s ilerlemezse slam_toolbox'ın parametre servisini yokluyor. Cevap gelirse düğüm yalnız yavaştır, dokunulmaz. 8 s cevap gelmezse öldürüyor; launch onu aynı kayıtlı haritayla yeniden başlatıyor (`respawn`). Cevap verse de `map→odom` 20 s ilerlemezse yine yeniden başlatıyor (bir kez görüldü: yürütücü canlı, taramalar işlenmiyor). Yoklama, takılmanın ne zamandır sürdüğünü sıfırlamıyor. `/tf` kuyruğu 10'a indi: 100'lük kuyrukla bekçi yük altında 90 s geriden geliyordu. Her 10 s'de bir durumunu yazıyor (`WATCH: map→odom last moved … ago`). Bekçi son sağlam `map→odom` ile güncel odometriden hesaplanan konumu `/initialpose` ile geri veriyor. Yalnız yerelleştirmede çalışıyor; haritalamada yeniden başlatma, çıkarılan haritayı kaybettirirdi.
  4. Aynı açılış yarışı iki yerde daha vardı ve kapatıldı: `dock_nav_node` ile senaryo betiği hedefi artık Nav2'nin yaşam döngüsü yöneticisi "aktif" demeden göndermiyor.
- **Kanıt:**
  - Damga düzeltmesinden önce aynı yük düzeyinde 38 denemede 6 düşüş vardı (4 kilitlenme, 2 takılma). Düzeltmeden sonra **12/12 art arda geçti**: yalnız LiDAR 6/6, tüm sensörler 6/6. Hiç kilitlenme olmadı, bekçi bir kez bile devreye girmedi.
  - Bekçi ayrıca zorla denendi: koşunun 25. saniyesinde slam_toolbox SIGSTOP ile kalıcı dondurulunca 12 s içinde yeniden başlattı ve konumu (2.27, 1.39) geri verdi; senaryo **12/12** geçti. Son bekçiyle (tüm sensörler) tekrarlandı: 28. s'de yokladı, 36. s'de yeniden başlattı, 39–41. s'de konumu (2.97, 1.10) üç kez verdi, sonra yeniden WATCH. Robot başlangıçtan 54 s sonra kenetlendi, dock'ta SLAM ile gerçek konum farkı 1.8 cm. Senaryo 11/12 geçti. Düşen kontrol dondurmadan önceydi: açılışta SLAM konumu 5.4° saptı, sınır 4.6° (0.08 rad).
  - Düzeltmeden önceki doğrulamada da (bekçi vardı, damga eskiydi) bekçi gerçek bir kilitlenmeyi iki kez yeniden başlatarak kurtardı. Ama ikinci örnek de açılışta kilitlendiği için 42 s sürdü ve o deneme düştü.
- **Gerçek robotta:** sllidar_ros2 zaten tur başında damgalıyor. Ama Python base_node'un odometri TF'i yük altında 0.1 s'den fazla gecikirse aynı bekleme yolu açılır. Bekçi bu yüzden kalıyor. Xavier'de TF gecikmesi ölçülmeli.

**Dock yolunda ayakkabı kutusu: kök neden ve çözüm (19.10.2026)**

Kutu (Z0–0.10) dock öncesi poza giden yolun iç köşesinde. Tüm sensörlerle bile 2 ve 4 temaslı koşular, temassızlarda 0.7–4.8 cm'lik açıklıklar görülmüştü.
- **Hangi evre?** Temasların ve düşük açıklıkların hepsi NAVIGATE evresinde oldu: Nav2 robotu dock öncesi poza (dock'un 0.6 m önü) götürürken. dock_node'un son 0.6 m'lik düz yaklaşmasında hiç olmadı; kutu o bölgenin dışında. Robot her seferinde kutunun yanında yerinde dönerken köşesiyle kutuya girdi.
- **Kök neden 1: kurtarmalar alçak engeli unutturuyordu.**
  - Nav2 Humble'ın varsayılan davranış ağacı takılınca iki maliyet haritasını da tamamen siler (`ClearEntireCostmap`).
  - LiDAR duvarları bir turda geri yazar. 0.10 m'lik kutuyu ise yalnız kamera görür, o da kutu önündeyse ve 0.35 m'den uzaktaysa. Kutu robotun yanındayken silinince bir daha görülemedi, robot üstüne döndü.
  - Ölçüm: temaslı her koşuda temastan birkaç saniye önce bir temizleme vardı (ör. temizleme 30. s, temas 34. s).
  - "Voxel ışın temizlemesi kutuyu siliyor" varsayımı elendi: maliyet haritası yoklaması, temizleme olmayınca kutunun robot yanından geçerken ölümcül kaldığını gösterdi.
- **Kök neden 2: dönüş süpürmesi planlayıcının payından büyük.**
  - Paletli robot yerinde dönerken köşeleri hypot(0.17, 0.189) = 0.254 m yarıçaplı daire süpürür.
  - NavFn yolu engelden yalnız iç yarıçap kadar uzak tutuyordu: 0.17 + 0.01 dolgu = 0.18 m. RPP yön değiştirmek için yerinde dönünce köşe kutuya 7 cm'ye kadar girebiliyordu.
  - RPP'nin çarpışma denetimi yalnız ölümcül hücreye bakar. Kutu haritadayken bile robot ancak 0–2 cm kala durur.
- **Çözüm:**
  1. `behavior_trees/navigate_to_pose_keep_near_obstacles.xml`: Humble ağacının kopyası. Dört `ClearEntireCostmap` yerine `ClearCostmapExceptRegion` (`reset_distance` 2.0) kullanılıyor; robotun çevresindeki 2 m'lik kare korunuyor, uzaktaki izler yine siliniyor. `launch/nav2.launch.py` bunu `default_nav_to_pose_bt_xml` olarak veriyor.
  2. Planlayıcının (global harita) ayak izi dolgusu 0.01 → **0.14**: iç yarıçap 0.31 m ≥ 0.254 + 5 cm. Planın geçtiği her noktada yerinde dönüş, haritada bilinen engelden 5 cm uzak kalır. RPP köşe keserse ya da engel geç görülürse bu bir garanti değil.
  3. Global şişirme yarıçapı 0.45 → 0.55: dolgulu çevrel yarıçaptan (0.45 m) büyük kalmalı.
  4. Yerel harita dolgusu 0.01'de kaldı. RPP'nin çarpışma denetimi gerçek ayak iziyle bakıyor; orada dolgu, robotu dar geçitte "collision ahead" ile durdurdu.
  5. İki test bunları sabitliyor: `test_turning_in_place_stays_inside_the_planners_obstacle_margin` ve `test_recoveries_keep_what_only_the_camera_and_tof_saw`.
- **Denenip bırakılanlar:**
  - İki haritaya da 0.09 dolgu: robot 0.68 m'lik geçitte "collision ahead" deyip takıldı.
  - Global şişirme 0.60 / 2.5: fark yok.
  - Global 0.09 dolgu + yeni ağaç: 5 koşuda 0 temas, ama kutu açıklığı 0.7, 4.8, 3.9 cm. Köşe bir dönüşte kutunun 0.6 cm yanından geçti; pay yetmiyor.
- **Bedeli:**
  - **0.70 m'lik kapıdan geçemiyor.** Engelsiz 0.70 m'lik kapıda iki kipte de planlayıcı kapıyı kapalı gördü ve Nav2 vazgeçti. 0.75 ve 0.80 m'lik kapılardan iki kipte de geçti (yukarıdaki tablo). Kâğıt üstünde sınır, işaretli hücreler arasında 2 × 0.31 m + bir 0.05 m hücre ≈ 0.67 m. Ama duvar uçları haritada bir hücre kalın çıkınca 0.70 m kapanıyor. Dolgu 0.01 ile 0.70 m geometri gereği açıktı (koşulmadı).
  - Evde 0.70–0.75 m'lik net kapı genişliği (ör. banyo kapısı) yaygınsa bu bedel kabul edilemez. O zaman ayak izine bakan bir planlayıcı gerekir (Smac Hybrid-A* / Lattice), dolgu değil.
  - Kutu ile kuzey duvar arası 0.68 m. Izgaranın düştüğü yere göre açık ya da kapalı görünüyor; koşularda hep kapalı çıktı.
  - Kutu kayıtlı haritada yok. Robot önce kuzeye yöneliyor; kutuyu kamerada görünce dolabın güneyinden dolaşıyor. Kuzeyde 15–45 s duraksıyor (yol bir o yana bir bu yana dönüyor).
  - Kenetlenen koşularda dock süresi 43–133 s (önceki setlerde 84–145 s).

**Kanıt** (dolgu 0.14 + yeni ağaç, tüm sensörler, kayıtlı harita, 19.10.2026 09:09–09:26). Her koşudan önce Mac yükünün 12'nin, yüklü koşularda 14'ün altına inmesi beklendi. Yük için VM içinde 6 × `yes` çalıştırıldı. VM yükü koşu sonundaki 1 dk ortalaması.

| Koşu | Yapay yük | VM yükü | Sonuç | Süre | Temas | Ayakkabı kutusu | Oyuncak araba | Top | Yol |
|---|---|---|---|---|---|---|---|---|---|
| 1 | yok | 1.5 | **CHARGING + DOCKED** | 133.1 s | **0** | 29.7 cm | 22.3 cm | 17.0 cm | kuzey → güney |
| 2 | yok | 1.1 | **CHARGING + DOCKED** | 96.2 s | **0** | 31.6 cm | 23.3 cm | 18.3 cm | kuzey → güney |
| 3 | yok | 1.5 | **CHARGING + DOCKED** | 90.7 s | **0** | 35.5 cm | 26.0 cm | 21.5 cm | kuzey → güney |
| 4 | 6 × `yes` | 8.7 | **kenetlenmedi**, TF donması (aşağıda) | 240 s zaman aşımı | **0** | 92.7 cm | 223.5 cm | 127.1 cm | kuzeyde kaldı |
| 4b (yeniden koşu) | 6 × `yes` | 6.2 | **CHARGING + DOCKED** | 42.6 s | **0** | 28.9 cm | 22.8 cm | 17.3 cm | baştan güney |
| 5 | 6 × `yes` | 8.6 | **CHARGING + DOCKED** | 78.9 s | **0** | 30.9 cm | 23.1 cm | 16.3 cm | kuzey → güney |
| 6 | 6 × `yes` | 8.0 | **CHARGING + DOCKED** | 71.6 s | **0** | 32.7 cm | 24.5 cm | 26.6 cm | kuzey → güney |

- 7 koşunun hepsinde temas 0. Kenetlenen 6 koşuda en küçük açıklık: ayakkabı kutusu **28.9 cm**, oyuncak araba 22.3 cm, top 16.3 cm. Önceki sette ayakkabı kutusu 1.9 cm'ydi.
- Yük altında 4 koşudan 3'ü kenetlendi.
- **Açık: yük altında Nav2'nin TF görüşü dondu (koşu 4).** Neden dock payı değil.
  - Robot 27. s'den sonra kuzeyde, (0.97, 1.34)'te kaldı.
  - 46. s'den sonra denetleyici her döngüde "Transform data too old (map → odom)" dedi. Aradaki fark saniyede 1 s büyüdü (0.6 → 36 s), yani yerel haritanın TF tamponuna 45. s'den sonra yeni `map→odom` hiç girmedi.
  - Aynı sırada odom→base o tampona ulaşmaya devam etti. slam_watchdog da `/tf`'te `map→odom` damgasının her 0.1 s'de ilerlediğini yazdı: slam_toolbox yayımlıyordu.
  - Fast DDS hata yazmadı. Kopukluk tek bir yayımcı → abone hattında görünüyor. Bekçi yayın tarafını izlediği için bunu göremez.
  - 7 dock koşusunda bir kez görüldü; kural gereği yeniden koşuldu (4b).
  - 0.09 dolgulu önceki setin dockP_4'ünde benzer bir uyarı vardı. Ama orada fark yalnız 1.4 s'ydi ve geçti; robot planlayıcı hatası ile yük altında başarısız kurtarmalar yüzünden ("Current footprint not available") takılmıştı. Orada bekçinin susması doğruydu.
  - Gerçek robotta tüketici tarafında bir TF tazelik denetimi düşünülmeli.

**Bulunan ve düzeltilen sorunlar**
1. **Köşe ToF'leri çapraz değil, düz bakıyor.** R07'deki `FOV_R06_TOF_*` konilerinin ekseni ±Y_cad, yani robotun ileri/geri yönü. Eski simülatör çapraz modelliyordu. CAD'deki `*_L` parçaları robotun **sağında** (X < 0): `tof_fl` = `P20_F_R`. `/luna/mcu/corners` ve tampon bitleri robotun kendi sol/sağını kullanır.
2. **Zemin ToF konisinin içinde.** Z0.101'deki VL53L1X'in varsayılan tam ROI'si (16 × 16 SPAD, 27°) zemini 0.42 m'de görür; o zaman her okuma zemin olur. Simülatör ROI'nin 16 × 4'e daraltıldığını varsayıyor (27° geniş, ≈15° dikey, zemin ≈0.77 m). Yalnız 0.70 m'den yakın okumalar engel sayılıyor. Firmware ROI ayarlamıyor ve köşeleri 5 Hz yayımlıyor (`init_tof`, `main.cpp`). Gerçek robotta `setROISize(16, 4)` gerekecek, kısa mod iyi olabilir. Zemin dönüşü gerçek sensörde ölçülmeli. Varsayım `test_obstacles.py` içinde iki testle sabitlendi (27°'de zemin 0.43 m, 15°'de > 0.70 m).
3. **Nav2 Humble'ın `RangeSensorLayer`'ı bu yığında kullanılamadı.**
   - Gelen her Range'i sınırsız bir listede tutuyor. Her TF araması başarısız olunca `transform_tolerance` kadar (0.5 s) bekliyor.
   - Açılışta SLAM'in `map` çerçevesi gelmeden gelen okumalar global haritayı kalıcı olarak kilitledi: 372/372 arama başarısız, planlayıcı 0.009 Hz'e düştü. Robot, ön yüzü sehpaya 24 cm kala 3 dakika bekledi.
   - Aynı yapılandırmayla bir başka koşu sorunsuz geçti; hata açılış sırasına bağlı.
   - ToF bu yüzden haritalara nokta yayı olarak, TF filtreli sıradan bir ObstacleLayer üzerinden veriliyor. collision_monitor yalnız son Range'i tuttuğu için onu okumaya devam ediyor.
4. **Range damgası.** Varış anıyla damgalanınca TF araması henüz yayımlanmamış odometriyi istiyordu. Damga artık "varış − 0.1 s" (ölçüm anı tahmini: 50 ms ölçüm bütçesi, 5 Hz okuma, micro-ROS).
5. **Dock'a git, Nav2 açılırken yarışa giriyordu.** `dock_nav_node`, Nav2'nin eylem sunucusu var ama henüz aktif değilken hedef gönderebiliyordu. Hedef birkaç saniye sonra kabul edildiğinde zaman damgası bt_navigator / planner_server'ın TF önbelleğinden eski kalıyordu; robot pozunu o anda arayan Nav2 sonsuza dek "geçmişe uzanma" hatası verdi ve robot yerinden kalkmadı. Yeni düğümler açılış sırasını değiştirince ortaya çıktı (eski bir yarış). Damgayı sıfır ("en son") yapmak denendi ama Nav2 Humble'da hedefi aralıklı olarak başlangıçta "ulaşıldı" saydı. Çözüm: `dock_nav_node` artık `lifecycle_manager_navigation/is_active` "aktif" diyene kadar hedef göndermiyor; bu bekleme deneme sayılmıyor.
6. **Tampon kapsamı dar.**
   - P24 şeritleri yalnız |y| ≤ 0.125 m ve Z125–149'u kaplıyor.
   - Palet uçları (|y| 0.141–0.189) ve 0.125 m'den alçak her şey tampona değmeden çarpıyor: oyuncak kutusu, palet ucuna gelen sandalye ayağı.
   - Bu çarpmaları güvenlik kapısı hiç görmez.

**R08 pozları ile bu dalın önceki pozları (karşılaştırma)**
- `laser`: Z0.580 → **0.5898** (C1 veri sayfası lazer yüksekliği) ve yaw 0 → **−90°**. sllidar_ros2'de +x kablo çıkışına, yani robotun sağına bakıyor. Simülatör taramayı bu çerçevede üretiyor. Dock'taki duvar ekseni de (`front_points`) bu açıyı kullanıyor. Robot yüksekliği C1 tepesiyle 0.606 → 0.6013 m.
- OAK-D: `oakd_link` aynı. `oakd_optical_frame` kaldırıldı; yerine depthai adlarıyla `oak_rgb/left/right_camera_optical_frame` geldi. Bulut RGB optik çerçevesinde, merkezi (0.105, 0.365) → (0.113, 0.368).
- Köşe ToF'ler aynı: 0.16888 / 0.09355 → 0.1689 / 0.0935 (R08 0.1 mm'ye yuvarlıyor), yönler aynı.
- Yeni: zemin ToF çerçeveleri `tof_down_*` (+x aşağı).
- Haritalar yeni çerçeveyle her koşuda baştan üretildi. Gerçek robotta henüz kayıtlı harita yok.

**Yalnız simülasyonda olanlar / doğrulanmayanlar**
- Gerçek sensörlerin hiçbiri denenmedi. VL53L1X (ROI, zemin dönüşü, koyu/parlak yüzey), OAK-D Lite ve `depthai_ros_driver` (parametre adları, `camera_model: OAK-D-LITE`, nokta bulutu adı) denenmedi. Sürücü imajda yok, kurulmadı.
- **Pasif stereo:** OAK-D Lite'ta IR projektör yok. Desensiz düz yüzeylerde (beyaz duvar, düz dolap kapağı, tek renk sehpa) eşleşme bulunamayabilir; o zaman derinlik boş kalır. Model bunu içermiyor; boş görüntü burada her zaman zemin ya da engel verir. Karanlıkta da çalışmaz.
- **~2 cm'den yassı nesneler** (kablo, çorap, ince kitap) 3 cm işaretleme eşiğinin altında kalır: kamera görmez, ToF zemin seviyesinin üstüne bakar. Bunlar hâlâ yalnız uçurum sensörü/tamponla, çoğu zaman hiç fark edilmez.
- **Cam ve ayna:** ToF ve stereo geçer ya da yansır. Cam sehpa tablası görünmeyebilir.
- Gövde eğimi modellenmedi: paletli robot fren/ivmede yalpalar. 1° eğim 2 m'de zemini 3.5 cm kaldırır ve sahte engel doğurabilir; kamera işaretleme menzili bu yüzden 2 m tutuldu. Gerçek robotta ölçülmeli.
- Kafa pan ±60° temas modelinde yok (pan 0 alındı). Çarpma olarak hesaplanan bantlar CAD kutularıdır, gerçek kabuk biraz farklı.
- Simülatörde engeller hareket etmez: robot oyuncağı itemez, değince durur.
- Xavier'de kamera + voxel katmanı + Nav2'nin CPU yükü ölçülmedi. Simülasyonda nokta bulutu 3072 nokta / 5 Hz; gerçek sürücüde çözünürlük azaltılmalı (`config/oakd.yaml`).
- dock_node'un son 0.6 m'lik yaklaşması maliyet haritasına bakmaz. Dock'un hemen önündeki bir eşyaya çarpar; parkurdaki eşyalar bu bölgenin dışında tutuldu.

**Tekrarlamak için** (konteynerde, "SLAM, Nav2 ve dock'a git" bölümündeki gibi; betikler stdin'den verildi)
```bash
ros2 launch luna_robot luna_nav.launch.py voice:=false world:=table obstacle_sensors:=lidar map:=/ws/maps/course_table
python3 /ws/src/luna_robot/tools/sim_nav_check.py course --world table --label lidar --map /ws/maps/course_table
```
`dock` parkuru kayıtlı `room` haritasında koşulur: `slam_mode:=localization world:=dock map:=/ws/maps/room_<kip>`.

## Ses zinciri: dokun-konuş, konuşmayı metne çevirme, Türkçe ses (25.10.2026, `ses-zinciri` dalı)

Luna artık bir soruyu baştan sona işleyebiliyor. Yüzüne dokunulur, Luna bir cümle dinler, robotun içinde metne çevirir, asistana sorar ve cevabı Türkçe seslendirir. Hiçbir parçası model eğitimi gerektirmiyor. Zincirin tamamı Docker'da, gerçek HTTP konuşan sahte sunucularla uçtan uca koşuldu. Ardından gerçek whisper modeli ve gerçek piper sesiyle Mac'te (Apple M4) denendi; ROS zinciri de Docker'dan bu gerçek sunuculara bağlanarak koşuldu (aşağıda "Gerçek modellerle deneme"). Gerçek mikrofonla ve Xavier'de henüz denenmedi.

### Akış

```
yüz (telefon) ──dokunma / basılı tutma──> face_bridge POST /talk ──/luna/voice/talk──> audio_node: 6 s dinleme penceresi
                                                                      ├──> speech_node: Luna konuşuyorsa susar
                                                                      └──> assistant_node: yoldaki eski cevabı düşürür
XVF3800 ──arecord──> audio_node ──/luna/audio/utterance (WAV)──> stt_node ──HTTP──> whisper-server (127.0.0.1:8178, tr)
stt_node ──/luna/assistant/request──> assistant_node ──/luna/assistant/reply──> speech_node ──HTTP──> piper (127.0.0.1:5000)
speech_node ──WAV──> aplay ──> XVF3800 hoparlör çıkışı
audio_node ──/luna/voice/listening──> face_bridge (meraklı yüz) ve attention_node (kafa sabit)
stt_node ──/luna/stt/status──> face_bridge (metne çevirirken "düşünüyor")
```

### Dokun-konuş

- **Hangi dokunuş:** kısa dokunma (≤ 360 ms, ≤ 14 px) ya da basılı tutma (430 ms'de). Okşama, yani parmağı kaydırmak, soru açmaz.
  - Eşikler yüzün kendi hareket eşikleriyle aynı (Telefon Yüzü R01: `gestures.js`, `app.js`). Yüz bu hareketlere kendi tepkisini vermeye devam eder.
  - Klavyede Enter ya da Boşluk da sayılır.
- **Tarayıcı tarafı** `web/robot-link.js`: `connectRobot(face, { url, talkTarget })`. `talkTarget` verilmezse eskisi gibi yalnız olay akışı okunur.
- **Telefon yüzü sayfası bu bağlantıyı henüz çağırmıyor.** R01 teslim klasörü değiştirilmedi. Yüz robota bağlanırken sayfaya şunlar eklenmeli:
  ```js
  import { lunaApp } from './app.js';
  import { connectRobot } from './robot-link.js';
  connectRobot(lunaApp.core, { url: 'http://127.0.0.1:8765/events', talkTarget: document.getElementById('face-stage') });
  ```
- **Yüz köprüsü** `POST /talk` → `/luna/voice/talk` (`std_msgs/Empty`), cevap 204.
  - Tarayıcıdan gelen ve `Origin`'i `allowed_origin` olmayan istek 403 alır: başka bir web sayfası mikrofonları açamaz.
  - `Origin`'siz istek (curl, ROS araçları) kabul edilir. Gövde en fazla 1 KB.
  - `host: 0.0.0.0` ile yerel ağdaki herkes Luna'ya bir soru dinletebilir; yüze dokunmakla aynı şey.
- **Pencere** (`core/audio.ListenWindow`):
  - Dokunmadan sonraki 6 s içinde başlayan **tek** konuşma soru olur. Başlamış konuşma, pencere kapansa da sonuna kadar alınır.
  - 0,5 s içindeki ikinci dokunma aynı dokunma sayılır.
  - 0,4 s'den kısa bir ses (öksürük gibi) dokunma hakkını harcamaz.
- **Dinlemediği durumlar:** susturulmuşken, dock'ta (Luna uyuyor) ve Luna konuşurken.
- **Dokunarak söz kesme:** Luna konuşurken dokunulursa susar, sıradaki cümleleri de atar ve dinlemeye geçer. O sırada buluttan gelmekte olan cevap düşürülür ve sohbet geçmişine yazılmaz.
- **Yüz:** pencere açıkken ve soru sürerken `curious`, metne çevirme ve bulut beklenirken de `curious`, Luna konuşurken `stroke`. Kayıt kapalıysa eski davranış sürer: dizi bir ses duyunca `curious`.
- **Kafa:** soru alınırken dikkat düğümü kafayı oynatmaz, yalnız gözler sese döner. Böylece servo sesi kayda girmez; soran kişi zaten karşıda. Soru bitince kafa kişiye döner.
- **`listen_mode: always`** her konuşmayı soru sayar (ileride uyandırma sözcüğü için). İki kipte de geçerli yeni kural: Luna konuşurken soru başlamaz, Luna kendi sesini soru sanmaz.

### Gizlilik

- `capture` artık açık (`config/luna.yaml`). Ama dokunma olmadan hiçbir ses diske yazılmaz; bellekte yalnız 0,3 s'lik ön tampon tutulur ve dokunmasız konuşmalar atılır.
- WAV, metne çevrildikten hemen sonra silinir (`delete_wav`).
- Ses ve metin robottan çıkmaz: whisper-server da piper da 127.0.0.1'de çalışır. Buluta yalnız metin gider, o da eskisi gibi yalnız `cloud:=true` verilmişse ve anahtar varsa.
- Konuşma metni loglanmaz; yalnız uzunluk, süre ve sonuç yazılır (`/luna/stt/status`: `transcribing`, `ok`, `empty`, `error:<neden>`).

### Konuşmayı metne çevirme (`stt_node`, `core/stt.py`)

- **Varsayılan motor `whisper_server`:** whisper.cpp'nin `whisper-server`'ı; model bellekte kalır.
  - İstek: `POST /inference`, çok parçalı form. Alanlar: `file` (16 kHz mono WAV), `language=tr`, `response_format=json`, `temperature=0.0`. Cevap: `{"text": …}`.
  - Kaynak: whisper.cpp `examples/server` README'si ve `server.cpp` (25.10.2026'da okundu).
- **Diğer motorlar:** `command` (ör. `whisper-cli … -f {wav}`; kabuk kullanılmaz, `{wav}` ve `{lang}` tek argüman olarak verilir) ve `none`. Bilinmeyen motor adı hata verir: bir yazım hatası Luna'yı sessizce sağır etmesin.
- **Halüsinasyon süzgeci:** Whisper sessizlikte ve gürültüde altyazı yazıları uydurur ("Altyazı M.K.", "İzlediğiniz için teşekkür ederim", "[Müzik]").
  - Bu cümleler ve köşeli ya da yuvarlak parantezli ses etiketleri atılır. Geriye bir şey kalmazsa soru gönderilmez.
  - Türkçe I/İ büyük harfleri doğru küçültülür. Sıradan "Teşekkürler" atılmaz.
- **Anlaşılamayan soru sessiz kalmaz** (`reply_on_failure`): boş sonuçta "Seni tam anlayamadım. Bir daha söyler misin?", sunucu yoksa "Şu an konuşmanı anlayamıyorum. Biraz sonra tekrar dener misin?" denir.
- **Model:** varsayılan `ggml-large-v3-turbo-q5_0.bin`; Türkçede küçük modellerden belirgin biçimde daha iyi. **Xavier'deki hızı ölçülmedi;** yavaş kalırsa önce `audio_ctx: 768`, sonra `ggml-small.bin` denenir. `timeout_s` 20 s.
- **`audio_ctx`** (varsayılan 0 = 30 s'lik tam pencere): kodlayıcının penceresini kısaltır. 768 yaklaşık 15 s'dir; en uzun soru 12 s olduğu için hepsi sığar. M4'te süreyi yarıya indirdi, doğruluk aynı kaldı (aşağıda).
- **`prompt`** (başlangıç ipucu) boş bırakıldı: "Luna" ipucu ismi düzeltmedi, `audio_ctx` ile birleşince gürültüye kelime uydurttu.

### Türkçe ses (`speech_node`, `core/speech.py`)

- **Varsayılan motor `piper_http`:** piper'ın HTTP sunucusu (`python3 -m piper.http_server`).
  - `POST /synthesize` `{"text", "length_scale"}` → WAV. WAV, `aplay`'e stdin'den verilir; örnekleme hızı WAV başlığından okunur.
  - Ses modeli bir kez yüklenir. Kaynak: OHF-Voice/piper1-gpl `docs/API_HTTP.md` (25.10.2026).
- **Tek Türkçe ses:** `tr_TR-dfki-medium`. rhasspy/piper-voices'ta fahrettin ve fettah sesleri katkıcıların isteğiyle kaldırılmış (25.10.2026).
- Konuşma hızı `tts_length_scale` ile ayarlanır; 1.1 biraz daha yavaş, çocuklar için denenebilir.
- Sunucu ya da `aplay` yoksa hata bir kez loglanır, düğüm çalışmaya devam eder ve Luna sessiz kalır.
- Ses örnekleri: `Önizlemeler/Ses_Ornekleri_2026-10-25/` (tanışma cümlesi hız 1.0 ve 1.1; ROS zincirinin söylediği "anlayamadım" ve "bulut kapalı" cevapları). Piper'ın süreleri her seferinde biraz rastgele değişir, o yüzden 1.1 her cümlede tam %10 yavaş çıkmaz.

### launch

- `luna.launch.py` artık `stt_node`'u da başlatır.
- `voice_servers:=true` (varsayılan) whisper-server'ı ve piper sunucusunu `config/luna.yaml`'daki adreslerle başlatır. Program ya da model yoksa yalnız bir log satırı yazar.
- Model yolları: `whisper_model:=/models/whisper/ggml-large-v3-turbo-q5_0.bin`, `piper_voice:=/models/piper/tr_TR-dfki-medium.onnx`.
- İki sunucu `launch_util.STOP_AS_CLEAN_EXIT` önekiyle başlar: launch'ın kapanış SIGINT'i sunucuya SIGTERM olarak gider ve süreç temiz çıkar. Sunucu kendi kendine düşerse çıkış kodu olduğu gibi görünür.
- `docker/compose.yaml`: ana makinedeki `/opt/luna/models`, konteynerde `/models` olarak salt okunur bağlanır.

### Kurulum (Jetson'da DENENMEDİ, modeller indirilmedi)

```bash
git clone https://github.com/ggml-org/whisper.cpp && cd whisper.cpp
cmake -B build -DGGML_CUDA=1 && cmake --build build -j --config Release
sh ./models/download-ggml-model.sh large-v3-turbo-q5_0 /opt/luna/models/whisper
python3 -m pip install "piper-tts[http]"
python3 -m piper.download_voices tr_TR-dfki-medium --data-dir /opt/luna/models/piper
```

- `whisper-server` PATH'te olmalı (`build/bin`).
- Mac'te deneme için kurulan: `brew install whisper-cpp` (1.9.5; bağımlılık olarak llama.cpp ve ggml de geldi), Python 3.13 sanal ortamında `piper-tts[http]` 1.8.0 (onnxruntime 1.31). Model ve ses `~/.cache/luna-voice/` altında (whisper 547 MB, SHA256 Hugging Face'teki değerle aynı; Türkçe ses 60 MB). Silmek için: `rm -rf ~/.cache/luna-voice`.
- JetPack 5'in Python 3.8'iyle `piper-tts` uyumu ve onnxruntime'ın aarch64 paketi denenmedi. Olmazsa piper ayrı bir konteynerde 5000 portunda çalıştırılır.

### Bu dalda doğrulananlar

| Kontrol | Nerede | Sonuç |
|---|---|---|
| `python3 -m unittest discover -s test` (eski 260 + yeni 37) | Mac, Py 3.15 | **297 OK**, 22 atlandı (numpy/PyYAML yok) |
| `colcon build`, `colcon test` | Docker `luna-nav:humble`, Py 3.10 | **297/297**, 0 atlanan |
| `cd web && npm test` (+4: dokunma, basılı tutma ve okşama ayrımı; `/talk` isteği; gerçek yüz çekirdeğiyle bağlantı) | Mac | **8/8** |
| `firmware/luna_mcu/host_test/run.sh` | Mac | 19 test / 246 kontrol (firmware değişmedi) |
| Testler gerçekten hata yakalıyor mu: bilerek bozulan 19 davranış | Mac | **19/19'u test kırdı.** İlk turda 2'si kaçtı (Luna konuşurken kayıt, Türkçe büyük I); testler güçlendirildi |
| Canlı zincir | Docker, `--network none`, `doa:=mock`, sahte MCU (SoC 0.60), sahte whisper-server / piper / aplay (gerçek HTTP) | aşağıda |
| Gerçek modeller: Luna'nın `core/stt.py` ve `core/speech.py` kodu gerçek whisper-server ve piper'a karşı | Mac (M4, Metal) | aşağıda, "Gerçek modellerle deneme" |
| ROS zinciri gerçek sunuculara karşı | Docker → Mac'teki whisper-server ve piper (`192.168.5.2`) | aşağıda |
| launch sunucuları başlatıyor mu | Docker, PATH'te aynı adlı sahte `whisper-server` ve `piper.http_server` | doğru argümanlar (`-m`, `-l tr`, `luna.yaml`'daki host/port, `-sns`); zincir onlarla çalıştı; kapanışta **14/14 süreç temiz**, arkada sunucu kalmadı |

Bozulan 19 davranış: pencerenin tek soruda kapanması, dokunmasız kayıt, Luna konuşurken kayıt, susturma, çift dokunma, Origin denetimi, gövde sınırı, halüsinasyon süzgeci, Türkçe I, dil alanı, kabuk birleştirme, bilinmeyen motor, sentez sırasında durdurma, WAV denetimi, okşama, basılı tutmanın iki kez tetiklemesi, ağ hatası, kafa dönünce kesilme, soru sırasında kafa dönmesi.

Canlı koşu (25.10.2026). Sahte sesler 20 s'lik bir döngüde konuşuyor (+35° 3,0–5,5 s, −30° 11,0–13,0 s):
- **A. Dokunma** (yüz sayfasının Origin'i) → 204.
  - Sahte ses 3,0–5,5 s → 3,5 s'lik WAV (0,3 ön tampon + 2,5 + 0,7).
  - whisper'a `language=tr`, `response_format=json`, 16 kHz mono gitti → istek "Merhaba Luna, bugün hava nasıl?" → asistanın çevrimdışı cevabı → piper'a iki cümle → aplay.
  - Yüz: `curious` ×2, ardından `stroke` ×2.
- **B. Luna konuşurken dokunma** → aplay aynı anda kesildi, ikinci cümle çalınmadı, `speaking` 0,2 s sonra false oldu. İkinci ses (11,0–13,0 s) 3,0 s'lik WAV oldu.
- **Kafa:** iki pencerede de hiç kafa komutu gitmedi. Kafa soru bitince, cevap sırasında kişiye döndü.
- **C.** `Origin: https://evil.example` → 403, kayıt yok.
- **D.** Dokunmadan konuşan ses → kayıt yok.
- **F.** Dock'ta (CHARGING) dokunma → "touch to talk ignored: asleep on the dock", kayıt yok.
- Koşu sonunda `/tmp/luna_utterances` boştu: WAV'lar okunduktan sonra silindi. Kapanış: launch çıkış kodu 0, 12/12 süreç temiz.
- launch, whisper-server ve piper kurulu olmadığını iki log satırıyla bildirdi; düğümler etkilenmedi.

### Gerçek modellerle deneme (Mac, 25.10.2026)

Piper (`tr_TR-dfki-medium`) on Türkçe cümleyi seslendirdi; WAV'lar robotun kaydettiği biçime (16 kHz mono) çevrildi ve whisper-server'a (`large-v3-turbo-q5_0`, Metal) Luna'nın kendi `WhisperServerStt`'siyle gönderildi. Her cümle üç biçimde: temiz, gürültülü (SNR 10 dB), uzak ve gürültülü (−20 dB, SNR 5 dB). Konuşma sentetik olduğu için sonuçlar gerçek bir insan sesinden iyimser olabilir.

| Durum | Tam doğru | Kelime hata oranı | Yazıya çevirme süresi |
|---|---|---|---|
| Temiz | 5/10 | 0.23 | ort 1.28 s (ilk istek 1.74 s) |
| Gürültü, SNR 10 dB | 5/10 | 0.24 | ort 1.23 s |
| Uzak + gürültü, SNR 5 dB | 5/10 | 0.29 | ort 1.23 s |

- **Doğru çıkanlar:** "Merhaba Luna, bugün hava nasıl?", "Saat kaç oldu?", "Yarın okula gidecek miyim?", "Işıkları kapatır mısın lütfen", "Çok güzel bir şarkı söyle bana" üç biçimde de harfi harfine doğru.
- "Bir, iki, üç, dört, beş" → "1, 2, 3, 4, 5": anlam doğru, ölçü hata sayıyor.
- **"Luna" adı** 9 cümlenin 4'ünde doğru yazıldı; diğerlerinde "Yuna", "Juna", "Yunan", "buna". İpucu ("Luna" ya da "Luna ile konuşuyorum.") bunu düzeltmedi (4/9 ve 5/9). Dokun-konuş için önemli değil: soru yine anlaşılır. İleride uyandırma sözcüğü için whisper'a güvenilmemeli.
- Diğer hatalar: "mırlar" → "varlar", "Ağaçlar kışın" → "Ağaç daha kışın".
- **Sessizlik ve gürültü:** 3 s sessizlik, hafif gürültü (−45 dBFS), gürültü (−25 dBFS) ve 0,5 s'lik tık sesinin dördüne de whisper tam olarak "Altyazı M.K." yazdı; süzgeç dördünü de attı (sonuç `empty`).
- **`audio_ctx 768`:** ortalama süre 1.25 s'den 0.62 s'ye indi, kelime hata oranı 0.251'e karşı 0.256, sessizlikte uydurma yok. İpucuyla birlikte kullanıldığında gürültüye "İzgur" ve "İngilizce" yazdı (süzgeçten geçer); bu yüzden ipucu kullanılmıyor.
- **Piper:** bir cümleyi ortalama 0.09 s'de (en çok 0.33 s) üretti; cümleler ortalama 2 s.
- **ROS zinciri gerçek sunucularla** (Docker konteyneri Mac'teki sunuculara `192.168.5.2` üzerinden bağlandı, sunucular yalnız 127.0.0.1'de dinledi):
  - Dokunma → sahte sesin sessiz kaydı → whisper "Altyazı M.K." → `empty` (1.24 s) → piper sesiyle "Seni tam anlayamadım. Bir daha söyler misin?".
  - Gerçek konuşma WAV'ı "Merhaba Luna, bugün hava nasıl?" → harfi harfine doğru (1.21 s) → asistanın çevrimdışı cevabı → piper; cevaptan ilk sese 0.07 s.
  - Uzak ve gürültülü "Işıkları kapatır mısın lütfen" → doğru (1.22 s).
  - WAV'lar okunduktan sonra silindi; launch çıkış 0, 12/12 süreç temiz.
- Mac'te gerçek whisper-server ve piper, SIGINT'te ikisi de 0 koduyla kapandı.

### Bulunan ve düzeltilen sorunlar

1. **Kafa dönünce soru kesiliyordu (eski bir hata).**
   - Konuşma başlayınca dikkat düğümü 0,3 s sonra kafayı sese çeviriyor. Mikrofon kapısı hareket sırasında konuşma bayrağını 0 yapıyor; kayıt bunu sessizlik sayıp 0,7 s sonra bitiriyordu.
   - İlk canlı koşuda 2,5 s'lik soru 1,4 s'de kesildi. `ses-bulut` dalındaki "başlamış olan biter" kuralı da bu yüzden fiilen işlemiyordu.
   - Düzeltme: hareket süresi artık ne konuşma ne sessizlik sayılıyor (`Segmenter.feed(freeze=…)`; dikkat düğümünün donma kuralıyla aynı). Ayrıca dokunarak sorulan soru sırasında kafa hiç dönmüyor.
2. **Luna kendi sesini soru sanabilirdi.** Eskiden Luna konuşurken de kayıt başlayabiliyordu; `always` kipinde kendi cevabı metne çevrilip buluta gidebilirdi. Artık Luna konuşurken soru başlamıyor.

### Bu dalda doğrulanmayanlar

- **Xavier'de hiçbir şey çalıştırılmadı.** large-v3-turbo'nun Xavier GPU'sundaki gecikmesi (M4'te 1.2 s) ve piper'ın Xavier CPU'sundaki hızı bilinmiyor.
- Gerçek insan sesiyle tanıma denenmedi; ölçümler piper'ın sentetik sesiyle yapıldı.
- Gerçek mikrofon, XVF3800'ün yankı engelleyicisi ve servo ile LiDAR gürültüsünün tanımaya etkisi denenmedi.
- Telefon yüzü sayfası `connectRobot`'u henüz çağırmıyor. Gerçek dokunma yalnız birim testlerinde ve sahte HTTP isteğiyle denendi, tarayıcıda denenmedi.
- Jetson imajı ve yukarıdaki kurulum komutları denenmedi.
- **Kapanış:** bu daldaki 12 kapanışın birinde (ilk uzun canlı koşu) `assistant_node` kapanırken -11 verdi. Sonraki 11 kapanışta (6'sı soru-cevapsız, 5'i soru-cevaplı) görülmedi. Önceki bölümlerdeki seyrek Humble kapanış sorunlarıyla aynı olabilir; neden bulunamadı.

### Açık işler

- **Uyandırma sözcüğü ("Luna"):** hazır model yok, openWakeWord ile sentetik sesle **eğitim** gerekir. O zamana kadar dokun-konuş kullanılır.
- Telefon yüzüne `connectRobot(… talkTarget …)` satırının eklenmesi (yüz teslimi seçildiğinde).
- Xavier'de uçtan uca deneme: whisper modeli (turbo ya da small), `audio_ctx` ve `timeout_s` seçimi; gerçek mikrofonla ve gerçek insan sesiyle tanıma.
- Söz kesme yalnız dokunmayla var; Luna konuşurken kişinin konuşmaya başlamasıyla susma yok.
- Mute durumunun yüzde gösterilmesi ve `/luna/head/target` için öncelik düzenleyici hâlâ açık.

### Tekrarlamak için

Konteynerde, "SLAM, Nav2 ve dock'a git" bölümündeki gibi (`colcon build` kurulu `luna.yaml`'ı geri getirir):
```bash
bash /ws/src/luna_robot/tools/voice_check/live.sh short
```
Kayıtlar `/tmp/voice_check`'e yazılır. `short` olmadan C–F adımları da koşulur (yaklaşık 2 dakika).

## Yönlendirici: önce hızlı model, zor soruda düşünen model (25.10.2026)

Kimi Code üyeliğiyle aynı model iki biçimde kullanılır (`core/router.py`, `assistant_node`):

| Yol | Model | Ne zaman |
|---|---|---|
| hızlı | `kimi-for-coding`, düşünme kapalı (düşünmesiz K2.8 Preview), en fazla 300 token | her soru önce buraya |
| düşünen | aynı model, düşünme açık, `reasoning_effort: "low"`, en fazla 4000 token (düşünme dahil), 30 s / 45 s | kural ön elemesi ya da hızlı modelin `[DÜŞÜN]` cevabı |

- **Bedava kural ön elemesi:** soru doğrudan düşünen modele gider, hızlı modele hiç sorulmaz.
  - **hesap:** iki sayı arasında işlem ("12 çarpı 7", "5+3"; "-" ve ":" yalnız boşluklu sayılır, "2-3 gün" ve "saat 7:30" sayılmaz), sözle "artı / çarpı / bölü", "kaç eder", "hesapla".
  - **ödev:** "ödev", "denklem", "kesir", "problemi çöz".
  - **açıklama:** "nasıl çalışır", "açıkla", "arasındaki fark", "adım adım".
  - **uzun:** 25 kelimeden uzun soru.
  - "Kediler neden mırlar?" gibi sorular bilerek hızlı yolda kalır: çocukların sorularının çoğu "neden" diye başlıyor.
- **`[DÜŞÜN]`:** hızlı modelin sistem istemine "Soru dikkatli düşünmeyi gerektiriyorsa (bir hesap, ödev, birkaç adımlı bir açıklama ya da emin olmadığın bir bilgi) cevap verme, yalnız [DÜŞÜN] yaz." eklenir. Bu cevap gelirse soru düşünen modele aktarılır. İşaret sohbet geçmişine yazılmaz, düşünen model hata verse bile.
- **"Hmm, bir düşüneyim.":** düşünen modele geçmeden hemen önce söylenir (`think_filler`); bekleme sessiz geçmez. Onay, anahtar ya da açık devre kesici yüzünden düşünen modele sorulamayacaksa söylenmez; Luna doğrudan çevrimdışı cevabı verir.
- İki istemci **tek sohbet geçmişi** paylaşır: düşünen modelin cevabından sonraki hızlı soru o cevabı görür. Dokunmayla düşürülen cevap iki geçmişten de silinir.
- Log yalnız yolu ve nedeni yazar (`fast ok`, `think (hesap) ok`, `think (model) ok`), soru ve cevap metnini yazmaz.
- `router_enabled: false` eski tek modelli davranışa döner. `think_model` ile düşünen yol başka bir modele (ör. `k3`) çevrilebilir; Kimi Code sayfasına göre o kotayı daha çok harcar.

**Doğrulananlar**
- Birim testleri (`test/test_router.py`, 11 test): kurallar, işaret biçimleri, istek alanları (düşünme kapalı / `reasoning_effort`, token sınırları, sistem istemi), dolgu cümlesinin ne zaman söylendiği, paylaşılan geçmiş, düşürülen cevap. Bilerek bozulan 9 davranışın 9'unu testler kırdı (ilk turda biri kaçtı: düşünen model hata verince işaretin geçmişte kalması; test eklendi).
- Mac'te 308 test (22 atlandı), Docker `colcon test` 308/308.
- Canlı yığın, Docker, `cloud:=true`, sahte Kimi sunucusu (`tools/voice_check/router_check.sh`):
  - "Merhaba Luna" → hızlı yol.
  - "12 çarpı 7 kaç eder?" → `think (hesap)`, hızlı modele istek yok.
  - "Gökkuşağı neden oluşur?" → hızlı `[DÜŞÜN]` → `think (model)`.
  - piper'a sırayla: cevap, "Hmm, bir düşüneyim.", cevap. Geçmiş 0 → 2 → 4 mesaj. 12/12 süreç temiz kapandı.

**Doğrulanmayanlar**
- Gerçek Kimi Code API'siyle denenmedi (Luna için anahtar yok). `reasoning_effort` ve `thinking` alanlarının bu uç noktada aynen kabul edildiği, gerçek modelin `[DÜŞÜN]` talimatına ne sıklıkla uyduğu ve düşünen yolun gerçek süresi ölçülmedi.
- Kural listesi tahmin; gerçek konuşmalarda hangi soruların düşünen yola düştüğü loglardan izlenip ayarlanmalı.

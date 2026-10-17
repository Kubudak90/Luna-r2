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
    dock_nav.py           "dock'a git": Nav2 ile dock öncesi poza, sonra dock_node'a devir (aşağıda)
    sim2d.py              2B oda simülatörü: LiDAR, tamponlar, dock ve etiket, kayan paletler
    face_mood.py          robot olayları → yüz olayları (tap/stroke/curious/sleep/wake/gaze)
  luna_robot/*_node.py  ince ROS 2 düğümleri: base, head, dock, face_bridge, mock_mcu, sim2d, dock_nav
  launch/luna.launch.py   mock:=true (varsayılan) | sim:=true | mock:=false lidar:=true
  launch/luna_nav.launch.py  luna + slam_toolbox + Nav2 + dock_nav (slam.launch.py, nav2.launch.py)
  config/luna.yaml        CALIBRATE işaretli değerler gerçek robotta ölçülecek
  config/slam_*.yaml, nav2.yaml  harita/yerelleştirme ve navigasyon ayarları
  urdf/luna.urdf          R05 CAD'den sadeleştirilmiş çerçeveler
  tools/sim_nav_check.py  simülatörde uçtan uca senaryo denetimi (Docker'da)
firmware/luna_mcu/       Teensy 4.1 micro-ROS iskeleti (pins.h pin haritası)
web/robot-link.js        yüz PWA'sını robota bağlayan EventSource adaptörü
docker/                  dustynv/ros:humble-ros-base-l4t-r35.4.1 tabanlı imaj
```

## Topic sözleşmesi

Teensy ile AGX arasındaki topic'lerin tek kaynağı `luna_robot/topics.py`. `firmware/luna_mcu/src/topics.h` bu dosyayla birebir aynı tutulmalı.

| Yön | Topic | Tip |
|---|---|---|
| MCU → | `/luna/mcu/wheels` | Float32MultiArray [sol rad, sağ rad, sol rad/s, sağ rad/s] |
| MCU → | `/luna/mcu/bumpers` · `/estop` · `/dock_sense` | UInt8 bit maskesi · Bool · Bool |
| MCU → | `/luna/mcu/cliff` · `/corners` | Float32MultiArray (m) |
| MCU → | `/luna/mcu/battery` | sensor_msgs/BatteryState (BMS UART) |
| → MCU | `/luna/mcu/wheel_target` | [sol rad/s, sağ rad/s, pwm_cap] |
| → MCU | `/luna/mcu/head_us` | [pan µs, tilt µs] |
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
- **SLAM ve navigasyon:** `slam-nav2` dalında eklendi; ayrıntı ve sınırlar aşağıda, "SLAM, Nav2 ve dock'a git" bölümünde.
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

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
    face_mood.py          robot olayları → yüz olayları (tap/stroke/curious/sleep/wake/gaze); dinliyor/konuşuyor
    xvf3800.py            ReSpeaker XVF3800 USB kontrol protokolü (DoA), ALSA kart bulma
    audio.py              mikrofon PCM'i: kanal seçme, seviye, konuşmayı kayıt parçalarına (WAV) bölme
    attention.py          ses yönü → gözler hemen, kafa yavaşça o yöne; dock'ta ve şarjda kurallı
    assistant.py          sağlayıcıdan bağımsız bulut asistanı istemcisi (Kimi varsayılan), çevrimdışı yedek
    speech.py             cevabı seslendirme: metni temizleme, cümle kuyruğu, takılabilir TTS
  luna_robot/*_node.py  ince ROS 2 düğümleri: base, head, dock, face_bridge, mock_mcu,
                          audio, attention, assistant, speech
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
| MCU → | `/luna/mcu/bumpers` · `/estop` · `/dock_sense` | UInt8 bit maskesi · Bool · Bool |
| MCU → | `/luna/mcu/cliff` · `/corners` | Float32MultiArray (m) |
| MCU → | `/luna/mcu/battery` | sensor_msgs/BatteryState (BMS UART) |
| → MCU | `/luna/mcu/wheel_target` | [sol rad/s, sağ rad/s, pwm_cap] |
| → MCU | `/luna/mcu/head_us` | [pan µs, tilt µs] |
| AGX | `/cmd_vel`, `/luna/dock/request` (dock/undock/cancel), `/luna/head/target` | |
| AGX | `/luna/head/state` | Vector3 [pan°, tilt°], hız sınırından sonraki gerçek komut |
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
- **AprilTag düğümü:** bu pakette yok. `apriltag_ros` bağlanıp etiket ölçümü `/luna/dock/tag` topic'ine (PointStamped: x = yatay sapma, z = mesafe) yayımlanacak.
- **SLAM ve navigasyon:** bağlanmadı. Sıradaki adaylar slam_toolbox ve Nav2; LiDAR verisi `laser` çerçevesinde hazır.
- **USB kameralar:** Arducam UVC kameralar için `usb_cam` ya da `v4l2_camera` eklenecek.
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

- **Konuşmayı metne çevirme eklentisi:** `/luna/audio/utterance` → `/luna/assistant/request`. Adaylar: AGX'te whisper, ya da sağlayıcının STT'si (ayrı onay ister).
- **Uyandırma sözcüğü ya da bas-konuş:** şu an her konuşma "dinliyor" sayılıyor. Buluta yalnız Luna'ya yöneltilen sorular gitmeli.
- **Mute durumu dışarıdan görünmüyor:** kart düğmesinin (GPI X1D09) okunması belgelerde çelişkili (Seeed sayfası iki farklı okuma veriyor), okunmadı. Robotun kendisi de susturulduğunu yüzde göstermiyor; kırmızı LED kafa içinde kalıyor, kabuktaki LED deliklerinden görünüp görünmeyeceği bilinmiyor.
- **Söz kesme (barge-in):** Luna konuşurken kişi konuşmaya başlarsa susmuyor.
- **`/luna/head/target` tek sahipli değil:** kamera/takip düğümleri de kafayı sürecekse bir öncelik düzenleyici gerekir. Şu an yalnız dikkat düğümü yayımlıyor, o da yalnız kafayı kullandığı sürece.

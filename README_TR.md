# Luna R2 — robot yazılımı ve tasarımı (25.10.2026)

Luna, çocuklar için bir ev arkadaşı robotu. R2, R1'in yeni tasarımı:
- **Yüz:** kafadaki 5,5" AMOLED ekranda çalışıyor, telefon gerekmiyor. Çocuk ekrana dokununca Luna dinliyor.
- **Kafa:** sabit. Üstünde LiDAR, ekranın üstünde tek küçük kamera var.
- **Gövde:** Ø 36 cm yuvarlak. Ortada iki tahrikli teker, önde ve arkada birer sarhoş teker.
- **Parçalar:** hepsi Türkiye'deki satıcılardan alınabiliyor.

| Belge | İçerik |
|---|---|
| `docs/TASARIM_R2.md` | Tasarım ve gerekçeleri: şasi, kafa ve yüz, algılama, uçurum, ses, gizlilik, güç, elektrik, dock |
| `docs/MALZEME_LISTESI.md` | Türkiye malzeme listesi: satıcı, fiyat, stok (25.10.2026). Toplam ~155–160 bin TL |
| `firmware/luna_mcu/README_TR.md` | Teensy 4.1 yazılımı |
| `docs/R1_README_TR.md` | R1'in belgesi. Ses zinciri, bulut asistanı ve SLAM'in ayrıntılı geçmişi orada |

**Hiçbir parça henüz alınmadı; hiçbir şey gerçek donanımda denenmedi.** Yazılım simülatörde, testlerde ve derlemede doğrulandı. Ayrıntısı aşağıda, "R2 doğrulama" bölümünde.

## R2'de ne var

| | Parça | Yazılımda |
|---|---|---|
| Bilgisayar | Jetson Orin NX 16 GB (Waveshare Super kit, JetPack 6.2) | ROS 2 Humble, Docker (`docker/`) |
| Yüz | Waveshare 5,5" AMOLED, HDMI, dokunmatik | `web/face.html` + `face.js` + `face-core.js`, `face_bridge_node`, `tools/kiosk.sh` |
| Yüz kamerası | Raspberry Pi Camera v2, CSI | `face_track_node` (YuNet) → bakış ve gövde dönüşü |
| LiDAR | RPLIDAR C1, kafanın tepesinde, Z550 | `sllidar_ros2` → slam_toolbox, Nav2 |
| Derinlik + renk + IMU | Orbbec Gemini 336, gövde önünde, 15° aşağı | OrbbecSDK_ROS2 v2.9.3 → Nav2 voksel katmanı, dock etiketi, jiroskop |
| Mikrofon ve hoparlör | ReSpeaker Mic Array v2.0 (gövdeye sabit), 3,5 mm → amfi → hoparlör | `audio_node`, `attention_node`, `stt_node` (whisper.cpp), `speech_node` (Piper) |
| Yürüyüş | 2 × Pololu 37D 70:1, G2 18v18 sürücü, 2 sarhoş teker | `core/kinematics.py`, `base_node` (jiroskopla yön) |
| Alt seviye | Teensy 4.1 + micro-ROS | `firmware/luna_mcu` |
| Engel ve güvenlik | Tampon halkası (4 anahtar), 4 köşe ve 4 zemin VL53L1X, acil durdurma | `base_node` güvenlik kapısı, `tof_range_node` |
| Güç | 4S2P LiFePO4 32700 (154 Wh), Daly BMS | `core/battery.py`, `dock_node` |
| Dock | Teker hunisi, alttan yaylı pimler, AprilTag direği | `dock_node`, `dock_tag_node`, `dock_nav_node` |

## R2 doğrulama (25.10.2026)

| Kontrol | Nerede | Sonuç |
|---|---|---|
| Python birim testleri | Linux, Python 3.13 | **311/311** (`python3 -m unittest discover -s test`) |
| `colcon build`: OrbbecSDK_ROS2 v2.9.3, sllidar_ros2, luna_robot | Docker, x86 `ros:humble-ros-base` + Nav2, slam_toolbox, apriltag_ros | 4 paket derlendi |
| `colcon test` | aynı konteyner | **311 test, 0 hata, 0 atlanan** |
| Web yüzü (`face-core`, `robot-link`) | Node 22 | **21/21** (`cd luna_ws/src/luna_robot/web && npm test`) |
| Yüz, gerçek köprüden uçtan uca | Python yüz köprüsü (ROS taklit) → SSE → Chromium | 9 durumun hepsi doğru çizildi, sayfa hatası 0. Uyurken dokunma `/talk`'a gitmedi, uyanıkken gitti |
| Firmware host testleri | Linux, GCC, ASan + UBSan, `-Werror` | **19 test, 188 kontrol, 0 hata** |
| Teensy 4.1 derlemesi | PlatformIO 6.1.18, micro-ROS Humble kaynaktan | **Başarılı, 0 uyarı.** FLASH 117 520 + 15 544 B, RAM1 37 280 B |
| Kamerayla dock, `tags:=sim` | Docker: sahte MCU → simüle Gemini görüntüsü → gerçek `apriltag_ros` → `dock_tag_node` → `dock_node` | Düz başlangıç: 19,1 s'de CHARGING, son poz 0 mm. 25° açılı ve 8 cm yana kaymış başlangıç: 17,0 s'de CHARGING, yanal 21 mm (huni ±45 mm). Etiket 10 Hz |
| Gemini sürücüsü luna.launch'tan | Docker, `depth:=gemini`, kamera yok | Sürücü bileşeni doğru argümanlarla yüklendi; USB aygıtı olmadığı için durdu ("Usb pal is not exist"). Yığının geri kalanı çalıştı. URDF'in iki optik çerçevesi sürücüye bırakıldı |
| Engel parkurları (`sim_nav_check.py course`, `obstacle_sensors:=all`) | Docker, 2.5B simülatör + slam_toolbox + Nav2 | aşağıda, "Engel algılama" |
| Haritalama, yerelleştirme, dock'a git | aynı | aşağıda, "SLAM, Nav2 ve dock'a git" |

**Doğrulanmayanlar:**
- Gerçek donanım. Hiçbir parça alınmadı.
- Robot imajı (`docker/Dockerfile`, `l4t-jetpack:r36.4.0`) Jetson'da derlenmedi. ROS kısmı x86'da aynı paketlerle derlendi. whisper.cpp'nin CUDA derlemesi ve CSI kamera (nvarguscamerasrc) konteynerde denenmedi.
- Gerçek Kimi Code API'siyle çağrı yapılmadı (R1'deki gibi).
- Ölçülecekler ve açık riskler `docs/TASARIM_R2.md` belgesinin sonunda. En önemlisi uçurum için fren mesafesi ("Uçurum").

## Yapı

```
luna_ws/src/luna_robot/
  luna_robot/core/      ROS'tan bağımsız, test edilen mantık
    kinematics.py         diferansiyel sürüş, odometri + jiroskop, jiroskop sapmasını öğrenme, PWM sınırı
    attention.py          ses ve yüz yönü → gözler hemen, gövde yavaşça (30°/s), dock'ta ve soru alırken kurallı
    face_target.py        yüz kamerası pikselleri → base_link yönü, konuşan kişiyi seçme
    face_mood.py          robot durumu → yüz olayları (dinliyor/düşünüyor/konuşuyor, pil, susturma, bakış ...)
    safety.py             acil durdurma, tampon, uçurum, komut zaman aşımı kapısı
    battery.py            LiFePO4: BMS SoC'u, gerilim yedeği; dock arama / kritik / şarj kararları, Orin güç modu
    dock.py, dock_nav.py  dock durum makinesi; Nav2 ile dock öncesi poza, sonra devir
    dock_tag.py           AprilTag → /luna/dock/tag (yön, oturma noktasına mesafe)
    sim2d.py, sim_worlds.py   2.5B simülatör: yuvarlak gövde, tampon halkası, ToF konileri, R2 dock'u, parkurlar
    depth_camera.py       Gemini 336 derinlik simülasyonu
    respeaker.py, audio.py    ReSpeaker USB kontrolü (DoA), PCM, gizlilik sürgüsü
    stt.py, speech.py, assistant.py, router.py   konuşma → metin, Türkçe ses, Kimi istemcisi, hızlı/düşünen yönlendirici
    camera_model.py, frames.py, robot_model.py, tag36h11.py, tag_scene.py, tof.py
  luna_robot/*_node.py  ince ROS 2 düğümleri
  launch/luna.launch.py      mock:=true (varsayılan) | sim:=true | mock:=false lidar:=true; depth:= tags:= face:= voice:= cloud:=
  launch/luna_nav.launch.py  + slam_toolbox + Nav2 + dock_nav; obstacle_sensors:=all|lidar world:=
  config/                    luna.yaml (CALIBRATE işaretliler ölçülecek), nav2*.yaml, slam_*.yaml, gemini.yaml, apriltag.yaml, camera_info/
  urdf/luna.urdf             R2'nin bütün ölçüleri burada
  web/                       ekran yüzü ve köprü bağlantısı (node --test)
  tools/                     kiosk.sh, sim_nav_check.py, dock_watch.py, tag_sweep.py, make_tag_svg.py, voice_check/
  test/                      Python testleri
firmware/luna_mcu/           Teensy 4.1 micro-ROS (pins.h, Daly BMS, VL53L1X, sürüş döngüsü), host_test/
docker/                      Dockerfile (JetPack 6), compose.yaml, Dockerfile.dev
docs/                        TASARIM_R2.md, MALZEME_LISTESI.md, dock etiketi SVG, R1 belgesi
```

## Topic sözleşmesi

Teensy ile Orin arasındaki topic'lerin tek kaynağı `luna_robot/topics.py`; `firmware/luna_mcu/src/topics.h` ile birebir aynı tutulmalı.

| Yön | Topic | Tip |
|---|---|---|
| MCU → | `/luna/mcu/wheels` | Float32MultiArray [sol rad, sağ rad, sol rad/s, sağ rad/s] |
| MCU → | `/luna/mcu/bumpers` · `/estop` · `/dock_sense` | UInt8 (bit 0 FL, 1 FR, 2 BL, 3 BR) · Bool · Bool |
| MCU → | `/luna/mcu/cliff` [FL, BL, FR, BR] · `/corners` [FL, FR, BL, BR] | Float32MultiArray (m); 0 = okuma yok = uçurum |
| MCU → | `/luna/mcu/battery` | sensor_msgs/BatteryState (Daly BMS, LIFE) |
| → MCU | `/luna/mcu/wheel_target` | [sol rad/s, sağ rad/s, pwm_cap] |
| → MCU | `/luna/mcu/charge_enable` | Bool: false = DOCK_INHIBIT, 1 s geçerli |
| Hız | `/cmd_vel` (Nav2, teleop) · `/luna/dock/cmd_vel` · `/luna/attention/cmd_vel` | Twist; öncelik dock > teleop/Nav2 > dikkat; hepsi güvenlik kapısından geçer |
| Gemini | `/camera/depth/points` · `/camera/color/image_raw` · `/camera/gyro/sample` | PointCloud2 · Image · Imu |
| Yüz | `/luna/vision/face` → `/luna/face/gaze` | Vector3 (yön°, yükseklik°, piksel) → Vector3 (−1..1, +x Luna'nın solu, +y yukarı) |
| Ses | `/luna/audio/doa` · `/mute` → `/muted` · `/luna/privacy` | DoA (base_link°, konuşma, ham açı, gövde dönüyor) · Bool · Bool (sürgü kapalı) |
| Ses | `/luna/voice/talk` → `/luna/audio/utterance` → `/luna/assistant/request` → `/reply` | dokun-konuş zinciri (R1 belgesi, "Ses zinciri") |
| Dock | `/luna/dock/request` · `/state` · `/tag` · `/luna/nav/dock` | String · String · PointStamped · String (go/cancel) |

## Güvenlik katmanları

1. **Donanım:** acil durdurmanın 1. kontağı motor sürücünün SLP hattını doğrudan keser. `DRV_SLP`'deki 10k direnç, Teensy sıfırlanırken sürücüyü uyutur.
2. **Teensy:** acil durdurma basılıysa, sürücünün FLT pini düşükse, 300 ms hız komutu gelmezse ya da micro-ROS agent yoksa motorlar durur. 0,5 s'lik donanım watchdog'u var.
3. **Orin güvenlik kapısı (`base_node`):**
   - Basılı tampon yönüne gidilmez.
   - Zemini görmeyen yöne gidilmez. Okuma yoksa da uçurum sayılır.
   - Robot havaya kaldırılmışsa hiçbir hareket olmaz.
   - Açılışta, MCU'dan ilk mesaj gelene kadar acil durdurma basılı sayılır.
4. **PWM sınırı:** pwm_cap = 12 V / paket gerilimi. BMS yokken gerilim 0 gelir, pwm_cap da 0 olur ve robot yürümez.
5. **Dock pimleri:** robot oturmadıkça gerilimsiz (P4 kimliği).
6. **Gizlilik sürgüsü:** donanımdan mikrofonu keser, kamerayı örter.

## Çalıştırma

**Jetson'da (Orin NX, JetPack 6.2), depo kökünden:**
```bash
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml up luna                          # gerçek robot
docker compose -f docker/compose.yaml --profile mock up luna-mock      # donanımsız: sahte MCU kendiliğinden dock'a gider
```

İlk kurulumda yapılacaklar:
- **Orbbec udev kuralları ana makineye kurulur:** OrbbecSDK_ROS2 deposundaki `orbbec_camera/scripts/install_udev_rules.sh`.
- **Modeller `/opt/luna/models` altına konur** (konteynerde `/models`):
  - `whisper/ggml-large-v3-turbo-q5_0.bin` (whisper.cpp'nin `models/download-ggml-model.sh large-v3-turbo-q5_0`)
  - `piper/tr_TR-dfki-medium.onnx` (`python3 -m piper.download_voices tr_TR-dfki-medium`)
  - `face/face_detection_yunet_2023mar.onnx` (opencv_zoo, `models/face_detection_yunet`) — bkz. "Yüz kamerası"
- **Bulut asistanı:** `KIMI_API_KEY` ana makinenin ortamında olmalı ve `cloud:=true` verilmeli. Anahtar hiçbir zaman depoya ya da `compose.yaml`'a yazılmaz. Kurallar R1 belgesinde, "Bulut asistanı ve anahtar".
- **Yüz:** Orin'in masaüstü oturumunda `tools/kiosk.sh` otomatik başlatılır. Ekranı ve dokunmatiği 90° çevirir, köprü hazır olunca tam ekran tarayıcıyı açar.

**Geliştirme bilgisayarında yüze bakmak için:**
```bash
ros2 launch luna_robot luna.launch.py          # sahte MCU ile
# tarayıcıda http://127.0.0.1:8765/
```

## Yüz kamerası

`face_track_node` iki şeyle çalışır:
- **Kamera:** `device:=auto` ile CSI kamera (nvarguscamerasrc). Bir sayı ya da `/dev/videoN` verilirse USB kamera.
- **Model:** OpenCV'nin `FaceDetectorYN`'i (YuNet). `face_detection_yunet_2023mar.onnx` dosyası opencv_zoo deposundan indirilip `/opt/luna/models/face/` altına konur.

Biri eksikse düğüm nedenini loglar ve hiçbir şey yayımlamaz; gözler kendi kendine bakınır. Görüntü kaydedilmez ve robottan çıkmaz. Kamera iç parametreleri `config/camera_info/face_camera.yaml` dosyasında; bu değerler nominal, kalibre edilecek.

## Engel algılama

LiDAR düzlemi Z550. Bunun altındaki engelleri şunlar görüyor:
- **Gemini 336 derinliği:** Nav2'nin voksel katmanında yerden 3–58 cm arası engel sayılıyor.
- **4 köşe ToF:** `tof_range_node` ile ayrı bir engel katmanı.
- **Tampon halkası:** son çare.

`obstacle_sensors:=lidar` verilirse yalnız LiDAR kullanılır. Ayarlar `config/nav2_obstacle_sensors.yaml` dosyasında.

**Köşe ToF'ları:** 16 × 4'lük ROI'nin firmware'de hâlâ ayarlanmadığı R1'den kalan açık bir iş. ROI olmazsa sensör zemini 0,43 m'de görür ve `tof.py`'nin 0,70 m'lik menzili yanlış olur.

**Parkur sonuçları** (Docker, `obstacle_sensors:=all`, her biri tek koşu, simülatörün gerçek konumuyla ölçüldü):

| Dünya | Beklenen | Sonuç | Süre | Temas | En yakın geçiş |
|---|---|---|---|---|---|
| `toy` (8 cm oyuncak kutusu yolda) | hedefe varmak | **vardı** | 20,5 s | 0 | kutuya 34 cm |
| `table` (sehpa, üstü Z40–45) | vardı | **vardı** | 31,6 s | 0 | ayağa 30 cm, üstüne 25 cm |
| `chair` (sandalye) | vardı | **vardı** | 21,4 s | 0 | ayağa 30 cm, oturağa 27 cm |
| `door` (70 cm kapı, puf kapatıyor) | Nav2 vazgeçmeli | **vazgeçti** | 31,2 s | 0 | |
| `door70` (70 cm açık kapı) | vardı | **vardı** | 19,0 s | 0 | R1'in paleti bu kapıdan geçemiyordu |
| `door60` (60 cm açık kapı) | vardı | **2 koşunun 1'inde vardı** | 21,4 s | 0 | Sınırda. Düşen koşuda planlayıcı açıklıktan yol bulamadı (aşağıda) |
| `dock` (kayıtlı haritada dock'a git, yolda ayakkabı kutusu, top, oyuncak araba) | dock'a oturmak | **oturamadı** (1 koşu) | 240 s | 0 | Kutuya 7,8 cm (aşağıda) |

## SLAM, Nav2 ve dock'a git

R1'deki gibi: slam_toolbox ile harita, Nav2 (NavFn + Regulated Pure Pursuit, hız düzleştirici, çarpışma izleyici) ve `dock_nav`. Nav2 `/cmd_vel`'e yalnız çarpışma izleyicisinden ulaşır; sürüşü her zaman `base_node`'un güvenlik kapısı yapar. R2'de değişenler:
- **Ayak izi daire** (`robot_radius` 0,18). Planlayıcının payı 0,04 m; R1'de dönüş payı için 0,14 m'ydi.
- **Odometri:** tekerler ve Gemini jiroskobu.
- **Dock öncesi poz:** dock ekseninde 0,6 m geride. Devir konisi `docs/TASARIM_R2.md`, "Dock" bölümünde.

**Senaryo sonuçları** (Docker, 4 çekirdek, her biri tek koşu):

| Senaryo | Sonuç | Ayrıntı |
|---|---|---|
| Haritalama (`sim_nav_check.py mapping`) | **17/17** | Acil durdurma Nav2 sürerken motor hedefini 0,03 s'de sıfırladı, robot 0,42 s'de durdu. Dolabın arkasındaki keşif hedefine vardı; SLAM hatası 2,2 cm. Dock'a git → devir → CHARGING 22,9 s, dock'ta 6 mm. Dock pozunu duvardan öğrendi (gerçeğinden 2,5 cm, 0,4°). Harita ve poz grafı kaydedildi |
| Yerelleştirme + düşük pil (`localization`, `sim_soc:=0.24`) | **12/12** | Kayıtlı haritada başlangıç hatası 0,6 cm. Düşük pil dock'a gidişi kendisi başlattı, CHARGING 26,9 s, dock'ta 18 mm, politika `charge`'a geçti |

**Açık iki sınır durum:**
- **60 cm kapı:**
  - Kâğıt üstünde açık: 2 × 0,22 m dolgulu yarıçap + bir 0,05 m hücre = 0,49 m < 0,60 m.
  - Ama duvar uçları haritada bir hücre kalın çıkınca açıklık bir hücreye iniyor ve gürültü onu da kapatabiliyor.
  - 70 ve 80 cm kapılarda sorun yok. Evdeki iç kapılar çoğunlukla 70 cm'den geniş.
- **Dock yolunda alçak eşyalar:**
  - Kutu ile kuzey duvarı arasındaki 0,68 m'lik geçidi R2 açık görüyor (R1 dolgu yüzünden kapalı görüyordu) ve oradan gitmeye çalışıyor.
  - RPP kutunun yanında 20 kez "collision ahead" diyip durdu. İlerleme olmayınca kurtarma–yeniden planlama döngüsüne girdi, 240 s'de vazgeçti. Temas olmadı.
  - Ayar sürüyor (planlayıcı payı, RPP'nin çarpışma bakışı). R1'in bu parkurdaki çözümü `docs/R1_README_TR.md`, "Dock yolunda ayakkabı kutusu".

İlk koşu setinde bir önceki denemeden arkada kalan bir yığın (iki `luna_base`) haritalamaya karıştı. O koşular geçersiz sayıldı, temiz konteynerde yeniden koşuldu; yukarıdaki sonuçlar yeniden koşulanlar.

## Tekrarlamak için

Docker'da, depo kökünden (x86 ya da arm64, `ros:humble-ros-base`):
```bash
docker run -d --name luna-r2-ws --init -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=73 -v "$PWD:/repo:ro" ros:humble-ros-base sleep infinity
docker exec -it luna-r2-ws bash
apt-get update && apt-get install -y ros-humble-slam-toolbox ros-humble-nav2-controller ros-humble-nav2-planner \
  ros-humble-nav2-navfn-planner ros-humble-nav2-behaviors ros-humble-nav2-bt-navigator ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-velocity-smoother ros-humble-nav2-collision-monitor ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-nav2-costmap-2d ros-humble-nav2-msgs ros-humble-nav2-map-server ros-humble-apriltag-ros python3-numpy
source /opt/ros/humble/setup.bash && mkdir -p /ws/src && cd /ws/src
git clone --depth 1 -b v2.9.3 https://github.com/orbbec/OrbbecSDK_ROS2.git     # isteğe bağlı: depth:=gemini için
git clone --depth 1 https://github.com/Slamtec/sllidar_ros2.git
cp -r /repo/luna_ws/src/luna_robot . && rosdep update && rosdep install --from-paths . --ignore-src -y
cd /ws && colcon build && source install/setup.bash && colcon test --packages-select luna_robot && colcon test-result
# kamerayla dock:
set -m; ros2 launch luna_robot luna.launch.py tags:=sim voice:=false & sleep 4; python3 src/luna_robot/tools/dock_watch.py --seconds 50
# haritalama (önceki launch kapatıldıktan sonra; arkada süreç kalmadığından emin olun):
ros2 launch luna_robot luna_nav.launch.py map:=/ws/maps/room & sleep 15; python3 src/luna_robot/tools/sim_nav_check.py mapping --map /ws/maps/room
# parkur:
ros2 launch luna_robot luna_nav.launch.py world:=table map:=/ws/maps/course_table & sleep 15
python3 src/luna_robot/tools/sim_nav_check.py course --world table --label all --map /ws/maps/course_table
```

Bir sonraki senaryoya geçmeden önce eski launch'ın bütün süreçleri kapanmış olmalı. Kalan bir `base_node`, yeni koşuyu bozar (`sim_nav_check` bunu "only base_node drives the MCU" kontrolünde yakalar).

Yerel testler:
```bash
cd luna_ws/src/luna_robot && python3 -m unittest discover -s test
cd web && npm test
cd firmware/luna_mcu && ./host_test/run.sh && pio run -e teensy41
```

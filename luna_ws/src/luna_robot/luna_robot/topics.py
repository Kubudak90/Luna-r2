"""Single source of truth for topic names shared by the AGX nodes and the Teensy (micro-ROS) firmware.
Keep firmware/luna_mcu/src/topics.h in sync with this file."""

# Published by the MCU (or mock_mcu)
MCU_WHEELS = '/luna/mcu/wheels'          # Float32MultiArray [left_pos_rad, right_pos_rad, left_rad_s, right_rad_s]
MCU_BUMPERS = '/luna/mcu/bumpers'        # UInt8 bit0 FL, bit1 FR, bit2 BL, bit3 BR
MCU_CLIFF = '/luna/mcu/cliff'            # Float32MultiArray [FL, BL, FR, BR] metres, floor-facing VL53L4CD
MCU_CORNERS = '/luna/mcu/corners'        # Float32MultiArray [FL, FR, BL, BR] metres, VL53L1X
MCU_ESTOP = '/luna/mcu/estop'            # Bool, true = pressed (hardware already cut motor SLP)
MCU_DOCK_SENSE = '/luna/mcu/dock_sense'  # Bool, robot seated on a powered dock (P4 ID)
MCU_BATTERY = '/luna/mcu/battery'        # sensor_msgs/BatteryState from the BMS UART

# Consumed by the MCU
MCU_WHEEL_TARGET = '/luna/mcu/wheel_target'  # Float32MultiArray [left_rad_s, right_rad_s, pwm_cap]
MCU_HEAD_US = '/luna/mcu/head_us'            # Float32MultiArray [pan_us, tilt_us]
MCU_CHARGE_ENABLE = '/luna/mcu/charge_enable'  # Bool, false = DOCK_INHIBIT (dock switches the contacts off).
                                               # A false lapses after 1 s and on agent loss: charging is the default

# AGX-side
CMD_VEL = '/cmd_vel'
DOCK_CMD_VEL = '/luna/dock/cmd_vel'
DOCK_REQUEST = '/luna/dock/request'      # String: dock | undock | cancel
DOCK_STATE = '/luna/dock/state'          # String
DOCK_TAG = '/luna/dock/tag'              # PointStamped: x = body bearing offset -1..1 (right +), z = mouth-to-nose m (dock_tag_node)
DOCK_TAG_POSITION = '/luna/dock/tag_position'  # PointStamped base_link: fused tag centre (diagnostics)
HEAD_TARGET = '/luna/head/target'        # Vector3: x = pan deg (+ left), y = tilt deg (+ down)
HEAD_GAZE = '/luna/head/gaze'            # Vector3: x, y normalised -1..1
JOINT_STATES = '/joint_states'           # head_pan, head_tilt rad (head_node) → robot_state_publisher → TF
MOCK_TRUTH = '/luna/mock/truth'          # PoseStamped, frame 'dock': true mock robot pose (sim camera only)
SAFETY = '/luna/safety'                  # String: last gate reason
POWER_ACTION = '/luna/power/action'      # String: unknown (no BMS yet) | run | seek_dock | critical | charge | overtemp
POWER_MODE_W = '/luna/power/mode_w'      # Int32: requested AGX nvpmodel budget (10 / 15 / 30 W)

# Mapping / navigation (AGX only, not in topics.h)
SCAN = '/scan'                           # sensor_msgs/LaserScan, frame 'laser' (RPLIDAR C1 or sim2d)
NAV_DOCK = '/luna/nav/dock'              # String: go | cancel  (Nav2 to the pre-dock pose, then dock_node)
NAV_DOCK_STATE = '/luna/nav/dock_state'  # String: STATE or STATE:reason (core/dock_nav.py)
NAV_PRE_DOCK = '/luna/nav/pre_dock'      # PoseStamped (map), latched: where Nav2 hands over to dock_node
NAV_SAVE_DOCK = '/luna/nav/save_dock'    # std_srvs/Trigger: current pose = docked pose (robot on the dock)
INITIAL_POSE = '/initialpose'            # PoseWithCovarianceStamped (map): slam_toolbox relocalizes near it
SIM_TRUTH = '/luna/sim/truth'            # nav_msgs/Odometry, frame 'world': sim2d ground truth, for tests only
SIM_ESTOP = '/luna/sim/estop'            # Bool: press (true) / release the simulated E-stop, for tests only
SIM_CONTACT = '/luna/sim/contact'        # String "t name kind bands": sim2d contact with an obstacle, for tests only

# Obstacle sensors below the LiDAR plane (AGX only, not in topics.h)
TOF_RANGES = ('/luna/tof/fl', '/luna/tof/fr', '/luna/tof/bl', '/luna/tof/br')  # sensor_msgs/Range, frames tof_*
TOF_POINTS = tuple(t + '/points' for t in TOF_RANGES)  # PointCloud2: the same reading as an arc, for the costmaps
DEPTH_POINTS = '/oak/points'             # sensor_msgs/PointCloud2: OAK-D Lite (depthai_ros_driver) or sim_depth_node

HEAD_STATE = '/luna/head/state'          # Vector3: x = pan deg, y = tilt deg, as commanded after the slew limit

# Audio and voice (AGX-side; ReSpeaker XVF3800 under the head top shell, speaker on its amplified output)
AUDIO_DOA = '/luna/audio/doa'            # Float32MultiArray [bearing deg in base_link (+ = left), speech 0/1, raw board deg,
                                         #   head moving 0/1, bearing deg in the head]; speech is 0 while the head moves
AUDIO_MUTE = '/luna/audio/mute'          # Bool command: true = mute the microphones (board buttons are inside the head)
AUDIO_MUTED = '/luna/audio/muted'        # Bool state, 1 Hz and on change
AUDIO_UTTERANCE = '/luna/audio/utterance'  # String: path of a finished utterance WAV (16 kHz mono) for speech-to-text
VOICE_TALK = '/luna/voice/talk'          # Empty: a touch on the face to talk (push-to-talk, face bridge POST /talk): opens
                                         #   the listening window, cuts Luna off mid-sentence, drops an answer still on its way
VOICE_LISTENING = '/luna/voice/listening'  # Bool: the microphones take a question now (window open or utterance running)
STT_STATUS = '/luna/stt/status'          # String: idle | transcribing | ok | empty | error:<reason>  (never the words)
FACE_GAZE = '/luna/face/gaze'            # Vector3: eye gaze toward a voice, -1..1; the face prefers it to head gaze while fresh
ASSISTANT_REQUEST = '/luna/assistant/request'  # String: what the person said (from speech-to-text, or typed for tests)
ASSISTANT_REPLY = '/luna/assistant/reply'      # String: text to speak (cloud answer or offline fallback)
ASSISTANT_STATUS = '/luna/assistant/status'    # String: idle | thinking | ok | offline:<reason>
VOICE_SPEAKING = '/luna/voice/speaking'  # Bool: the speaker is playing Luna's voice

"""Single source of truth for topic names shared by the AGX nodes and the Teensy (micro-ROS) firmware.
Keep firmware/luna_mcu/src/topics.h in sync with this file."""

# Published by the MCU (or mock_mcu)
MCU_WHEELS = '/luna/mcu/wheels'          # Float32MultiArray [left_pos_rad, right_pos_rad, left_rad_s, right_rad_s]
MCU_BUMPERS = '/luna/mcu/bumpers'        # UInt8 bit0 FL, bit1 FR, bit2 BL, bit3 BR
MCU_CLIFF = '/luna/mcu/cliff'            # Float32MultiArray [FL, BL, FR, BR] metres, floor-facing VL53L4CD
MCU_CORNERS = '/luna/mcu/corners'        # Float32MultiArray [FL, FR, BL, BR] metres, VL53L1X
MCU_ESTOP = '/luna/mcu/estop'            # Bool, true = pressed (hardware already cut motor SLP)
MCU_DOCK_SENSE = '/luna/mcu/dock_sense'  # Bool, charge voltage present on the contacts
MCU_BATTERY = '/luna/mcu/battery'        # sensor_msgs/BatteryState from the BMS UART

# Consumed by the MCU
MCU_WHEEL_TARGET = '/luna/mcu/wheel_target'  # Float32MultiArray [left_rad_s, right_rad_s, pwm_cap]
MCU_HEAD_US = '/luna/mcu/head_us'            # Float32MultiArray [pan_us, tilt_us]

# AGX-side
CMD_VEL = '/cmd_vel'
DOCK_CMD_VEL = '/luna/dock/cmd_vel'
DOCK_REQUEST = '/luna/dock/request'      # String: dock | undock | cancel
DOCK_STATE = '/luna/dock/state'          # String
DOCK_TAG = '/luna/dock/tag'              # PointStamped: x = horizontal offset -1..1, z = distance m (from an AprilTag node)
HEAD_TARGET = '/luna/head/target'        # Vector3: x = pan deg, y = tilt deg
HEAD_GAZE = '/luna/head/gaze'            # Vector3: x, y normalised -1..1
SAFETY = '/luna/safety'                  # String: last gate reason
POWER_ACTION = '/luna/power/action'      # String: unknown (no BMS yet) | run | seek_dock | critical | charge | overtemp
POWER_MODE_W = '/luna/power/mode_w'      # Int32: requested AGX nvpmodel budget (10 / 15 / 30 W)

# Mapping / navigation (AGX only, not in topics.h)
SCAN = '/scan'                           # sensor_msgs/LaserScan, frame 'laser' (RPLIDAR C1 or sim2d)
NAV_DOCK = '/luna/nav/dock'              # String: go | cancel  (Nav2 to the pre-dock pose, then dock_node)
NAV_DOCK_STATE = '/luna/nav/dock_state'  # String: STATE or STATE:reason (core/dock_nav.py)
NAV_PRE_DOCK = '/luna/nav/pre_dock'      # PoseStamped (map), latched: where Nav2 hands over to dock_node
NAV_SAVE_DOCK = '/luna/nav/save_dock'    # std_srvs/Trigger: current pose = docked pose (robot on the dock)
SIM_TRUTH = '/luna/sim/truth'            # nav_msgs/Odometry, frame 'world': sim2d ground truth, for tests only
SIM_ESTOP = '/luna/sim/estop'            # Bool: press (true) / release the simulated E-stop, for tests only

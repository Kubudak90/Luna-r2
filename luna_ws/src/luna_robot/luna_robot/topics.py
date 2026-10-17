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
DOCK_TAG = '/luna/dock/tag'              # PointStamped: x = body bearing offset -1..1 (right +), z = mouth-to-nose m (dock_tag_node)
DOCK_TAG_POSITION = '/luna/dock/tag_position'  # PointStamped base_link: fused tag centre (diagnostics)
HEAD_TARGET = '/luna/head/target'        # Vector3: x = pan deg (+ left), y = tilt deg (+ down)
HEAD_GAZE = '/luna/head/gaze'            # Vector3: x, y normalised -1..1
JOINT_STATES = '/joint_states'           # head_pan, head_tilt rad (head_node) → robot_state_publisher → TF
MOCK_TRUTH = '/luna/mock/truth'          # PoseStamped, frame 'dock': true mock robot pose (sim camera only)
SAFETY = '/luna/safety'                  # String: last gate reason
POWER_ACTION = '/luna/power/action'      # String: unknown (no BMS yet) | run | seek_dock | critical | charge | overtemp
POWER_MODE_W = '/luna/power/mode_w'      # Int32: requested AGX nvpmodel budget (10 / 15 / 30 W)

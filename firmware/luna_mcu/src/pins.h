// Luna Teensy 4.1 pin map (Donanım ve Güç R01). Verify the encoder pins against the
// QuadEncoder library's XBAR-capable list before wiring; Teensy I/O is 3.3 V and NOT 5 V tolerant.
#pragma once

// Pololu Dual G2 18v18 — robot's own LEFT motor is on channel 1 (CAD REF_MOTOR_R, X>0).
constexpr int M1_PWM = 22, M1_DIR = 20, M1_FLT = 25, M1_CS = 38;   // CS: 20 mV/A
constexpr int M2_PWM = 23, M2_DIR = 21, M2_FLT = 26, M2_CS = 39;
constexpr int DRV_SLP = 24;          // goes to BOTH SLP pins THROUGH E-stop contact 1 (hardware cut)

// Encoders: 5 V supply, outputs through the Pololu #2595 level shifter.
// Checked 2026-10-17 against QuadEncoder.cpp's ARDUINO_TEENSY41 table (0-5, 7, 8, 30, 31, 33, 36, 37):
// 30/31 = XBAR IN23/IN22, 36/37 = XBAR IN16/IN17, all distinct → no change needed. 36/37 share
// XBAR inputs with pins 1/0, which is fine because 0/1 are Serial1 (BMS), not encoder inputs.
constexpr int ENC1_A = 30, ENC1_B = 31;   // left
constexpr int ENC2_A = 36, ENC2_B = 37;   // right

// goBILDA servos (signal 3.3 V is accepted by the servo; supply from D36V50F6 regulators).
constexpr int SERVO_PAN = 28, SERVO_TILT = 29;
constexpr int SERVO_EN = 27;         // to both regulator EN pins THROUGH E-stop contact 2

constexpr int ESTOP_SENSE = 32;      // INPUT_PULLUP: NC contact to GND, open (HIGH) = pressed
constexpr int BUMP_FL = 2, BUMP_FR = 3, BUMP_BL = 4, BUMP_BR = 5;   // D2F-01L to GND, INPUT_PULLUP

// ToF XSHUT lines: corners (VL53L1X on Wire) and floor (VL53L4CD on Wire1); all boot at 0x29.
constexpr int XSHUT_CORNER[4] = {6, 7, 8, 9};      // FL, FR, BL, BR
constexpr int XSHUT_CLIFF[4] = {10, 11, 12, 33};   // FL, BL, FR, BR

constexpr int VCHG_ADC = 40;         // A16, contact charge voltage via 47k/10k divider (16.8 V → 2.95 V); diagnostics only
constexpr float VCHG_DIVIDER = (47.0f + 10.0f) / 10.0f;
// Dock detection (Elektrik R01): P4 through 100k + 100 nF + BAT54S clamp; 2.50 V seated on a powered dock.
constexpr int DOCK_ID_ADC = 41;      // A17
constexpr int DOCK_INHIBIT = 35;     // HIGH asks the dock to switch the contacts off (AO3400A adds 2.2k on P4); 100k pull-down
// I2C: Wire SDA 18 / SCL 19 (corners), Wire1 SDA1 17 / SCL1 16 (floor VL53L4CD).
// BMS (JBD, 3.3 V TTL UART — confirm level on the chosen BMS) on Serial1: RX 0, TX 1.

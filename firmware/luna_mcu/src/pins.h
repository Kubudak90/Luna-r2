// Luna R2 Teensy 4.1 pin map (docs/TASARIM_R2.md, "Elektrik"). R1's pins carried over except the head servos,
// which R2 does not have. Teensy I/O is 3.3 V and NOT 5 V tolerant.
#pragma once

// Pololu Dual G2 18v18 — the robot's LEFT wheel motor is on channel 1.
constexpr int M1_PWM = 22, M1_DIR = 20, M1_FLT = 25, M1_CS = 38;   // CS: 20 mV/A
constexpr int M2_PWM = 23, M2_DIR = 21, M2_FLT = 26, M2_CS = 39;
constexpr int DRV_SLP = 24;          // goes to BOTH SLP pins THROUGH E-stop contact 1 (hardware cut); 10k to GND

// Encoders (Pololu 37D, 64 CPR Hall): 5 V supply, outputs through the Pololu #2595 level shifter.
// Checked 2026-10-17 against QuadEncoder.cpp's ARDUINO_TEENSY41 table (0-5, 7, 8, 30, 31, 33, 36, 37):
// 30/31 = XBAR IN23/IN22, 36/37 = XBAR IN16/IN17, all distinct → no change needed. 36/37 share
// XBAR inputs with pins 1/0, which is fine because 0/1 are Serial1 (BMS), not encoder inputs.
constexpr int ENC1_A = 30, ENC1_B = 31;   // left
constexpr int ENC2_A = 36, ENC2_B = 37;   // right

// Pins 27, 28, 29 (R1: head servos and their regulator EN) are free.

constexpr int ESTOP_SENSE = 32;      // INPUT_PULLUP: E-stop contact 2 (NC) to GND, open (HIGH) = pressed
// Bumper ring: the front and rear halves pivot on two D2F-01L each at ±45° (sim2d.bumper_bits). To GND, INPUT_PULLUP.
constexpr int BUMP_FL = 2, BUMP_FR = 3, BUMP_BL = 4, BUMP_BR = 5;

// ToF XSHUT lines: corners and floor, all VL53L1X (corners on Wire, floor on Wire1); all boot at 0x29.
constexpr int XSHUT_CORNER[4] = {6, 7, 8, 9};      // FL, FR, BL, BR
constexpr int XSHUT_CLIFF[4] = {10, 11, 12, 33};   // FL, BL, FR, BR

constexpr int VCHG_ADC = 40;         // A16, contact charge voltage via 47k/10k divider (14.6 V → 2.56 V); diagnostics only
constexpr float VCHG_DIVIDER = (47.0f + 10.0f) / 10.0f;
// Dock detection (Elektrik R01): P4 through 100k + 100 nF + BAT54S clamp; 2.50 V seated on a powered dock.
constexpr int DOCK_ID_ADC = 41;      // A17
constexpr int DOCK_INHIBIT = 35;     // HIGH asks the dock to switch the contacts off (AO3400A adds 2.2k on P4); 100k pull-down
// I2C: Wire SDA 18 / SCL 19 (corner VL53L1X), Wire1 SDA1 17 / SCL1 16 (floor VL53L1X).
// BMS: Daly smart BMS UART port on Serial1 (RX 0, TX 1), 9600 8N1. The port is meant for 3.3 V TTL but MEASURE it
// before wiring (Teensy is not 5 V tolerant); the same port feeds Daly's Bluetooth dongle, which must be unplugged.

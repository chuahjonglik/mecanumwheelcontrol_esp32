# Mecanum Wheel Control
Mecanum wheel control on ESP32 using Bluepad controller.

# Versions
- Without PID
- With PID (PID contants need to be tuned manually)

# Details
- Microcontroller: ESP32
- Motor Driver: 4-channel outputs with PWM and direction control
- Sensors: ICM20948 (only for PID version)
- Tools: Arduino IDE
- Libraries: Bluepad32 (for all versions), PID_v1, ICM20948_WE (only for PID version)

# Disclaimer
PID version is not tested yet, please submit ticket in Issue section if you encountered any problems.

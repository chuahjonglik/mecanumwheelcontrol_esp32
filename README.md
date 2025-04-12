# Mecanum Wheel Control
Mecanum wheel control on ESP32 using Bluepad controller.

## Versions
- Without PID
- With PID (PID contants need to be tuned manually)

## Details
- Microcontroller: ESP32
- Motor Driver: 4-channel outputs with PWM and direction control
- Sensors: ICM20948 (only for PID version)
- Tools: Arduino IDE
- Libraries: Bluepad32 (for all versions), PID_v1, ICM20948_WE (only for PID version)

## What to Change

**Pin definitions**
Modify the GPIO to match your wiring.
```C++
  // Motor driver pins
  #define PWM_FL 19  // Front Left Motor PWM
  #define DIR_FL 32  // Front Left Motor DIR
  #define PWM_FR 18  // Front Right Motor PWM
  #define DIR_FR 33  // Front Right Motor DIR
  #define PWM_RL 17  // Rear Left Motor PWM
  #define DIR_RL 25  // Rear Left Motor DIR
  #define PWM_RR 16  // Rear Right Motor PWM
  #define DIR_RR 26  // Rear Right Motor DIR
```

**Max Speed Control**
Modify `MAX_PWM` to limit your wheel maximum speed. Value should be within range of 0 to 255.
```C++
  // Speed control
  #define MAX_PWM 255  // Maximum PWM value
```

**Deadzone**
Modify `DEADZONE` to set a deadzone for left joystick. Value should be within range of 0 to 127. Setting a higher value can prevent accidental movement due to joystick drift.
```C++
  // Deadzone threshold
  #define DEADZONE 50
```

**Kinematics**
Adjust the sign of each parameters to match the desired outcome. Refer to reference links in the next section for more information.
```C++
  int frontLeft = LeftY + LeftX + rotation;
  int frontRight = LeftY - LeftX - rotation;
  int rearLeft = LeftY - LeftX + rotation;
  int rearRight = LeftY + LeftX - rotation;
```

## Reference
- Mecanum Wheels by FTC Team 7393 - electron Volts [GitHub](https://github.com/FTC7393/EVLib/wiki/Mecanum-Wheels)
- Mecanum Wheels by Ecam Eurobot [GitHub.io](https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html)

## 
**Disclaimer**
PID version is not tested yet, please submit ticket in Issue section if you encountered any problems.

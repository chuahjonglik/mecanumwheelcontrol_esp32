/* Libraries */
#include <Bluepad32.h>
#include <PID_v1.h>
#include <Wire.h>
#include <ICM20948_WE.h>

/* Pin definition */
// I2C
#define ICM20948_ADDR 0x68  // ICM-20948 address
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
// Motor driver pins
#define PWM_FL 18  // Front Left Motor PWM
#define DIR_FL 19  // Front Left Motor DIR
#define PWM_FR 4   // Front Right Motor PWM
#define DIR_FR 5   // Front Right Motor DIR
#define PWM_RL 16  // Rear Left Motor PWM
#define DIR_RL 17  // Rear Left Motor DIR
#define PWM_RR 21  // Rear Right Motor PWM
#define DIR_RR 22  // Rear Right Motor DIR
// Encoder pins
#define ENC_X 34  // X-axis encoder
#define ENC_Y 35  // Y-axis encoder

/* Constants and Objects */
// Speed control
#define MAX_PWM 255  // Maximum PWM value
// Controller
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
ControllerPtr PS4;
// PID variables for x, y, and yaw control
double setpointX = 0, inputX = 0, outputX = 0;
double setpointY = 0, inputY = 0, outputY = 0;
double setpointYaw = 0, inputYaw = 0, outputYaw = 0;
// PID tuning parameters (adjust as needed)
double Kp = 2.0, Ki = 0.5, Kd = 0.2;

PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT);
PID pidYaw(&inputYaw, &outputYaw, &setpointYaw, Kp, Ki, Kd, DIRECT);

volatile int countX = 0, countY = 0;  // Encoder pulse counters
float yawAngle = 0;                   // Gyro-based yaw tracking

void IRAM_ATTR readEncoderX() {
  countX++;
}
void IRAM_ATTR readEncoderY() {
  countY++;
}

/* Functions */
void initializePins() {
  // Initialize motor driver pins
  pinMode(PWM_FL, OUTPUT);
  pinMode(DIR_FL, OUTPUT);
  pinMode(PWM_FR, OUTPUT);
  pinMode(DIR_FR, OUTPUT);
  pinMode(PWM_RL, OUTPUT);
  pinMode(DIR_RL, OUTPUT);
  pinMode(PWM_RR, OUTPUT);
  pinMode(DIR_RR, OUTPUT);

  // Initialize encoders
  pinMode(ENC_X, INPUT_PULLUP);
  pinMode(ENC_Y, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_X), readEncoderX, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_Y), readEncoderY, RISING);
}

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;

      // To ensure only can connect to one controller
      if (PS4 == nullptr) {
        PS4 = ctl;
      }
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...
  // if (ctl->a()) {
  //   static int colorIdx = 0;
  //   // Some gamepads like DS4 and DualSense support changing the color LED.
  //   // It is possible to change it by calling:
  //   switch (colorIdx % 3) {
  //     case 0:
  //       // Red
  //       ctl->setColorLED(255, 0, 0);
  //       break;
  //     case 1:
  //       // Green
  //       ctl->setColorLED(0, 255, 0);
  //       break;
  //     case 2:
  //       // Blue
  //       ctl->setColorLED(0, 0, 255);
  //       break;
  //   }
  //   colorIdx++;
  // }

  // if (ctl->b()) {
  //   // Turn on the 4 LED. Each bit represents one LED.
  //   static int led = 0;
  //   led++;
  //   // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
  //   // support changing the "Player LEDs": those 4 LEDs that usually indicate
  //   // the "gamepad seat".
  //   // It is possible to change them by calling:
  //   ctl->setPlayerLEDs(led & 0x0f);
  // }

  // if (ctl->x()) {
  //   // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
  //   // It is possible to set it by calling:
  //   // Some controllers have two motors: "strong motor", "weak motor".
  //   // It is possible to control them independently.
  //   ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
  //                       0x40 /* strongMagnitude */);
  // }
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void moveMotor(int pwmPin, int dirPin, int speed) {
  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, constrain(abs(speed), 0, 255));
}

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

  // Initialize Bluepad controller
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();

  initializePins();

  // IMU setup
  Wire.begin();
  if (!myIMU.init()) {
    Serial.println("IMU not detected!");
    while (1)
      ;
  }
  myIMU.enableGyr(true);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);

  // Enable PID controllers
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidYaw.SetMode(AUTOMATIC);
}

void loop() {
  static unsigned long lastMillis = 0;

  // bool dataUpdated = BP32.update();

  // if (dataUpdated)
  //   processControllers();
  // // to ensure ps4 is connected
  // if (PS4 == nullptr) {
  //   Serial.println("Controller is disconnected");
  //   return;
  // }

  if (PS4 && PS4->isConnected()) {
    int x = PS4->axisX();                  // Left stick X (-512 to 512)
    int y = -PS4->axisY();                 // Left stick Y (-512 to 512)
    int rotation = PS4->r1() - PS4->l1();  // Combination of Left and Right stick X (-512 to 512)

    // Convert to target velocities
    setpointX = map(x, -512, 512, -100, 100);
    setpointY = map(y, -512, 512, -100, 100);
    setpointYaw = map(rotation, -512, 512, -50, 50);

    // Normalize motion
    int maxSpeed = max(max(abs(setpointX), abs(setpointY)), abs(setpointYaw));
    if (maxSpeed > 100) {
      setpointX = setpointX * 100 / maxSpeed;
      setpointY = setpointY * 100 / maxSpeed;
      setpointYaw = setpointYaw * 50 / maxSpeed;
    }
  }

  if (millis() - lastMillis >= 100) {
    lastMillis = millis();

    // Compute actual speed
    inputX = (countX * 600) / 20;
    inputY = (countY * 600) / 20;
    countX = countY = 0;

    // Read yaw from IMU
    xyzFloat gyroData;
    myIMU.getGValues(&gyroData);
    yawAngle += gyroData.z * 0.1;
    inputYaw = yawAngle;

    // Compute PID corrections
    pidX.Compute();
    pidY.Compute();
    pidYaw.Compute();

    // Apply PID-controlled speeds to mecanum drive
    double FL = outputY + outputX + outputYaw;
    double FR = outputY - outputX - outputYaw;
    double RL = outputY - outputX + outputYaw;
    double RR = outputY + outputX - outputYaw;

    // Normalize motor speeds
    int maxSpeed = max(max(abs(FL), abs(FR)), max(abs(RL), abs(RR)));
    if (maxSpeed > 255) {
      FL = FL * 255 / maxSpeed;
      FR = FR * 255 / maxSpeed;
      RL = RL * 255 / maxSpeed;
      RR = RR * 255 / maxSpeed;
    }

    // Move motors
    moveMotor(PWM_FL, DIR_FL, FL);
    moveMotor(PWM_FR, DIR_FR, FR);
    moveMotor(PWM_RL, DIR_RL, RL);
    moveMotor(PWM_RR, DIR_RR, RR);
  }
}

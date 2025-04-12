/* Libraries */
#include <Bluepad32.h>

/* Pin definition */
// Motor driver pins
#define PWM_FL 19  // Front Left Motor PWM
#define DIR_FL 32  // Front Left Motor DIR
#define PWM_FR 18  // Front Right Motor PWM
#define DIR_FR 33  // Front Right Motor DIR
#define PWM_RL 17  // Rear Left Motor PWM
#define DIR_RL 25  // Rear Left Motor DIR
#define PWM_RR 16  // Rear Right Motor PWM
#define DIR_RR 26  // Rear Right Motor DIR

/* Constants and Objects */
// Speed control
#define MAX_PWM 255  // Maximum PWM value
// Controller
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
ControllerPtr PS4;
// Controller Values
int LeftX, LeftY, LeftShoulder, RightShoulder;
// Deadzone threshold
#define DEADZONE 50

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
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
    speed = -speed;  // Convert to positive PWM
  }
  analogWrite(pwmPin, constrain(speed, 0, MAX_PWM));
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
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();
  // to ensure ps4 is connected
  if (PS4 == nullptr) {
    Serial.println("Controller is disconnected");
    return;
  }

  if (PS4 && PS4->isConnected()) {
    LeftX = PS4->axisX();
    LeftY = PS4->axisY();
    LeftShoulder = PS4->brake();
    RightShoulder = PS4->throttle();

    // Apply deadzone
    if (abs(LeftX) < DEADZONE) LeftX = 0;
    if (abs(LeftY) < DEADZONE) LeftY = 0;
    if (abs(LeftShoulder) < DEADZONE) LeftShoulder = 0;
    if (abs(RightShoulder) < DEADZONE) RightShoulder = 0;

    // Convert to target velocities
    LeftX = map(LeftX, -512, 512, -127, 127);
    LeftY = map(LeftY, -512, 512, -127, 127);
    LeftY = -LeftY;
    LeftShoulder = map(LeftShoulder, 0, 1023, 0, 127);
    RightShoulder = map(RightShoulder, 0, 1023, 0, 127);
  }

  int rotation = RightShoulder - LeftShoulder;
  Serial.printf("LeftX = %d LeftY = %d Rotation = %d ", LeftX, LeftY, rotation);
  // Omniwheel kinematics (Original)
  int frontLeft = LeftY + LeftX + rotation;
  int frontRight = LeftY - LeftX - rotation;
  int rearLeft = LeftY - LeftX + rotation;
  int rearRight = LeftY + LeftX - rotation;

  // These equations are not the correct one but were used due to wrong wiring setup
  // int frontLeft = LeftY + LeftX - rotation;
  // int frontRight = LeftY - LeftX - rotation;
  // int rearLeft = -LeftY + LeftX - rotation;
  // int rearRight = LeftY + LeftX + rotation;

  // Normalize motor speeds
  int maxSpeed = max(max(abs(frontLeft), abs(frontRight)), max(abs(rearLeft), abs(rearRight)));
  if (maxSpeed > 127) {
    frontLeft = frontLeft * 127 / maxSpeed;
    frontRight = frontRight * 127 / maxSpeed;
    rearLeft = rearLeft * 127 / maxSpeed;
    rearRight = rearRight * 127 / maxSpeed;
  }

  // Scale to PWM range (0 - 255)
  frontLeft = map(frontLeft, -127, 127, -MAX_PWM, MAX_PWM);
  frontRight = map(frontRight, -127, 127, -MAX_PWM, MAX_PWM);
  rearLeft = map(rearLeft, -127, 127, -MAX_PWM, MAX_PWM);
  rearRight = map(rearRight, -127, 127, -MAX_PWM, MAX_PWM);
  Serial.printf("FL = %d FR = %d RL = %d RR = %d\n", frontLeft, frontRight, rearLeft, rearRight);

  // Apply motor movement
  moveMotor(PWM_FL, DIR_FL, frontLeft);
  moveMotor(PWM_FR, DIR_FR, frontRight);
  moveMotor(PWM_RL, DIR_RL, rearLeft);
  moveMotor(PWM_RR, DIR_RR, rearRight);
}

#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define bucketServoPin 23  // Bucket servo
#define clawServoPin 22    // Claw Servo
// ESP32 output pins can drive > 10mA, so can drive (low power) LEDs directly
#define cabLights 16  // Either both cab lights, or just left
#define auxLights 17  // Either right cab light, or aux light

#define armMotor0 21  // Used for controlling arm
#define armMotor1 19  // Used for controlling arm
#define auxMotor0 5   // Used for controlling auxiliary attachment movement
#define auxMotor1 18  // Used for controlling auxiliary attachment movement

#define leftMotor0 32   // Used for controlling the left motor movement
#define leftMotor1 33   // Used for controlling the left motor movement
#define rightMotor0 26  // Used for controlling the right motor movementc:\Users\JohnC\Desktop\SOLIDWORKS Connected.lnk
#define rightMotor1 25  // Used for controlling the right motor movement

#define BucketMAX 180           // max extent of bucket servo in degrees
#define BucketMIN 0             // min extent of bucket servo in degrees
#define ClawMAX 180             // max extent of claw servo in degrees
#define ClawMIN 0               // min extent of claw servo in degrees
#define BucketHOME 150          // home (power-on) position of bucket servo in degrees
#define ClawHOME 180            // home (power-on) position of claw servo in degrees
#define MotorDEADBAND 30        // value below which we don't move motors (deadband around zero)
#define toggleDebounceTIME 300  // button toggle delay in milliseconds
#define BucketJoystickSENSE 48  // divider for x axis joystick for bucket movement

unsigned long lastInputTime = 0;         // failsafe timer to halt motors if controller disconnects
const unsigned long INPUT_TIMEOUT = 40;  // failsafe timeout in milliseconds - adjust to suit conditions

Servo bucketServo;  // initialise servo class for bucket
Servo clawServo;    // initialise servo class for claw

int bucketServoValue = BucketHOME;  // set servos to home position
int clawServoValue = ClawHOME;

bool cabLightsOn = false;       // toggle for cab lights (or Left cab light)
bool auxLightsOn = false;       // toggle for aux lights (or Right cab light)
static int toggleDebounce = 0;  // delay between toggle button presses

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
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but no empty slot found");
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


// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(armMotor0, OUTPUT);
  pinMode(armMotor1, OUTPUT);
  pinMode(auxMotor0, OUTPUT);
  pinMode(auxMotor1, OUTPUT);
  digitalWrite(auxMotor0, LOW);
  digitalWrite(auxMotor1, LOW);
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);

  pinMode(cabLights, OUTPUT);
  pinMode(auxLights, OUTPUT);

  digitalWrite(leftMotor0, LOW);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(rightMotor0, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(armMotor0, LOW);
  digitalWrite(armMotor1, LOW);

  digitalWrite(cabLights, LOW);
  digitalWrite(auxLights, LOW);

  bucketServo.attach(bucketServoPin);
  clawServo.attach(clawServoPin);

  bucketServo.write(bucketServoValue);  // set bucket to initial position
  clawServo.write(clawServoValue);      // set claw to open position

  Serial.begin(115200);
  const uint8_t *addr = BP32.localBdAddress();

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // So requires gamepad to be re-paired on every power-up
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled (we only use gamepad functionality here).
  BP32.enableVirtualDevice(false);

  // Add additional setup or error detection functions here

  lastInputTime = millis();  // initialize failsafe timer
}

void processGamepad(ControllerPtr ctl) {

  lastInputTime = millis();  // Reset failsafe timer as we received fresh input
  //Throttle
  processThrottle(ctl->axisX(), ctl->axisY());  // control track motors
  // Arm
  processArm(ctl->axisRY());  // arm uses right joystick
  //Lights
  processLights(ctl->y(), ctl->b());  // control LED pins with triangle and circle buttons
  // Bucket
  processBucket(ctl->topRight(), ctl->axisRX(), ctl->r1());  // control bucket and claw
}

// control motion and steeriing with left joystick (y axis forward/reverse, x axis left/right)
void processThrottle(int xaxis, int yaxis) {
  static int Lvalue, Rvalue;             // calculated motor velocities
  Lvalue = Rvalue = yaxis / 2;           // forward/reverse
  Lvalue = Lvalue + (-1 * (xaxis / 4));  // add left/right value to motor motion
  Rvalue = Rvalue + (xaxis / 4);
  moveMotor(leftMotor0, leftMotor1, Lvalue);
  moveMotor(rightMotor0, rightMotor1, Rvalue);
}

void processArm(int velocity) {
  moveMotor(armMotor0, armMotor1, (velocity / 6));  // reduce arm motor speed to controllable level
}
// convert signed velocity value into motor movement in the right direction
void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if(velocity > 255)
    velocity = 255;
  if(velocity < -255)
    velocity = -255;
  
  if (velocity > MotorDEADBAND) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -MotorDEADBAND) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, LOW);
  }
}

//  Use ESP32 output pins to directly drive LEDs. Pins can source >10 mA
void processLights(bool triangle, bool circle) {
  if (millis() - toggleDebounce > toggleDebounceTIME) {  // delay in milliseconds between toggle button actions
    // first output pin
    if (triangle) {
      if (!cabLightsOn)  // toggle LED on or off from a single button
      {
        digitalWrite(cabLights, HIGH);
        cabLightsOn = true;
      } else {
        digitalWrite(cabLights, LOW);
        cabLightsOn = false;
      }
    }
    // second output pin
    if (circle) {
      if (!auxLightsOn)  // toggle LED on or off from a single button
      {
        digitalWrite(auxLights, HIGH);
        auxLightsOn = true;
      } else {
        digitalWrite(auxLights, LOW);
        auxLightsOn = false;
      }
    }
    toggleDebounce = millis();  // reset toggle timer
  }
}

void processBucket(int switches, int rxaxis, int rbutton) {
  // switches is 4-way with bitmap - up=1,down=2,left=8,right=4
  // up/down is claw, left/right is bucket
  if (switches & 0x01)  // claw open
    if (clawServoValue < ClawMAX)
      clawServoValue++;
  if (switches & 0x02)  // claw close
    if (clawServoValue > ClawMIN)
      clawServoValue--;

  if (switches & 0x08)  // bucket up
    if (bucketServoValue < BucketMAX)
      bucketServoValue++;
  if (switches & 0x04)  // bucket down
    if (bucketServoValue > BucketMIN)
      bucketServoValue--;
  // also control bucket angle from right joystick x axis
  rxaxis = rxaxis / BucketJoystickSENSE;  // reduce sensitivity on joystick
  if (rxaxis > 1 || rxaxis < -1) {
    bucketServoValue -= rxaxis;  // adjust bucket pos using joystick (left=up, right=down)
  }
  if (rbutton > 0)  // right top button resets to default positions
  {
    bucketServoValue = BucketHOME;  // set servos to home position
    clawServoValue = ClawHOME;
  }

  if (bucketServoValue > BucketMAX)  // ensure server values don't go out of range
    bucketServoValue = BucketMAX;
  if (bucketServoValue < BucketMIN)
    bucketServoValue = BucketMIN;

  bucketServo.write(bucketServoValue);  // set bucket to new position
  clawServo.write(clawServoValue);      // set claw to new position
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

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  else { vTaskDelay(1); }
  // Failsafe check: if no input for too long, stop motors
  if (millis() - lastInputTime > INPUT_TIMEOUT) {
    digitalWrite(leftMotor0, LOW);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(rightMotor0, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(armMotor0, LOW);
    digitalWrite(armMotor1, LOW);
  }
}

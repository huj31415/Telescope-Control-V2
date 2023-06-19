#include <AccelStepper.h>
#include <SiderealObjects.h>
#include <SiderealPlanets.h>

// Define the stepper control pins
#define PitchDir 8
#define PitchStep 9
#define YawDir 6
#define YawStep 7

// Resolution pins (common)
#define M0 5
#define M1 4
#define M2 3

// Steps per revolution of the axle (default 200)
#define StepsPerRevolution 200

// Pitch gear ratio: 10:100
#define PitchGearRatio 10

// Yaw gear ratio: 8:80
#define YawGearRatio 10

// Initial number of microsteps per step
// Multiples of 2, 1 to 32
// Set to 32 for fine control, 8 for coarse
#define MaxStepRes 32

// Max speed in steps/sec
#define maxSpeed 50 * MaxStepRes

// Length of a sidereal day in seconds
// #define SiderealDay 86164.0905

// Joystick pins
// X output
#define joystickX A0

// Y output
#define joystickY A1

// Button press
#define joystickB 2

// Joystick detection threshold
#define threshold 24

// Mode switch button pin
#define modeBtn 10

// // Joystick offsets compensate for drift
// byte xOffset;
// byte yOffset;

// initialize the steppers
AccelStepper az(AccelStepper::DRIVER, PitchStep, PitchDir);
AccelStepper alt(AccelStepper::DRIVER, YawStep, YawDir);

// initialize sidereal objects and planets
SiderealObjects objects;
SiderealPlanets planets;

// fine adjustment state
volatile bool fine = false;

// Set the resolution (microsteps/step) of the steppers. The res pins are common to both so it is only done once.
// Resolution is 1-32, multiples of 2.
void setRes(int res = 32) {
  int x = log(res) / log(2);
  digitalWrite(M0, (x & 1) == 0 ? LOW : HIGH);
  digitalWrite(M1, (x & 2) == 0 ? LOW : HIGH);
  digitalWrite(M2, (x & 4) == 0 ? LOW : HIGH);
}

// Joystick interrupt service routine to switch between coarse and fine manual control
void changeRes() {
  fine = !fine;
  fine == true ? setRes(MaxStepRes) : setRes(MaxStepRes / 4);
  Serial.println("Resolution Change");
}

// Mode change button ISR to toggle tracking, etc.
void changeMode() {
  //
}

// Read the x/y values of the joystick and move the motors accordingly
void manualControl() {

  // read joystick values and set them to [-511, 512] instead of [0, 1023]
  int x = (analogRead(joystickX) - 511);
  int y = (analogRead(joystickY) - 511);

  // Check if the joystick values are above the threshold to reduce drift or noise, then map + set speeds
  if (abs(x) > threshold || abs(y) > threshold) {
    // Map joystick values to speeds
    az.setSpeed(map(x, -512, 512, -maxSpeed, maxSpeed));
    alt.setSpeed(map(y, -512, 512, -maxSpeed, maxSpeed));

    // Move to implement acceleration (?)
    az.move(8);
    alt.move(8);

  } else {
    // Ignore small variations below the threshold
    az.stop();
    alt.stop();
    az.setSpeed(0);
    alt.setSpeed(0);
  }

}

void setup()  // setup stuff
{
  // set joystick pin modes to input
  pinMode(joystickX, INPUT);
  pinMode(joystickY, INPUT);

  // set joystick button pin to input
  pinMode(joystickB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(joystickB), changeRes, FALLING);

  pinMode(modeBtn, INPUT_PULLUP)
  attachInterrupt(digitalPinToInterrupt(modeBtn), changeMode, FALLING);

  // init serial for debugging
  Serial.begin(9600);

  // set stepper resolution
  setRes(8);

  // set stepper max speed
  alt.setMaxSpeed(maxSpeed);
  az.setMaxSpeed(maxSpeed);

  // set acceleration
  alt.setAcceleration(0.01);
  az.setAcceleration(0.01);

  // // Aim at Polaris to establish relative position
  // manualControl();
  // alt.setCurrentPosition();
  // az.setCurrentPosition();
}

void loop()  // loop
{
  manualControl();
}
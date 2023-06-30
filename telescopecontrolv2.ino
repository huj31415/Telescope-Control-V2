#include <AccelStepper.h>

// Define the stepper control pins
#define AltDir 8
#define AltStep 9
#define AzDir 16
#define AzStep 10

// Resolution pins (common)
#define M0 5
#define M1 4
#define M2 3

// Steps per revolution of the axle (default 200)
// #define StepsPerRevolution 200

// Alt gear ratio: 10:100
#define AltGearRatio 10

// Az gear ratio: 8:80
#define AzGearRatio 10

// Initial number of microsteps per step
// Multiples of 2, 1 to 32
// Set to 32 for fine control, 8 for coarse
#define MaxStepRes 32

// Max speed in steps/sec
#define MaxSpeed 50 * MaxStepRes

// Length of a sidereal day in seconds
// #define SiderealDay 86164.0905

// Joystick pins
// X output
#define JoystickX A0

// Y output
#define JoystickY A1

// Button press
#define JoystickB 2

// Joystick detection Threshold
#define Threshold 24

// Mode switch button pin
#define ModeBtn 7 // for leonardo, 3 for uno

// initialize the steppers
AccelStepper az(AccelStepper::DRIVER, AltStep, AltDir);
AccelStepper alt(AccelStepper::DRIVER, AzStep, AzDir);

// possible program states
enum State {
  COARSE,
  FINE,
  AUTOAIM,
  TRACK
};

volatile State currentState = COARSE;

// large objects for use with SiderealPlanets
enum Objects {
  SUN,
  MOON,
  MERCURY,
  VENUS,
  MARS,
  JUPITER,
  SATURN,
  URANUS,
  NEPTUNE
};

struct Coord {
  // alt az in degrees
  double alt;
  double az;
  // ra dec
  double ra; // RA in hrs
  double dec; // Dec in deg
  // // rise set in hrs since midnight
  // double rise;
  // double set;
};

Coord planet;

// Set the resolution (microsteps/step) of the steppers. The res pins are common to both so it is only done once.
// Resolution is 1-32, powers of 2.
void setRes(int res = 32) {
  int x = log(res) / log(2);
  digitalWrite(M0, (x & 1) == 0 ? LOW : HIGH);
  digitalWrite(M1, (x & 2) == 0 ? LOW : HIGH);
  digitalWrite(M2, (x & 4) == 0 ? LOW : HIGH);
}

// Joystick interrupt service routine to switch between coarse and fine manual control
void changeRes() {
  if (currentState == COARSE) {
    currentState = FINE;
    setRes(MaxStepRes);
  } else if (currentState == FINE) {
    currentState = COARSE;
    setRes(MaxStepRes / 4);
  }
}


// Read the x/y values of the joystick and move the motors accordingly
void manualControl() {

  // read joystick values and set them to [-511, 512] instead of [0, 1023]
  int x = (analogRead(JoystickX) - 511);
  int y = (analogRead(JoystickY) - 511);

  // Check if the joystick values are above the Threshold to reduce drift or noise, then map + set speeds
  if (abs(x) > Threshold || abs(y) > Threshold) {
    // Map joystick values to speeds
    az.setSpeed(map(x, -512, 512, -MaxSpeed, MaxSpeed));
    alt.setSpeed(map(y, -512, 512, -MaxSpeed, MaxSpeed));

    // Move to implement acceleration (?)
    az.move(8);
    alt.move(8);
    
    Serial.println("moved");

  } else {
    // Ignore small variations below the Threshold
    az.stop();
    alt.stop();
    az.setSpeed(0);
    alt.setSpeed(0);
  }

  // move the motors
  alt.run();
  az.run();
}

void setup()
{
  // set joystick pin modes to input
  pinMode(JoystickX, INPUT);
  pinMode(JoystickY, INPUT);

  // set joystick button pin to input - only pins 2 and 3 are interrupt pins
  pinMode(JoystickB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(JoystickB), changeRes, RISING);

  // init serial for debugging
  Serial.begin(9600);

  // set stepper resolution
  setRes(8);

  // set stepper max speed
  alt.setMaxSpeed(MaxSpeed);
  az.setMaxSpeed(MaxSpeed);

  // set acceleration
  alt.setAcceleration(1);
  az.setAcceleration(1);
}

void loop()
{
  if (currentState == COARSE || currentState == FINE) {
    manualControl();
  } else if (currentState == AUTOAIM) {
    // get target planet info, then aim
  } else if (currentState == TRACK) {
    // aim then track either by calculating position every loop or aiming in a circle around polaris
  }
}


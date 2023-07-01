#include <AccelStepper.h>
#include <SiderealPlanets.h>

// ====== PIN ASSIGNMENTS ======

// Define the stepper control pins
#define AltDir 8
#define AltStep 9
#define AzDir 16
#define AzStep 10

// Resolution pins (common)
#define M0 5
#define M1 4
#define M2 3

// Joystick pins
// X output
#define JoystickX A0

// Y output
#define JoystickY A1

// Button press
#define JoystickB 2

// Mode switch button pin
#define ModeBtn 7  // for leonardo, 3 for uno

// ====== OTHER CONSTANTS ======

// Joystick detection Threshold
#define Threshold 24

// Alt gear ratio: 10:100
#define AltGearRatio 10

// Az gear ratio: 8:80
#define AzGearRatio 10

// Initial number of microsteps per step
// Multiples of 2, 1 to 32 for DRV8825
// Set to 32 for fine control, 8 for coarse
#define MaxStepRes 32
#define FineAdjRes MaxStepRes
#define CoarseAdjRes MaxStepRes / 4

// Max speed in steps/sec
#define MaxSpeed 100 * MaxStepRes

// Length of a sidereal day in milliseconds
#define SiderealDayMillis 86164090.5

// Angular velocity of Earth
const double EarthAngularVel = (2 * PI) / (24 * 60 * 60 * 1000);

// initialize the steppers
AccelStepper az(AccelStepper::DRIVER, AltStep, AltDir);
AccelStepper alt(AccelStepper::DRIVER, AzStep, AzDir);

// time when tracking began in millis
long trackingStartTime;
long trackingElapsedTime;

// possible program states
enum State {
  CALIBRATE,  // find polaris in setup
  STICK,      // using joystick
  MANUAL,     // by hand, disable steppers
  AUTOAIM,    // autoaim + track planets
  TRACK       // track current position
};

// program current state
volatile State currentState;

// current resolution of steppers
volatile int currentRes;

// steps per revolution accounting for microsteps
volatile int stepsPerRevolution;

// vars for simple tracking
double altRad, azRad, radDistToPole, altTarget, azTarget;

// are the steppers enabled
bool enabled = true;

/* AutoAim and planet tracking not currently implemented - requires ESP8266 to do calcs
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
*/

// Set the resolution (microsteps/step) of the steppers. The res pins are common to both so it is only done once.
// Resolution is 1-32, powers of 2.
void setRes(int res = 32) {
  int x = log(res) / log(2);
  digitalWrite(M0, (x & 1) == 0 ? LOW : HIGH);
  digitalWrite(M1, (x & 2) == 0 ? LOW : HIGH);
  digitalWrite(M2, (x & 4) == 0 ? LOW : HIGH);
  currentRes = res;
}

// Joystick interrupt service routine to switch between coarse and fine manual control
void changeRes() {
  if (currentRes == CoarseAdjRes) {
    // fine adjustment state
    setRes(FineAdjRes);
    stepsPerRevolution = 200 * FineAdjRes;
  } else if (currentRes == FineAdjRes) {
    // coarse adjustment state
    setRes(CoarseAdjRes);
    stepsPerRevolution = 200 * CoarseAdjRes;
  }
}

// ISR to switch modes from calibration/polaris aiming to tracking
void changeMode() {
  switch (currentState) {
    case CALIBRATE:
      currentState = STICK;
      // set current pos as 0
      alt.setCurrentPosition(0);
      az.setCurrentPosition(0);
      break;
    case STICK:
      currentState = TRACK;
      break;
    case TRACK:
      currentState = STICK;
      break;
  }
  Serial.println(currentState);
}

// Read the x/y values of the joystick and move the motors accordingly
void stickControl() {
  // read joystick values and set them to [-511, 512] instead of [0, 1023]
  int x = (analogRead(JoystickX) - 511);
  int y = (analogRead(JoystickY) - 511);

  // Check if the joystick values are above the Threshold to reduce drift or noise, then map + set speeds
  if (abs(x) > Threshold || abs(y) > Threshold) {
    // Map joystick values to speeds
    az.setSpeed(map(x, -512, 512, -MaxSpeed, MaxSpeed));
    alt.setSpeed(map(y, -512, 512, -MaxSpeed, MaxSpeed));

    // Move to implement acceleration (?)
    az.move(1);
    alt.move(1);
  } else {
    // Ignore small variations below the Threshold
    az.stop();
    alt.stop();
    az.setSpeed(0);
    alt.setSpeed(0);
  }
}

// use current alt az position (difference from Polaris) to calculate distance to Polaris, then use maths and current time to calculate angles
void altAzTrack(double elapsedTime) {
  // Alt/Az distance to polaris in radians: Current steps / steps/rev -> stepper revolutions, / GearRatio -> Telescope revolutions, * 2PI -> Telescope angle in radians
  altRad = alt.currentPosition() / stepsPerRevolution / AltGearRatio * (2 * PI);
  azRad = az.currentPosition() / stepsPerRevolution / AzGearRatio * (2 * PI);
  Serial.print("Rads: ");
  Serial.print(altRad);
  Serial.print(",");
  Serial.println(azRad);
  delay(200);

  // use spherical law of cosines to get angular distance in rad to Polaris - cos c = cos a * cos b
  radDistToPole = acos(cos(altRad) * cos(azRad));
  Serial.println(radDistToPole);

  // calculate the absolute target then translate to steps - Credit to ChatGPT
  altTarget = (radDistToPole * sin(EarthAngularVel * elapsedTime)) / (2 * PI) * AltGearRatio * stepsPerRevolution;
  azTarget = (atan2(sin(EarthAngularVel * elapsedTime), cos(EarthAngularVel * elapsedTime) * cos(radDistToPole)) + PI / 2) / (2 * PI) * AltGearRatio * stepsPerRevolution;

  alt.moveTo(altTarget);
  az.moveTo(azTarget);
}

void setup() {
  // init serial for debugging
  Serial.begin(9600);
  delay(2000);

  // set joystick pin modes to input
  pinMode(JoystickX, INPUT);
  pinMode(JoystickY, INPUT);

  // set joystick button pin to input - only pins 2 and 3 are interrupt pins
  pinMode(JoystickB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(JoystickB), changeRes, RISING);

  // set control btn pin to input
  pinMode(ModeBtn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ModeBtn), changeMode, RISING);

  // set stepper resolution
  setRes(CoarseAdjRes);

  // set stepper max speed
  alt.setMaxSpeed(MaxSpeed);
  az.setMaxSpeed(MaxSpeed);

  // set acceleration
  alt.setAcceleration(1);
  az.setAcceleration(1);

  currentState = CALIBRATE;
}

void loop() {
  // if not tracking, reset tracking start time
  if (currentState != TRACK && trackingStartTime) trackingStartTime = 0;

  switch (currentState) {
    case CALIBRATE:
    case STICK:
      stickControl();
      alt.run();
      az.run();
      break;
    case MANUAL:
      // already did stuff
      break;
    case AUTOAIM:
      // request positions from esp8266 and move accordingly
      break;
    case TRACK:
      if (!trackingStartTime) trackingStartTime = millis();
      if (currentRes != FineAdjRes) {
        setRes(FineAdjRes);
      }
      trackingElapsedTime = (millis() - trackingStartTime) * 100000;
      altAzTrack(trackingElapsedTime);
      break;
  }
}

#include <AccelStepper.h>
#include <SiderealPlanets.h>
#include <LiquidCrystal.h>

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

// LCD Pins
#define LCD_RS 15
#define LCD_EN 14
#define LCD_D4 3
#define LCD_D5 4
#define LCD_D6 5
#define LCD_D7 6

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
// #define MaxStepRes 32
// #define FineAdjRes MaxStepRes
// #define CoarseAdjRes MaxStepRes / 4

// Max speed in steps/sec
#define MaxFineSpeed 32
#define MaxCoarseSpeed 2000

// steps per revolution accounting for microsteps
const int stepsPerRevolution = 200 * 32;

// Length of a sidereal day in milliseconds
#define SiderealDayMillis 86164090.5

// Angular velocity of Earth
const double EarthAngularVel = (2 * PI) / SiderealDayMillis;

// initialize the steppers
AccelStepper az(AccelStepper::DRIVER, AltStep, AltDir);
AccelStepper alt(AccelStepper::DRIVER, AzStep, AzDir);

// init LCD
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// time when tracking began in millis
long trackingStartTime;
long trackingElapsedTime;

// possible program states
enum State {
  NORTH,    // find north to get 0,0
  POLARIS,  // find polaris
  STICK,    // aim using joystick
  MANUAL,   // aim by hand, disable steppers
  AUTOAIM,  // autoaim + track planets
  TRACK     // track current position
};

// program current state
volatile State currentState;

volatile bool fine = false;

// vars for simple tracking
double altRad, azRad, radDistToPole, altDisplacement, azDisplacement, altTarget, azTarget;

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
*/
struct Coord {
  // alt az in radians
  double alt;  // alt in radians
  double az;   // az in radians
  // ra dec
  double ra;   // RA in hrs
  double dec;  // Dec in deg
  // // rise set in hrs since midnight
  // double rise;
  // double set;
};

Coord Polaris;



// Joystick interrupt service routine to switch between coarse and fine manual control
void changeSpeed() {
  fine = !fine;
  lcd.setCursor(0, 1);
  lcd.print(fine ? "Fine  " : "Coarse");
}

// ISR to switch modes from calibration/polaris aiming to tracking
void changeMode() {
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (currentState) {
    case NORTH:
      currentState = POLARIS;
      // set current pos as 0
      alt.setCurrentPosition(0);
      az.setCurrentPosition(0);

      lcd.print("Aim at Polaris");
      break;
    case POLARIS:
      currentState = STICK;
      // store alt/az coordinates of Polaris
      Polaris.alt = alt.currentPosition() * PI * 2 / stepsPerRevolution / AzGearRatio;
      Polaris.az = az.currentPosition() * PI * 2 / stepsPerRevolution / AzGearRatio;

      lcd.print("StickCtrl: ");
    case STICK:
      currentState = TRACK;
      // Alt/Az distance to polaris in radians: Current steps / steps/rev -> stepper revolutions, / GearRatio -> Telescope revolutions, * 2PI -> Telescope angle in radians
      altRad = 2 * PI * alt.currentPosition() / stepsPerRevolution / AltGearRatio;
      azRad = 2 * PI * az.currentPosition() / stepsPerRevolution / AzGearRatio;

      altDisplacement = altRad - Polaris.alt;
      azDisplacement = azRad - Polaris.az;

      // use spherical law of cosines to get angular distance in rad to Polaris - cos c = cos a * cos b
      radDistToPole = acos(cos(altDisplacement) * cos(azDisplacement)) * 1.;
      lcd.print("Tracking");
      break;
    case TRACK:
      currentState = STICK;
      altRad = azRad = altDisplacement = azDisplacement = radDistToPole = altTarget = azTarget = 0;
      break;
  }
}

// Read the x/y values of the joystick and move the motors accordingly
void stickControl() {
  // read joystick values and set them to [-511, 512] instead of [0, 1023]
  int x = (analogRead(JoystickX) - 511);
  int y = (analogRead(JoystickY) - 511);

  // Check if the joystick values are above the Threshold to reduce drift or noise, then map + set speeds
  if (abs(x) > Threshold || abs(y) > Threshold) {
    // Map joystick values to speeds
    int speed = fine ? MaxFineSpeed : MaxCoarseSpeed;
    az.setSpeed(map(x, -512, 512, -speed, speed));
    alt.setSpeed(map(y, -512, 512, -speed, speed));

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
  alt.run();
  az.run();
}

// Possible tracking method
// use current alt az position (difference from Polaris) to calculate distance to Polaris, then use maths and elapsed time to calculate angles
void altAzTrack(long elapsedTime) {
  // calculate the absolute target then translate to steps - Credit to ChatGPT
  //altTarget = (radDistToPole * sin(EarthAngularVel * elapsedTime)) / (2 * PI) * AltGearRatio * stepsPerRevolution;
  //azTarget = (atan2(sin(EarthAngularVel * elapsedTime), cos(EarthAngularVel * elapsedTime) * cos(radDistToPole)) + PI / 2) / (2 * PI) * AltGearRatio * stepsPerRevolution;
  altTarget = (altRad * 1. + Polaris.alt + radDistToPole * sin(radDistToPole * (elapsedTime * 1. / 1000))) / stepsPerRevolution / AltGearRatio / (2 * PI);
  azTarget = (azRad * 1. + Polaris.az + radDistToPole * cos(radDistToPole * (elapsedTime * 1. / 1000))) / stepsPerRevolution / AzGearRatio / (2 * PI);

  alt.move(altTarget);
  az.move(azTarget);
  alt.run();
  az.run();
}

void setup() {
  // init serial for debugging
  // Serial.begin(9600);
  lcd.begin(16, 2);
  //delay(2000);

  // set joystick pin modes to input
  pinMode(JoystickX, INPUT);
  pinMode(JoystickY, INPUT);

  // set joystick button pin to input - only pins 2 and 3 are interrupt pins
  pinMode(JoystickB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(JoystickB), changeSpeed, RISING);

  // set control btn pin to input
  pinMode(ModeBtn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ModeBtn), changeMode, RISING);

  // set stepper resolution
  //setRes(CoarseAdjRes);

  // set stepper max speed
  alt.setMaxSpeed(MaxCoarseSpeed);
  az.setMaxSpeed(MaxCoarseSpeed);

  // set acceleration
  alt.setAcceleration(1);
  az.setAcceleration(1);

  currentState = NORTH;

  lcd.print("Aim North Horiz");
}

void loop() {
  lcd.setCursor(0, 1);
  // if not tracking, reset tracking start time
  if (currentState != TRACK) trackingStartTime = 0;
  switch (currentState) {
    case NORTH:
    case POLARIS:
    case STICK:
      stickControl();
      // lcd.print(alt.currentPosition() * 1. / stepsPerRevolution / AltGearRatio * 360);
      // lcd.print(",");
      // lcd.print(az.currentPosition() * 1. / stepsPerRevolution / AzGearRatio * 360);
      break;
      //case MANUAL:
      // stepper.disableoutputs?
      // break;
      //case AUTOAIM:
      // request positions from esp8266 and move accordingly
      // break;
    case TRACK:
      if (trackingStartTime == 0) trackingStartTime = millis();
      // trackingElapsedTime = (millis() - trackingStartTime)*1000;
      trackingElapsedTime += 1000;
      altAzTrack(trackingElapsedTime);
      lcd.print(trackingElapsedTime / 1000 / 100);
      break;
  }
}

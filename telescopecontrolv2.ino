#define debug_sidereal_planets

#include <AccelStepper.h>
#include <SiderealPlanets.h>
// #include <SiderealObjects.h>
#include <avr/pgmspace.h>

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

// initialize sidereal objects and planets
SiderealPlanets Planets;
// SiderealObjects Stars;

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
// Resolution is 1-32, multiples of 2.
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

void autoAim(Objects target) {

  // Planets.doPlans(target); // uses extra 10% program storage space - might be better for actual use
  Serial.println("calcs done");

  switch (target) {
    // do other stuff like rise/set for each
    case SUN:
      Planets.doSun();
      //planets.doSunRiseSetTimes();
      break;
    case MOON:
      Planets.doMoon();
      //planets.doMoonRiseSetTimes();
      //Planets.doLunarParallax(); // requires elevation
      break;
    case MERCURY:
      Planets.doMercury();
      break;
    case VENUS:
      Planets.doVenus();
      break;
    case MARS:
      Planets.doMars();
      break;
    case JUPITER:
      Planets.doJupiter();
      break;
    case SATURN:
      Planets.doSaturn();
      break;
    case URANUS:
      Planets.doUranus();
      break;
    case NEPTUNE:
      Planets.doNeptune();
      break;
  }
  planet.ra = Planets.getRAdec();
  planet.dec = Planets.getDeclinationDec();

  Serial.println("radec done");

  Planets.setRAdec(planet.ra, planet.dec);
  Planets.doRAdec2AltAz();
  planet.alt = Planets.getAltitude();
  planet.az = Planets.getAzimuth();

  Serial.println("altaz done");

  // Planets.setAltAz(planet.alt, planet.az);
  // Planets.doRefractionC(pressure, temperature in C); // integrate BMP280 sensor to get these, for no refraction compensation for now

  Serial.print("Alt, Az: ");
  Serial.print(planet.alt);
  Serial.print(",");
  Serial.println(planet.az);

  // after getting coords, ensure it is above horizon, then calculate distance between current position and target, then move accordingly (taking into account the gear ratios)
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

  // Init planets, integrate GPS module to get these
  Planets.begin();
  Planets.setTimeZone(-5);  // Relative to GMT: EST = GMT-5
  Planets.useAutoDST();

  // get these from gps module
  Planets.setLatLong(41.8, -72.9);  // Avon, CT
  Planets.setGMTdate(2023, 6, 23);
  Planets.setGMTtime(18, 9, 0);

  // for debugging and testing
  currentState = AUTOAIM;

  autoAim(MARS); // for some reason venus, jupiter, saturn don't work
}

void loop()
{/*
  if (currentState == COARSE || currentState == FINE) {
    // manualControl();
  } else if (currentState == AUTOAIM) {
    // get target planet info, then aim
  } else if (currentState == TRACK) {
    // aim then track either by calculating position every loop or aiming in a circle around polaris
  }*/
  while(1);
}

g
#include <AccelStepper.h>

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

// Number of microsteps per step
// Multiples of 2, 1 to 32
#define StepRes 32

// Max speed in steps/sec
#define maxSpeed 50 * StepRes

// Length of a sidereal day in seconds
#define SiderealDay 86164.0905

// Joystick pins
// X output
#define joystickX A0

// Y output
#define joystickY A1

// Button press
#define joystickB 2

// Joystick detection threshold
#define threshold 24

// Joystick offsets compensate for drift
byte xOffset;
byte yOffset;

// initialize the steppers
AccelStepper az(AccelStepper::DRIVER, PitchStep, PitchDir);
AccelStepper alt(AccelStepper::DRIVER, YawStep, YawDir);

// fine adjustment state
volatile bool fine = false;

// coordinate set
struct Coord {
  double x;
  double y;
  bool button = false;
};

// Target position based on tracking
// Coord targetPos;

// Current position of the telescope
// Coord currentPos;


// Set the resolution (microsteps/step) of the steppers. The res pins are common to both so it is only done once.
// Resolution is 1-32, multiples of 2.
void setRes(int res = 32) {
  int x = log(res) / log(2);
  digitalWrite(M0, (x & 1) == 0 ? LOW : HIGH);
  digitalWrite(M1, (x & 2) == 0 ? LOW : HIGH);
  digitalWrite(M2, (x & 4) == 0 ? LOW : HIGH);
}

// Interrupt service routine
void buttonInterrupt() {
  fine = !fine;
  fine == true ? setRes(32) : setRes(8);
  Serial.println("State Change");
}

// Read the x/y values of the joystick and return the appropriate speeds
int readJoystick(int Xpin, int Ypin) {
  // joystick values
  int x, y;

  // speeds
  int xSpeed, ySpeed;

  // analogread the joystick values and make them [-511, 512] instead of [0, 1023]
  x = analogRead(Xpin) - (511);
  y = analogRead(Ypin) - (511);
  xSpeed = map(x, -512, 512, -maxSpeed, maxSpeed);
  ySpeed = map(y, -512, 512, -maxSpeed, maxSpeed);


  // Check if the joystick values are above the threshold to reduce drift or noise, then map + set speeds
  if (abs(x) > threshold || abs(y) > threshold) {  // Map the values to speeds
    az.setSpeed(xSpeed);
    alt.setSpeed(ySpeed);

    // Debugging
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(" y: ");
    Serial.print(y);
    Serial.print(" xSpeed: ");
    Serial.print(xSpeed);
    Serial.print(" ySpeed: ");
    Serial.println(ySpeed);
  } else  // Ignore small variations below the threshold
  {
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
  attachInterrupt(digitalPinToInterrupt(joystickB), buttonInterrupt, FALLING);

  // init serial for debugging
  Serial.begin(9600);

  // set stepper resolution
  setRes(StepRes);

  // set stepper max speed
  alt.setMaxSpeed(maxSpeed);
  az.setMaxSpeed(maxSpeed);

  // set acceleration (doesn't work)
  alt.setAcceleration(.1);
  az.setAcceleration(.1);
}

void loop()  // loop
{
  readJoystick(joystickX, joystickY);
  alt.run();
  az.run();
}
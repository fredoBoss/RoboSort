/*
 * RoboSort - Arduino Mega firmware
 *
 * The Mega owns every physical output. The Orange Pi only tells it what
 * the camera saw; it never drives a pin.
 *
 * Serial protocol from the Pi (see orange_pi/serial_link.py), 9600 baud:
 *     B\n   biodegradable      -> sorting servo to the bio bin
 *     N\n   non-biodegradable  -> sorting servo to the non-bio bin
 *     X\n   nothing detected   -> hold position
 *
 * The Pi sends on change only, so a command is a latch, not a pulse.
 *
 * Manual test commands, typed into a serial monitor at 9600 baud. Any of
 * them puts the rover in MANUAL: the wheels stop and autonomous driving
 * is suspended until 'A' resumes it, so a bench test cannot be run over
 * by the navigation loop.
 *     U  lift up          D  lift down
 *     S  stretch out      R  retract
 *     H  home - every servo back to 90
 *     A  resume autonomous driving
 *     ?  print the list
 * Lowercase is accepted, and the line ending does not matter.
 *
 * Tabs:
 *     motorControl.ino  4 drive motors
 *     Servo.ino         4 MG996R servos (2x lift, stretch, sort) + conveyor
 *     Ultrasonic.ino    4 HC-SR04 rangefinders
 *
 * Obstacle avoidance runs here, independent of the Pi - a silent or
 * unplugged Pi must never stop the rover from braking.
 */

#include <Servo.h>

// ---------------------------------------------------------------- pins
// Drive: two L298N-style drivers, one per side, both motors on a side
// share a channel. EN pins must be PWM-capable.
const uint8_t PIN_L_EN   = 2;   // PWM
const uint8_t PIN_L_IN1  = 22;
const uint8_t PIN_L_IN2  = 23;
const uint8_t PIN_R_EN   = 3;   // PWM
const uint8_t PIN_R_IN1  = 24;
const uint8_t PIN_R_IN2  = 25;

// Conveyor belt motor (single direction is enough, but wired H-bridge)
const uint8_t PIN_CONV_EN  = 4; // PWM
const uint8_t PIN_CONV_IN1 = 26;
const uint8_t PIN_CONV_IN2 = 27;

// Servos - 4x MG996R, 180 degree. The lift joint is driven by a pair,
// one servo per side of the pivot; stretch and sort are single.
// Stretch and sort keep their original pins so existing wiring stands;
// the second lift servo is the new one, on 12.
const uint8_t PIN_SERVO_LIFT_L  = 9;
const uint8_t PIN_SERVO_STRETCH = 10;
const uint8_t PIN_SERVO_SORT    = 11;
const uint8_t PIN_SERVO_LIFT_R  = 12;

// Ultrasonics: front, left, right, rear
const uint8_t PIN_TRIG[4] = {30, 32, 34, 36};
const uint8_t PIN_ECHO[4] = {31, 33, 35, 37};

// ------------------------------------------------------------- tuning
const uint8_t  DRIVE_SPEED   = 160;   // 0-255 cruise
const uint8_t  TURN_SPEED    = 180;
const uint16_t STOP_CM       = 25;    // brake closer than this
const uint16_t CLEAR_CM      = 40;    // consider a direction open past this
const uint16_t TURN_MS       = 600;   // how long an avoidance turn runs
const uint32_t CMD_TIMEOUT_MS = 5000; // Pi silence before we assume no target

// ------------------------------------------------------------- state
enum Command { CMD_NONE = 'X', CMD_BIO = 'B', CMD_NONBIO = 'N' };

char     lastCommand   = CMD_NONE;
uint32_t lastCommandAt = 0;
bool     pickupBusy    = false;   // true while the arm sequence is running
bool     manualMode    = false;   // bench testing - navigation suspended

uint16_t distFront, distLeft, distRight, distRear;

void setup() {
  Serial.begin(9600);             // to the Orange Pi (USB / UART0)

  motorSetup();
  servoSetup();
  ultrasonicSetup();

  Serial.println(F("RoboSort Mega ready"));
  printHelp();
}

void loop() {
  readSerialCommand();

  // Bench testing: the wheels stay stopped and nothing below runs, so a
  // manual servo move is not immediately undone by the navigation logic.
  if (manualMode) return;

  readAllDistances();

  // Safety first, and without asking the Pi: anything close in front stops
  // the wheels regardless of what the vision pipeline is reporting.
  if (distFront > 0 && distFront < STOP_CM) {
    driveStop();
    avoidObstacle();
    return;
  }

  // A fresh B/N means the camera is looking at trash - collect it.
  if (isCommandFresh() && (lastCommand == CMD_BIO || lastCommand == CMD_NONBIO)) {
    driveStop();
    runPickupSequence();
    sortTo(lastCommand);
    // Consume the command so one detection triggers one pickup. The Pi
    // sends on change, so it will re-send B/N for the next item.
    lastCommand = CMD_NONE;
    return;
  }

  driveForward(DRIVE_SPEED);
}

// --------------------------------------------------------- serial in
/*
 * One byte per line: either a classification from the Pi, or a manual
 * test command. Anything unrecognised is refused rather than guessed at
 * - a garbled byte must not fling the sorting gate over.
 */
void readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;
    if (c >= 'a' && c <= 'z') c -= 32;   // typed by hand, accept lowercase

    switch (c) {
      // From the Pi: latch it, loop() decides what to do about it.
      case CMD_BIO:
      case CMD_NONBIO:
      case CMD_NONE:
        lastCommand   = c;
        lastCommandAt = millis();
        Serial.print(F("ACK "));
        Serial.println(c);
        break;

      // Manual movement tests. Each blocks until the joint has settled.
      case 'U': enterManual(); armLiftUp();     ackMove(F("lift up"));     break;
      case 'D': enterManual(); armLiftDown();   ackMove(F("lift down"));   break;
      case 'S': enterManual(); armStretchOut(); ackMove(F("stretch out")); break;
      case 'R': enterManual(); armRetract();    ackMove(F("retract"));     break;
      case 'H': enterManual(); servoResetAll(); ackMove(F("home 90"));     break;

      case 'A': resumeAuto(); break;
      case '?': printHelp();  break;

      default:
        Serial.print(F("NAK "));
        Serial.println(c);
    }
  }
}

void ackMove(const __FlashStringHelper *what) {
  Serial.print(F("ACK "));
  Serial.println(what);
}

// Entered by any test command, so a bench session never needs a mode
// switch typed first. Stopping the wheels is the whole point of it.
void enterManual() {
  if (manualMode) return;
  manualMode = true;
  driveStop();
  Serial.println(F("MANUAL - wheels stopped, send A to resume"));
}

/*
 * Back to autonomous. The latched command is dropped on the way out: a B
 * or N that arrived during the bench session would otherwise fire a
 * pickup the moment the rover starts driving again.
 */
void resumeAuto() {
  manualMode  = false;
  lastCommand = CMD_NONE;
  Serial.println(F("AUTO - navigating"));
}

void printHelp() {
  Serial.println(F("commands: U lift up  D lift down  S stretch out"));
  Serial.println(F("          R retract  H home 90    A resume auto"));
  Serial.println(F("          B bio      N non-bio    X none"));
}

bool isCommandFresh() {
  return (millis() - lastCommandAt) < CMD_TIMEOUT_MS;
}

// ------------------------------------------------------- avoidance
// Blocking turns are fine here: the wheels are already stopped and
// nothing else needs servicing during the manoeuvre.
void avoidObstacle() {
  driveBackward(DRIVE_SPEED);
  delay(300);
  driveStop();

  readAllDistances();

  if (distLeft > CLEAR_CM && distLeft >= distRight) {
    driveTurnLeft(TURN_SPEED);
  } else if (distRight > CLEAR_CM) {
    driveTurnRight(TURN_SPEED);
  } else {
    driveTurnLeft(TURN_SPEED);   // boxed in - spin until something opens
  }

  delay(TURN_MS);
  driveStop();
}

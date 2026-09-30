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
 * Nothing moves on its own. The rover boots in MANUAL with the wheels
 * stopped, and a motor only runs when a command arrives - typed into a
 * serial monitor at 9600 baud, or sent by a program on the same port. 'A'
 * hands driving to the autonomous loop; any other command below takes it
 * back, so a bench test cannot be run over by the navigation loop.
 *     8  or FORWARD       2  or BACKWARD / BACK
 *     4  or LEFT          6  or RIGHT
 *     5  or STOP - wheels and conveyor
 *     U  lift up          D  lift down
 *     S  stretch out      R  retract
 *     H  home - every servo back to 90
 *     C  conveyor on      O  conveyor off
 *     A  autonomous driving
 *     ?  print the list
 * One command per line; a line that is not exactly one of these is
 * refused whole. Every line is answered: "RX <line>" the moment it
 * arrives, then "OK ..." when done or "FAIL ... - reason" when refused
 * (see endLine()). '?' adds a STATUS line whose uptime shows whether the
 * Mega has reset. A drive command runs for MANUAL_MOVE_MS and then stops by
 * itself - send it again to keep going - so a dropped cable or a forgotten
 * key cannot leave the rover driving. Forward is refused, or cut short,
 * while either sensor reads closer than STOP_CM. Lowercase is accepted,
 * and any line ending works, including none.
 *
 * Tabs:
 *     motorControl.ino  2 drive motors, one per side, on relays or L298Ns
 *     Servo.ino         4 MG996R servos (2x lift, stretch, sort) + conveyor relay
 *     Ultrasonic.ino    2 HC-SR04 rangefinders, left and right
 *
 * Obstacle avoidance runs here, independent of the Pi - a silent or
 * unplugged Pi must never stop the rover from braking.
 */

#include <Servo.h>

// ---------------------------------------------------------------- pins
// Drive: two motors at diagonally opposite corners, back-left ("left") and
// front-right ("right"); the other two wheels roll free. DRIVE_BY_RELAY
// picks the hardware: true for relays, false for two L298N boards.
const bool DRIVE_BY_RELAY = true;

// Relays: a 4-channel module, two relays per motor - relays 1+2 on the
// left motor, 3+4 on the right. Every relay: NO to 12V+, NC to battery-,
// COM to one motor wire. Power the module's VCC from the 5V buck, not the
// Mega - four coils draw ~0.3 A. Most modules switch on when IN is LOW;
// if a relay clicks on when it should be off, the module is active-HIGH
// and DRIVE_RELAY_ACTIVE_LOW must be false.
const bool DRIVE_RELAY_ACTIVE_LOW = true;

// L298N: two boards, one per motor on channel A (OUT1/OUT2), ENA jumper
// pulled - ENA carries the PWM speed, so the EN pins must be PWM-capable.
// Channel B is unused.
//
// Either way, if a wheel runs backwards, swap its two motor wires.
//                                 relay module     L298N
const uint8_t PIN_L_IN1  = 22;  // IN1 - forward    board 1 IN1
const uint8_t PIN_L_IN2  = 23;  // IN2 - backward   board 1 IN2
const uint8_t PIN_R_IN1  = 24;  // IN3 - forward    board 2 IN1
const uint8_t PIN_R_IN2  = 25;  // IN4 - backward   board 2 IN2
const uint8_t PIN_L_EN   = 2;   // unused           board 1 ENA (PWM)
const uint8_t PIN_R_EN   = 3;   // unused           board 2 ENA (PWM)

// Conveyor belt motor, switched on and off by a relay module - one
// direction, full speed. Most relay modules are active-LOW (IN pulled to
// GND closes the relay); if the belt runs when it should be stopped, the
// module is active-HIGH and CONV_RELAY_ACTIVE_LOW must be false.
const uint8_t PIN_CONV_RELAY        = 26;
const bool    CONV_RELAY_ACTIVE_LOW = true;

// Servos - 4x MG996R, 180 degree. The lift joint is driven by a pair,
// one servo per side of the pivot; stretch and sort are single.
// Stretch and sort keep their original pins so existing wiring stands;
// the second lift servo is the new one, on 12.
const uint8_t PIN_SERVO_LIFT_L  = 9;
const uint8_t PIN_SERVO_STRETCH = 10;
const uint8_t PIN_SERVO_SORT    = 8;
const uint8_t PIN_SERVO_LIFT_R  = 12;

// Ultrasonics: left, right. These are the pins the left and right sensors
// had when there were four, so existing wiring stands; 30/31 and 36/37
// (the old front and rear) are free.
const uint8_t PIN_TRIG[2] = {32, 34};
const uint8_t PIN_ECHO[2] = {33, 35};

// ------------------------------------------------------------- tuning
// Speeds are PWM for the L298N. Relays have no speed: 0 is stop and
// anything else is full battery voltage.
const uint8_t  DRIVE_SPEED   = 160;   // 0-255 cruise
const uint8_t  VEER_SPEED    = DRIVE_BY_RELAY ? 0 : 100;  // inside wheel while bending away
const uint8_t  TURN_SPEED    = 180;
const uint16_t STOP_CM       = 20;    // either side closer than this: stop, back out, turn
const uint16_t VEER_CM       = 40;    // either side closer than this: keep going, bend away
const uint16_t VEER_HYST_CM  = 10;    // ...and keep bending until both sides clear VEER_CM by this
const uint16_t BACKUP_MS     = 300;   // blind - there is no rear sensor
const uint16_t TURN_MS       = 600;   // how long an avoidance turn runs
const uint16_t MANUAL_MOVE_MS = 1000; // how long one manual drive command runs
const uint32_t CMD_TIMEOUT_MS = 5000; // Pi silence before we assume no target

// ------------------------------------------------------------- state
enum Command { CMD_NONE = 'X', CMD_BIO = 'B', CMD_NONBIO = 'N' };

char     lastCommand   = CMD_NONE;
uint32_t lastCommandAt = 0;
bool     pickupBusy    = false;   // true while the arm sequence is running
bool     manualMode    = true;    // boots stopped - navigation waits for 'A'
char     manualMove    = 0;       // drive command running: '8' '2' '4' '6', 0 = none
uint32_t manualMoveAt  = 0;       // when it was last sent

uint16_t distLeft, distRight;

void setup() {
  Serial.begin(9600);             // to the Orange Pi (USB / UART0)

  pinMode(LED_BUILTIN, OUTPUT);   // heartbeat, see heartbeat()

  motorSetup();
  servoSetup();
  ultrasonicSetup();

  Serial.println(F("RoboSort Mega ready"));
  printHelp();
  Serial.println(F("MANUAL - wheels stopped, send a drive command or A"));
}

void loop() {
  heartbeat();
  readSerialCommand();

  // Manual: nothing below runs, so a manual move is not undone by the
  // navigation logic. Only a drive command in progress needs watching.
  if (manualMode) {
    serviceManualMove();
    return;
  }

  readAllDistances();

  // Safety first, and without asking the Pi: anything too close on either
  // side stops the wheels regardless of what the vision pipeline is
  // reporting.
  if (distLeft < STOP_CM || distRight < STOP_CM) {
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

  // Something near but not yet close: keep moving and bend away from it.
  // An arc scrubs the two free wheels far less than stopping to spin.
  if (veerAway()) return;

  driveForward(DRIVE_SPEED);
}

/*
 * Blink the on-board L LED (pin 13) twice a second while loop() is alive.
 * When the serial monitor goes quiet this tells the two failures apart:
 * still blinking - the Mega is fine and the USB-serial link dropped;
 * frozen on or off - the Mega itself hung, almost always a supply sag.
 */
void heartbeat() {
  static uint32_t lastToggle = 0;
  if (millis() - lastToggle >= 250) {
    lastToggle = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

// --------------------------------------------------------- serial in
/*
 * One command per line: a classification from the Pi, or a manual command.
 * The whole line is matched as one command, so a typed word is either
 * understood or refused - it can never fire a chain of one-letter
 * commands, the way "forward" once ran conveyor off, retract, AUTONOMOUS,
 * lift down. Anything unrecognised is refused rather than guessed at - a
 * garbled byte must not fling the sorting gate over.
 *
 * A line ends at \n or \r. A serial monitor set to "No line ending" sends
 * neither, so a line also ends once LINE_IDLE_MS pass with nothing new -
 * it sends a whole line at once, so the gap only comes after the last
 * character. Spaces are dropped, so " 8 " is "8".
 */
const uint8_t  LINE_MAX     = 12;    // longest command is "BACKWARD"
const uint16_t LINE_IDLE_MS = 100;

char     rxLine[LINE_MAX + 1];
uint8_t  rxLen     = 0;
bool     rxTooLong = false;
uint32_t rxLastAt  = 0;

void readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    rxLastAt = millis();

    if (c == '\n' || c == '\r') {
      endLine();
    } else if (c == ' ' || c == '\t') {
      // dropped
    } else if (rxLen < LINE_MAX) {
      if (c >= 'a' && c <= 'z') c -= 32;   // typed by hand, accept lowercase
      rxLine[rxLen++] = c;
    } else {
      rxTooLong = true;
    }
  }

  if ((rxLen > 0 || rxTooLong) && millis() - rxLastAt >= LINE_IDLE_MS) {
    endLine();
  }
}

/*
 * Every line is echoed as "RX <line>" before anything else happens, and
 * flushed onto the wire before a relay or servo can move - if switching a
 * motor knocks out the USB link, the last RX still proves the command
 * arrived. Exactly one result follows:
 *     OK <what>              done
 *     FAIL <what> - <why>    refused, nothing moved
 * and a drive move reports later how it ended:
 *     DONE <move> - ...      its time ran out
 *     STOP forward - ...     an obstacle cut it short
 */
void endLine() {
  rxLine[rxLen] = '\0';
  if (rxLen > 0 || rxTooLong) {
    Serial.print(F("RX "));
    Serial.print(rxLine);
    if (rxTooLong) Serial.print(F("..."));
    Serial.println();
    Serial.flush();
  }

  if (rxTooLong) {
    Serial.println(F("FAIL - too long, not a command (send ? for the list)"));
  } else if (rxLen > 0) {
    runCommand(rxLine);
  }
  rxLen     = 0;
  rxTooLong = false;
}

// The drive moves have a word as well as a number; everything else is one
// character. 0 for a word that is none of them.
char wordToCommand(const char *word) {
  if (strcmp_P(word, PSTR("FORWARD"))  == 0) return '8';
  if (strcmp_P(word, PSTR("BACKWARD")) == 0) return '2';
  if (strcmp_P(word, PSTR("BACK"))     == 0) return '2';
  if (strcmp_P(word, PSTR("LEFT"))     == 0) return '4';
  if (strcmp_P(word, PSTR("RIGHT"))    == 0) return '6';
  if (strcmp_P(word, PSTR("STOP"))     == 0) return '5';
  return 0;
}

void runCommand(const char *line) {
  char c = (line[1] == '\0') ? line[0] : wordToCommand(line);

  switch (c) {
    // From the Pi: latch it, loop() decides what to do about it.
    case CMD_BIO:
    case CMD_NONBIO:
    case CMD_NONE:
      lastCommand   = c;
      lastCommandAt = millis();
      Serial.print(F("OK "));
      if (c == CMD_BIO)         Serial.print(F("bio"));
      else if (c == CMD_NONBIO) Serial.print(F("non-bio"));
      else                      Serial.print(F("nothing detected"));
      if (manualMode && c != CMD_NONE) Serial.print(F(" - no pickup in MANUAL, send A"));
      Serial.println();
      break;

    // Manual driving - each command runs MANUAL_MOVE_MS, then stops.
    case '8':
    case '2':
    case '4':
    case '6':
      manualDrive(c);
      break;
    case '5': enterManual(); conveyorStop(); ok(F("stop - wheels and conveyor stopped")); break;

    // Manual movement tests. Each blocks until the joint has settled.
    case 'U': enterManual(); armLiftUp();     ok(F("lift up"));                 break;
    case 'D': enterManual(); armLiftDown();   ok(F("lift down"));               break;
    case 'S': enterManual(); armStretchOut(); ok(F("stretch out"));             break;
    case 'R': enterManual(); armRetract();    ok(F("retract"));                 break;
    case 'H': enterManual(); servoResetAll(); ok(F("home - all servos at 90")); break;
    case 'C': enterManual(); conveyorRun();   ok(F("conveyor on"));             break;
    case 'O': enterManual(); conveyorStop();  ok(F("conveyor off"));            break;

    case 'A': resumeAuto(); break;
    case '?': printHelp(); printStatus(); break;

    default:
      Serial.print(F("FAIL "));
      Serial.print(line);
      Serial.println(F(" - unknown command (send ? for the list)"));
  }
}

void ok(const __FlashStringHelper *what) {
  Serial.print(F("OK "));
  Serial.println(what);
}

/*
 * Entered by any manual command, so a bench session never needs a mode
 * switch typed first. The wheels stop every time, not just on the way
 * in: a servo test blocks for up to a second, and the rover must not keep
 * driving through it.
 */
void enterManual() {
  driveStop();
  manualMove = 0;
  if (manualMode) return;
  manualMode = true;
  Serial.println(F("MANUAL - wheels stopped, send A to resume"));
}

/*
 * Start a manual drive move, or keep one going. Repeating the same move
 * only pushes its deadline out, so a program that re-sends '8' every few
 * hundred ms drives smoothly - no stop in between to click the relays.
 */
void manualDrive(char move) {
  if (!manualMode) enterManual();   // taking over from autonomous driving

  if (move == '8' && sideBlocked()) {
    driveStop();
    manualMove = 0;
    Serial.print(F("FAIL forward - "));
    printBlocked();
    return;
  }

  switch (move) {
    case '8': driveForward(DRIVE_SPEED);  break;
    case '2': driveBackward(DRIVE_SPEED); break;
    case '4': driveTurnLeft(TURN_SPEED);  break;
    case '6': driveTurnRight(TURN_SPEED); break;
  }
  manualMove   = move;
  manualMoveAt = millis();   // after drive(), which may pause before a reversal

  Serial.print(F("OK "));
  Serial.print(moveName(move));
  Serial.print(F(" - running "));
  Serial.print(MANUAL_MOVE_MS);
  Serial.println(F(" ms"));
}

const __FlashStringHelper *moveName(char move) {
  switch (move) {
    case '8': return F("forward");
    case '2': return F("backward");
    case '4': return F("turn left");
    default:  return F("turn right");
  }
}

// Every loop in MANUAL: end a drive move when its time is up, and cut a
// forward move short if either side gets too close.
void serviceManualMove() {
  if (manualMove == 0) return;

  if (millis() - manualMoveAt >= MANUAL_MOVE_MS) {
    driveStop();
    Serial.print(F("DONE "));
    Serial.print(moveName(manualMove));
    Serial.println(F(" - time up, send it again to keep going"));
    manualMove = 0;
  } else if (manualMove == '8' && sideBlocked()) {
    driveStop();
    manualMove = 0;
    Serial.print(F("STOP forward - "));
    printBlocked();
  }
}

bool sideBlocked() {
  readAllDistances();
  return distLeft < STOP_CM || distRight < STOP_CM;
}

void printBlocked() {
  Serial.print(F("blocked: left "));
  Serial.print(distLeft);
  Serial.print(F(" cm, right "));
  Serial.print(distRight);
  Serial.print(F(" cm (limit "));
  Serial.print(STOP_CM);
  Serial.println(F(" cm)"));
}

/*
 * One line of state. After a silence it tells the two failures apart: a
 * small uptime means the Mega reset, a large one means it kept running
 * and only the USB link dropped.
 */
void printStatus() {
  readAllDistances();
  Serial.print(F("STATUS "));
  Serial.print(manualMode ? F("MANUAL") : F("AUTO"));
  Serial.print(F(", manual move: "));
  Serial.print(manualMove ? moveName(manualMove) : F("none"));
  Serial.print(F(", conveyor "));
  Serial.print(conveyorIsOn() ? F("on") : F("off"));
  Serial.print(F(", left "));
  Serial.print(distLeft);
  Serial.print(F(" cm, right "));
  Serial.print(distRight);
  Serial.print(F(" cm, up "));
  Serial.print(millis() / 1000);
  Serial.println(F(" s"));
}

/*
 * Back to autonomous. The latched command is dropped on the way out: a B
 * or N that arrived during the bench session would otherwise fire a
 * pickup the moment the rover starts driving again. A belt left running
 * by 'C' is stopped too - in AUTO the pickup sequence owns it.
 */
void resumeAuto() {
  manualMode  = false;
  manualMove  = 0;
  lastCommand = CMD_NONE;
  conveyorStop();
  ok(F("autonomous - driving on its own, send 5 to stop"));
}

void printHelp() {
  Serial.println(F("commands: 8 or FORWARD  2 or BACKWARD  4 or LEFT  6 or RIGHT  5 or STOP"));
  Serial.println(F("          U lift up  D lift down  S stretch out  R retract"));
  Serial.println(F("          H home 90  C conveyor on  O conveyor off"));
  Serial.println(F("          A autonomous  ? this list + status"));
  Serial.println(F("          B bio      N non-bio    X none"));
  Serial.println(F("replies:  RX received, OK done, FAIL refused, DONE/STOP move ended"));
}

bool isCommandFresh() {
  return (millis() - lastCommandAt) < CMD_TIMEOUT_MS;
}

// ------------------------------------------------------- avoidance
/*
 * With only a left and a right sensor the one decision is which way to
 * turn: back out so the front has room to swing, then turn toward the
 * more open side. If both are still tight, the next loop backs out and
 * turns again. Blocking turns are fine here: the wheels are already
 * stopped and nothing else needs servicing during the manoeuvre.
 */
void avoidObstacle() {
  driveBackward(DRIVE_SPEED);
  delay(BACKUP_MS);
  driveStop();

  readAllDistances();

  if (distLeft >= distRight) {
    driveTurnLeft(TURN_SPEED);
  } else {
    driveTurnRight(TURN_SPEED);
  }

  delay(TURN_MS);
  driveStop();
}

/*
 * Bend away from something near but not yet close, by slowing (L298N) or
 * stopping (relays) the wheel on the far side. Returns false when there
 * is nothing to bend away from.
 *
 * The bend starts under VEER_CM, picks its direction once, and holds it
 * until both sides read VEER_HYST_CM past the line. Without that, a
 * reading hovering on the line - or two walls about equally near - would
 * flip the drive every loop and chatter the relays to death.
 */
bool veerAway() {
  static int8_t bend = 0;   // +1 bending right, -1 bending left, 0 straight
  uint16_t nearest = min(distLeft, distRight);

  if (bend == 0 && nearest < VEER_CM) {
    bend = (distLeft < distRight) ? 1 : -1;
  } else if (nearest >= VEER_CM + VEER_HYST_CM) {
    bend = 0;
  }

  if (bend > 0) {
    driveCurve(DRIVE_SPEED, VEER_SPEED);   // nearer on the left - bend right
  } else if (bend < 0) {
    driveCurve(VEER_SPEED, DRIVE_SPEED);   // nearer on the right - bend left
  }
  return bend != 0;
}

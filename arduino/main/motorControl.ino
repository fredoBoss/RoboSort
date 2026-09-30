/*
 * RoboSort - drive motors
 *
 * Two DC wheel motors, back-left ("left") and front-right ("right"); the
 * other two wheels roll free. Tank steering, so a turn is one side forward
 * and the other reversed. DRIVE_BY_RELAY in main.ino picks the hardware,
 * and the wiring for both is there:
 *
 *   relays  Two per motor, wired as an H-bridge: IN1's relay puts 12V on
 *           one motor wire (forward), IN2's on the other (backward), both
 *           off grounds both wires (stop). No relay combination can short
 *           the battery. On/off only - any speed above 0 is full speed.
 *   L298N   One board per motor on channel A, speed as PWM on ENA.
 *
 * With the drive wheels on a diagonal, a spin turns about the middle of
 * the chassis, the same both ways. An arc does not: it pivots near the
 * slower wheel, so bending left swings the front around the back-left
 * wheel while bending right swings the back around the front-right one.
 */

/*
 * A spinning motor thrown straight into reverse draws about twice its
 * stall current - it pits relay contacts and overloads an L298N alike.
 * drive() remembers which way each side last ran and when a motor was
 * last powered, and stops for whatever is left of this before any side
 * flips direction.
 */
const uint16_t REVERSE_PAUSE_MS = 200;

static int8_t   lastDirL = 0, lastDirR = 0;   // way each side last ran, 0 = never
static bool     powered = false;              // a motor is running right now
static uint32_t lastPoweredMs = 0;            // last moment a motor was running

// Direction helpers ------------------------------------------------
static void relaySet(uint8_t pin, bool on) {
  if (DRIVE_RELAY_ACTIVE_LOW) on = !on;
  digitalWrite(pin, on ? HIGH : LOW);
}

// One side. dir is +1 forward, -1 backward, 0 stop.
static void sideWrite(uint8_t pinIn1, uint8_t pinIn2, uint8_t pinEn,
                      int8_t dir, uint8_t speed) {
  if (DRIVE_BY_RELAY) {
    relaySet(pinIn1, dir > 0);
    relaySet(pinIn2, dir < 0);
  } else {
    digitalWrite(pinIn1, dir > 0 ? HIGH : LOW);
    digitalWrite(pinIn2, dir < 0 ? HIGH : LOW);
    analogWrite(pinEn, dir == 0 ? 0 : speed);
  }
}

// Both sides at once. Everything below goes through here, so the
// reversal pause cannot be skipped by a new caller.
static void drive(int8_t dirL, uint8_t speedL, int8_t dirR, uint8_t speedR) {
  if (speedL == 0) dirL = 0;
  if (speedR == 0) dirR = 0;

  if (powered) lastPoweredMs = millis();
  bool flip = (dirL != 0 && dirL == -lastDirL) ||
              (dirR != 0 && dirR == -lastDirR);
  uint32_t idle = millis() - lastPoweredMs;
  if (flip && idle < REVERSE_PAUSE_MS) {
    sideWrite(PIN_L_IN1, PIN_L_IN2, PIN_L_EN, 0, 0);
    sideWrite(PIN_R_IN1, PIN_R_IN2, PIN_R_EN, 0, 0);
    delay(REVERSE_PAUSE_MS - idle);
  }

  sideWrite(PIN_L_IN1, PIN_L_IN2, PIN_L_EN, dirL, speedL);
  sideWrite(PIN_R_IN1, PIN_R_IN2, PIN_R_EN, dirR, speedR);

  if (dirL != 0) lastDirL = dirL;
  if (dirR != 0) lastDirR = dirR;
  powered = (dirL != 0 || dirR != 0);
}

// Public API -------------------------------------------------------
/*
 * Pins are written "stopped" before they become outputs, so no relay
 * clicks on at boot - the same trick as the conveyor relay. In L298N mode
 * analogWrite() makes the EN pins outputs itself.
 */
void motorSetup() {
  driveStop();
  const uint8_t pins[] = {PIN_L_IN1, PIN_L_IN2, PIN_R_IN1, PIN_R_IN2};
  for (uint8_t i = 0; i < sizeof(pins); i++) pinMode(pins[i], OUTPUT);
}

void driveForward(uint8_t speed) {
  drive(1, speed, 1, speed);
}

void driveBackward(uint8_t speed) {
  drive(-1, speed, -1, speed);
}

// Forward on an arc - the slower side is the inside of the curve. With
// relays a side is either full speed or stopped, so the arc is a pivot.
void driveCurve(uint8_t leftSpeed, uint8_t rightSpeed) {
  drive(1, leftSpeed, 1, rightSpeed);
}

void driveTurnLeft(uint8_t speed) {
  drive(-1, speed, 1, speed);
}

void driveTurnRight(uint8_t speed) {
  drive(1, speed, -1, speed);
}

void driveStop() {
  drive(0, 0, 0, 0);
}

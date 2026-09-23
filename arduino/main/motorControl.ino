/*
 * RoboSort - drive motors
 *
 * Four DC wheel motors on two H-bridge channels: the two left wheels are
 * wired in parallel on one channel, the two right wheels on the other.
 * Tank steering, so a turn is one side forward and the other reversed.
 */

// Direction helpers ------------------------------------------------
static void leftDrive(int8_t dir, uint8_t speed) {
  digitalWrite(PIN_L_IN1, dir > 0 ? HIGH : LOW);
  digitalWrite(PIN_L_IN2, dir < 0 ? HIGH : LOW);
  analogWrite(PIN_L_EN, dir == 0 ? 0 : speed);
}

static void rightDrive(int8_t dir, uint8_t speed) {
  digitalWrite(PIN_R_IN1, dir > 0 ? HIGH : LOW);
  digitalWrite(PIN_R_IN2, dir < 0 ? HIGH : LOW);
  analogWrite(PIN_R_EN, dir == 0 ? 0 : speed);
}

// Public API -------------------------------------------------------
void motorSetup() {
  const uint8_t pins[] = {PIN_L_EN, PIN_L_IN1, PIN_L_IN2,
                          PIN_R_EN, PIN_R_IN1, PIN_R_IN2};
  for (uint8_t i = 0; i < sizeof(pins); i++) pinMode(pins[i], OUTPUT);
  driveStop();
}

void driveForward(uint8_t speed) {
  leftDrive(1, speed);
  rightDrive(1, speed);
}

void driveBackward(uint8_t speed) {
  leftDrive(-1, speed);
  rightDrive(-1, speed);
}

void driveTurnLeft(uint8_t speed) {
  leftDrive(-1, speed);
  rightDrive(1, speed);
}

void driveTurnRight(uint8_t speed) {
  leftDrive(1, speed);
  rightDrive(-1, speed);
}

void driveStop() {
  leftDrive(0, 0);
  rightDrive(0, 0);
}

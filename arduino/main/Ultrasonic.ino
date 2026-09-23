/*
 * RoboSort - HC-SR04 rangefinders
 *
 * Four sensors: front, left, right, rear. Indices match PIN_TRIG /
 * PIN_ECHO in main.ino.
 *
 * pulseIn blocks, so a full sweep of four sensors costs up to
 * 4 x ECHO_TIMEOUT_US. The timeout is deliberately tight: at 25 ms the
 * loop would stall for a tenth of a second and the rover would coast
 * well past anything it was about to hit.
 */

const uint8_t IDX_FRONT = 0;
const uint8_t IDX_LEFT  = 1;
const uint8_t IDX_RIGHT = 2;
const uint8_t IDX_REAR  = 3;

const uint32_t ECHO_TIMEOUT_US = 12000;   // ~2 m of round trip
const uint16_t MAX_RANGE_CM    = 200;
const uint8_t  SETTLE_US       = 60;      // between sensors, avoids crosstalk

void ultrasonicSetup() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(PIN_TRIG[i], OUTPUT);
    pinMode(PIN_ECHO[i], INPUT);
    digitalWrite(PIN_TRIG[i], LOW);
  }
  delay(50);
}

/*
 * Distance in cm, or MAX_RANGE_CM when nothing echoes back. Out of range
 * is reported as far, never as 0 - a 0 from a timeout would read as an
 * imminent collision and lock the rover up.
 */
uint16_t readDistance(uint8_t index) {
  digitalWrite(PIN_TRIG[index], LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG[index], LOW);

  uint32_t duration = pulseIn(PIN_ECHO[index], HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return MAX_RANGE_CM;

  uint16_t cm = duration / 58;            // 343 m/s, there and back
  if (cm == 0 || cm > MAX_RANGE_CM) return MAX_RANGE_CM;
  return cm;
}

void readAllDistances() {
  distFront = readDistance(IDX_FRONT);
  delayMicroseconds(SETTLE_US);
  distLeft  = readDistance(IDX_LEFT);
  delayMicroseconds(SETTLE_US);
  distRight = readDistance(IDX_RIGHT);
  delayMicroseconds(SETTLE_US);
  distRear  = readDistance(IDX_REAR);
}

/*
 * RoboSort - HC-SR04 rangefinders
 *
 * Two sensors, left and right. Indices match PIN_TRIG / PIN_ECHO in
 * main.ino.
 *
 * Neither one looks straight ahead, so nothing here sees an obstacle dead
 * in front of the rover. Angling both 30-45 deg toward the front covers
 * most of it; the avoidance in main.ino works either way, because all it
 * asks is which side is blocked.
 *
 * pulseIn blocks, so a sweep costs up to 2 x ECHO_TIMEOUT_US plus the gap
 * between sensors. The timeout is deliberately tight: a long one lets a
 * missing echo stall the loop while the rover keeps rolling.
 */

const uint8_t IDX_LEFT  = 0;
const uint8_t IDX_RIGHT = 1;

const uint32_t ECHO_TIMEOUT_US = 12000;   // ~2 m of round trip
const uint16_t MAX_RANGE_CM    = 200;
const uint8_t  SONAR_GAP_MS    = 10;      // between sensors, so one does not hear the other's ping

void ultrasonicSetup() {
  for (uint8_t i = 0; i < sizeof(PIN_TRIG); i++) {
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
  distLeft  = readDistance(IDX_LEFT);
  delay(SONAR_GAP_MS);
  distRight = readDistance(IDX_RIGHT);
}

// Start-up report, one reading per sensor. MAX_RANGE_CM means no echo came
// back: nothing within ~2 m, or the sensor is not connected.
void printSensorCheck() {
  readAllDistances();
  Serial.print(F("INIT sensors - left "));
  Serial.print(distLeft);
  Serial.print(F(" cm, right "));
  Serial.print(distRight);
  Serial.print(F(" cm"));
  if (distLeft == MAX_RANGE_CM || distRight == MAX_RANGE_CM) {
    Serial.print(F(" ("));
    Serial.print(MAX_RANGE_CM);
    Serial.print(F(" = no echo: nothing within 2 m, or not connected)"));
  }
  Serial.println();
}

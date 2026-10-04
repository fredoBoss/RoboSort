/*
 * RoboSort - arm servos, camera pan, conveyor, and the sorting gate
 *
 * Four MG996R servos, 180 degree:
 *     lift L + lift R  one pair sharing the arm pivot, mounted alike
 *     stretch          extends / retracts the arm toward the trash
 *     sort             the gate that sends an item to the bio or non-bio bin
 * and one micro servo (SG90 class) on the mast:
 *     pan              turns the webcam 45 deg left or right of straight ahead
 *
 * Every servo resets at boot, after each pickup, and on the H command:
 * to SERVO_HOME (90 deg), except the stretch, which rests retracted at
 * STRETCH_IN.
 *
 * Plus the conveyor DC motor (on/off through a relay), which lives here because it is part of the
 * same collect-and-sort sequence rather than part of driving.
 *
 * POWER: an MG996R stalls at roughly 2.5 A. Four of them need their own
 * 5-6 V supply with its ground tied to the Mega's. Feeding them from the
 * Mega's 5V pin browns out the regulator and resets the board mid-pickup.
 *
 * Servo.h itself is included from main.ino; this tab is named for the
 * hardware it owns, not the library.
 */

Servo servoLiftL;
Servo servoLiftR;
Servo servoStretch;
Servo servoSort;
Servo servoPan;

/*
 * MG996R covers its full 180 deg over roughly 500-2500 us. The library's
 * default range is 544-2400, which clips both ends, so the range is given
 * to attach() explicitly - without it write(0) and write(180) fall short
 * of the real travel.
 */
const uint16_t SERVO_MIN_US = 500;
const uint16_t SERVO_MAX_US = 2500;

// ------------------------------------------------------------- angles
/*
 * The one reset angle, mid-travel on a 180 deg servo. It is also where the
 * library parks a channel on attach(), so nothing snaps to an unknown
 * position at boot, and it is the fixed point of the lift mirroring below
 * - 180 - 90 is 90, so the pair agrees on home whichever way it is wired.
 */
const uint8_t SERVO_HOME = 90;

// Lower angle is up on this linkage - 150 was measured driving the arm down.
// const uint8_t LIFT_UP     = 30;    // arm raised, clear of the ground
const uint8_t LIFT_UP     = 60; 
const uint8_t LIFT_DOWN   = 120;   // arm down on the item
// Higher angle is in on this linkage - 160 was seen pulling the arm back
// and 90 holding it out, so the stretch rests at STRETCH_IN, not at 90.
const uint8_t STRETCH_OUT = 90;    // reaching out for the item
const uint8_t STRETCH_IN  = 160;   // retracted over the belt - the rest position
const uint8_t SORT_BIO    = 40;
const uint8_t SORT_NONBIO = 140;

/*
 * Camera pan: home is straight ahead, the only direction the arm can
 * reach, and it turns 45 deg either way from there. On most servos seen
 * from the horn side, a larger angle turns anticlockwise - to the left.
 * If LOOKLEFT turns the camera right, swap PAN_LEFT and PAN_RIGHT.
 */
const uint8_t PAN_HOME  = SERVO_HOME;   // 90, looking straight ahead
const uint8_t PAN_LEFT  = 135;          // home + 45
const uint8_t PAN_RIGHT = 45;           // home - 45

/*
 * Both lift servos are mounted the same way round on the pivot, so they
 * take the same angle - no mirroring. If one is ever remounted facing the
 * other way it must be fed 180 - angle instead, or the pair fights itself
 * and two stalled MG996Rs pull ~5 A between them and strip their gears.
 *
 * Two servos are never centred identically, though. If they strain against
 * each other while holding still, trim the right one here: the offset is
 * added to its angle only, so a few degrees either way squares the pair up
 * without touching the angles the sequence asks for.
 */
const int8_t LIFT_R_TRIM = 0;   // degrees, applied to the right lift servo

/*
 * MG996R is about 0.17 s per 60 deg at 4.8 V, so the widest move here
 * (lift 30 -> 150) takes a little over 0.3 s unloaded. 600 ms leaves room
 * for the arm's own weight; drop it once the real linkage is on and timed.
 */
const uint16_t SERVO_SETTLE_MS = 600;

/*
 * The lift pair is ramped instead of jumped. Writing 30 -> 150 in one go
 * makes both MG996Rs slam at full speed together, and the current spike
 * sags the supply far enough to hang the Mega or its USB-serial chip - the
 * board goes silent with no reset banner. 2 deg every 15 ms is ~130 deg/s,
 * slower than the servo's own top speed, so it tracks the ramp and never
 * stalls; the full 120 deg swing takes about 0.9 s.
 */
const uint8_t  LIFT_STEP_DEG  = 2;
const uint8_t  LIFT_STEP_MS   = 15;
const uint16_t LIFT_HOLD_MS   = 150;    // after the ramp, let the arm stop swinging

// Last angle sent to the lift pair. attach() parks it at 90, so it starts there.
uint8_t liftAngle = SERVO_HOME;

/*
 * An SG90 is about 0.1 s per 60 deg, so the widest pan move (left to
 * right, 90 deg) takes ~0.15 s. 300 ms adds time for the camera on the
 * mast to stop wobbling before the next frame is worth classifying.
 */
const uint16_t PAN_SETTLE_MS = 300;
uint8_t panAngle = PAN_HOME;          // last angle sent to the pan servo
const uint16_t CONVEYOR_RUN_MS = 5000;  // belt time from the arm to the bin

void servoSetup() {
  // attach() starts a channel at 90 unless it was given a pulse first. 90
  // is reaching out on the stretch, so its rest pulse is loaded before
  // attach() - otherwise the arm swings out and back at every power-up.
  servoStretch.writeMicroseconds(map(STRETCH_IN, 0, 180, SERVO_MIN_US, SERVO_MAX_US));

  servoLiftL.attach(PIN_SERVO_LIFT_L,   SERVO_MIN_US, SERVO_MAX_US);
  servoLiftR.attach(PIN_SERVO_LIFT_R,   SERVO_MIN_US, SERVO_MAX_US);
  servoStretch.attach(PIN_SERVO_STRETCH, SERVO_MIN_US, SERVO_MAX_US);
  servoSort.attach(PIN_SERVO_SORT,       SERVO_MIN_US, SERVO_MAX_US);
  servoPan.attach(PIN_SERVO_PAN,         SERVO_MIN_US, SERVO_MAX_US);

  // Relay pin written to "off" before it becomes an output, so the belt
  // cannot twitch on for an instant at boot.
  conveyorStop();
  pinMode(PIN_CONV_RELAY, OUTPUT);

  servoResetAll();
}

/*
 * Drive the lift pair from one logical angle. Everything else in this file
 * talks to the joint through here, so the pairing lives in exactly one
 * place. The trim is clamped to the servo's 0-180 range so a large offset
 * cannot wrap a uint8_t and fling the arm to the far end.
 */
void liftWrite(uint8_t angle) {
  servoLiftL.write(angle);
  servoLiftR.write(constrain((int)angle + LIFT_R_TRIM, 0, 180));
  liftAngle = angle;
}

// Walk the lift pair to target a few degrees at a time, then hold briefly.
// Blocks until the arm is there, the same contract as the settle delays.
void liftMoveTo(uint8_t target) {
  while (liftAngle != target) {
    uint8_t next;
    if (liftAngle < target) {
      next = (target - liftAngle > LIFT_STEP_DEG) ? liftAngle + LIFT_STEP_DEG : target;
    } else {
      next = (liftAngle - target > LIFT_STEP_DEG) ? liftAngle - LIFT_STEP_DEG : target;
    }
    liftWrite(next);
    delay(LIFT_STEP_MS);
  }
  delay(LIFT_HOLD_MS);
}

// Park every servo, gate and camera included, at its rest angle: 90 for
// all but the stretch, which is pulled in.
void servoResetAll() {
  servoStretch.write(STRETCH_IN);
  servoSort.write(SERVO_HOME);
  servoPan.write(PAN_HOME);
  panAngle = PAN_HOME;
  liftMoveTo(SERVO_HOME);
  delay(SERVO_SETTLE_MS);
}

// Arm only - the gate is sequenced separately by sortTo().
void armHome() {
  servoStretch.write(STRETCH_IN);
  liftMoveTo(SERVO_HOME);
  delay(SERVO_SETTLE_MS);
}

/*
 * Reach out, drop onto the item, scoop it back in, then belt it to the
 * bin area and return the arm to rest. Blocking on purpose - the rover is
 * stationary for this and nothing should interleave with a half-extended
 * arm.
 */
void runPickupSequence() {
  pickupBusy = true;

  servoStretch.write(STRETCH_OUT);
  delay(SERVO_SETTLE_MS);

  liftMoveTo(LIFT_DOWN);

  liftMoveTo(LIFT_UP);              // scoop

  servoStretch.write(STRETCH_IN);   // bring it over the belt
  delay(SERVO_SETTLE_MS);

  conveyorPulse();

  armHome();                       // lift back to 90, stretch pulled in
  pickupBusy = false;
}

// Set the gate from a protocol byte. Anything other than B/N leaves it at
// the reset angle rather than picking a bin at random.
void sortTo(char command) {
  if (command == CMD_BIO) {
    servoSort.write(SORT_BIO);
  } else if (command == CMD_NONBIO) {
    servoSort.write(SORT_NONBIO);
  } else {
    servoSort.write(SERVO_HOME);
  }
  delay(SERVO_SETTLE_MS);

  servoSort.write(SERVO_HOME);      // let the item fall, then back to reset
  delay(SERVO_SETTLE_MS);
}

// --------------------------------------------------- single movements
/*
 * One move each, for the manual serial tests in main.ino. Every one of
 * them blocks for the settle time, so the caller knows the joint has
 * arrived when the call returns - the same contract runPickupSequence()
 * relies on.
 *
 * main.ino cannot reach servoStretch or the lift pair itself: the tabs are
 * concatenated with main.ino first, so these objects do not exist yet at
 * that point in the file. Prototypes are generated for functions, though,
 * which is why the moves are exposed as calls.
 */
void armLiftUp() {
  liftMoveTo(LIFT_UP);
}

void armLiftDown() {
  liftMoveTo(LIFT_DOWN);
}

void armStretchOut() {
  servoStretch.write(STRETCH_OUT);
  delay(SERVO_SETTLE_MS);
}

void armRetract() {
  servoStretch.write(STRETCH_IN);
  delay(SERVO_SETTLE_MS);
}

// ------------------------------------------------------- camera pan
// Same contract: each returns once the camera has stopped moving. A move
// to where the camera already points returns at once.
void panTo(uint8_t angle) {
  if (angle == panAngle) return;
  servoPan.write(angle);
  panAngle = angle;
  delay(PAN_SETTLE_MS);
}

void cameraLookLeft()  { panTo(PAN_LEFT);  }
void cameraLookRight() { panTo(PAN_RIGHT); }
void cameraLookAhead() { panTo(PAN_HOME);  }

// For the STATUS line - main.ino cannot read panAngle itself.
uint8_t cameraAngle() {
  return panAngle;
}

/*
 * Bench check for the lift pair, called by hand from setup() - never from
 * loop(). Both horns should travel together and sit level at 90. If they
 * strain against each other while stopped, cut power and adjust
 * LIFT_R_TRIM; if one travels the wrong way, it is mounted backwards.
 */
void servoSweepTest() {
  liftMoveTo(SERVO_HOME);
  liftMoveTo(LIFT_UP);
  liftMoveTo(SERVO_HOME);
  liftMoveTo(LIFT_DOWN);
  liftMoveTo(SERVO_HOME);
}

// --------------------------------------------------------- conveyor
// On/off only - a relay cannot set a speed.
void conveyorRun() {
  digitalWrite(PIN_CONV_RELAY, CONV_RELAY_ACTIVE_LOW ? LOW : HIGH);
}

void conveyorStop() {
  digitalWrite(PIN_CONV_RELAY, CONV_RELAY_ACTIVE_LOW ? HIGH : LOW);
}

// Reads back the level the relay pin is driven to.
bool conveyorIsOn() {
  return digitalRead(PIN_CONV_RELAY) == (CONV_RELAY_ACTIVE_LOW ? LOW : HIGH);
}

// Run the belt long enough to carry one item to the bin, then stop.
void conveyorPulse() {
  conveyorRun();
  delay(CONVEYOR_RUN_MS);
  conveyorStop();
}

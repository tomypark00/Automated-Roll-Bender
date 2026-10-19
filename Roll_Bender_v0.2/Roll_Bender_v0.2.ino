#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>


// --- Pin Definitions ---
// Encoder
const int ENCODER_PIN_A = 2;  // Phase A (White Wire)
const int ENCODER_PIN_B = 3;  // Phase B (Green Wire)

// Limit switches
const int REVERSE_LIMIT_PIN = 5;  // Home / Zero reference
const int FORWARD_LIMIT_PIN = 10; // Far-end safety limit

// Motor (Cytron MDD10A)
const int MOTOR_PWM = 8;
const int MOTOR_DIR = 7;

// Onboard LED
const int LED_ARD = 13;

// Operator interface (OLED + KY-040 rotary encoder only)
// OLED on software (bit-banged) I2C: SDA -> 28, SCK/SCL -> 36
const int OLED_SDA_PIN = 28;
const int OLED_SCL_PIN = 36;
// KY-040 wiring: CLK -> 29, DT -> 37, SW -> 45, + -> 5V, GND -> GND
// These pins have no external or pin-change interrupts, so the knob is sampled by a
// 1 kHz timer interrupt instead (see knobPoll / TIMER0_COMPA_vect). Arduino Mega (AVR) only.
const int KNOB_CLK_PIN = 29;   // KY-040 CLK
const int KNOB_DT_PIN = 37;    // KY-040 DT
const int BTN_ENTER_PIN = 45;  // KY-040 SW (knob push): select / confirm / stop / back
const unsigned long LONG_PRESS_MS = 700; // Hold the knob this long to go back / cancel
const int KNOB_STEPS_PER_DETENT = 4; // Quadrature edges per click; use 2 if one click skips a step
const int KNOB_DIRECTION = 1;        // Set to -1 if turning clockwise scrolls the wrong way
const unsigned long BUTTON_DEBOUNCE_MS = 30;

// Most 1.3" 128x64 I2C OLEDs use the SH1106 controller (0.96" ones use SSD1306).
// If the image is shifted ~2 px or has a garbage column, swap to the SSD1306 line.
// SW_I2C = any two pins, but slower than the hardware I2C pins (Mega: SDA 20 / SCL 21).
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL_PIN, /* data=*/ OLED_SDA_PIN, /* reset=*/ U8X8_PIN_NONE);
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL_PIN, /* data=*/ OLED_SDA_PIN, /* reset=*/ U8X8_PIN_NONE);

// --- Calibration & Limits Constants ---
const float COUNTS_PER_DEGREE = 6.6; // Tuned empirically

// Software Limits (Virtual boundaries before hitting physical switches)
const float SOFT_MIN_ANGLE = -1.0;   // Just below home (0°)
const float SOFT_MAX_ANGLE = 131;    // Adjust based on your maximum safe range

// Motion Settings
const int HOMING_SPEED = 50;
const int POSITION_SPEED = 130;
const int DEAD_BAND = 4;          // Allowable error in counts (hold phase)
// Motor ramp, in PWM counts per loop pass (a pass is roughly 20-25 ms)
const int ACCEL_STEP = 5;         // Speeding up
const int DECEL_STEP = 12;        // Slowing down, and crossing zero before a direction change
const int KICK_STEP = 30;         // Hop across the dead zone below BEND_MIN_PWM when a forward bend starts
const int HOLD_TORQUE_PWM = 60;   // Light forward torque kept on at a position instead of dropping to 0
const long IN_POSITION_WINDOW = 3; // Counts: "target reached" window

// Approach speeds
// Below BEND_MIN_PWM the motor cannot bend the metal (forward = loaded direction),
// so every FORWARD command is kept at or above it. Reverse is unloaded and may run lower.
const int BEND_MIN_PWM = 90;
const int APPROACH_MIN_PWM = 115;  // Forward torque floor while bending (must be >= BEND_MIN_PWM)
const int REVERSE_CORRECT_MIN_PWM = 60; // Gentle floor for reverse corrections (overshoot recovery)
// Position command (P<angle>): no sheet metal loaded, so it can run at low PWM in both directions
const int FREE_MOVE_PWM = 60;      // Cruise PWM
const int FREE_MOVE_MIN_PWM = 45;  // Slowest PWM near the target (keep above REVERSE_STALL_PWM_THRESHOLD)
const long APPROACH_RAMP_COUNTS = 60;   // Start slowing down inside this many counts (longer = gentler arrival)

// --- Variables ---
volatile long encoderpos = 0;

// --- Limit switch state (sampled at 1 kHz in the Timer0 interrupt, see limitPoll) ---
// Sampling in an interrupt means a switch that is only actuated briefly can't slip by
// while the main loop is busy (e.g. redrawing the OLED).
const uint8_t LIMIT_DEBOUNCE_SAMPLES = 3;   // Consecutive 1 ms samples required (3 ms debounce)
volatile bool revLimitActive = false;       // Debounced: switch currently pressed
volatile bool fwdLimitActive = false;
volatile bool revLimitLatched = false;      // Set on each new press; cleared when the loop reads it
volatile bool fwdLimitLatched = false;
volatile long revLimitHitPos = 0;           // Encoder position at the moment the reverse switch tripped
long lastReportedPosition = 0;

// Backlash Correction & Dwell Variables
// positionPasses counts correction passes on the final leg: every failed dwell re-check
// and every direction reversal (overshoot) counts as one pass.
int positionPasses = 0;
const int MAX_POSITION_PASSES = 5;
unsigned long dwellTimer = 0;
bool isDwelling = false;
const unsigned long DWELL_TIME_MS = 200; // Dwell at target before re-checking position and locking

// --- Stall Detection Variables ---
unsigned long lastStallCheckTime = 0;
long lastStallEncoderPos = 0;
const unsigned long STALL_CHECK_INTERVAL = 300; // Check every 300ms
const long MIN_EXPECTED_COUNTS = 2; // Minimum counts expected to move in that window
const int STALL_TRIP_COUNT = 3;     // Number of consecutive checks before triggering stall
// Stall is only evaluated above these PWM levels. Forward needs BEND_MIN_PWM to move the
// metal at all; reverse is unloaded, so even low PWM should still move the encoder.
const int FORWARD_STALL_PWM_THRESHOLD = BEND_MIN_PWM;
const int REVERSE_STALL_PWM_THRESHOLD = 40;
int stallCounter = 0;

// Assign names using enum integer constants
enum MotorDIR { STOP, FORWARD, REVERSE };
MotorDIR currentDIR = STOP;
MotorDIR lastCorrectionDir = STOP; // Used to detect direction reversals (overshoot)

// --- Two-Stage Bending Variables ---
enum MoveStage { STAGE_IDLE, STAGE_FIRST_LEG, STAGE_PAUSE, STAGE_FINAL_LEG };
MoveStage currentMoveStage = STAGE_IDLE;
float finalTargetAngle = 0.0;
unsigned long stagePauseTimer = 0;
const unsigned long STAGE_PAUSE_DURATION_MS = 500; // Pause duration between 75% and final (0.5s)

// Control States
bool isHoming = false;
// A/B bend commands home first; the bend is queued here and starts when homing completes
bool pendingBend = false;
// P<angle>: home first, then free (unloaded) move to the angle
bool pendingMove = false;
float pendingMoveAngle = 0.0;
bool freeMoveActive = false;
int freeMovePasses = 0;
MotorDIR freeMoveLastDir = STOP;
float pendingBendAngle = 0.0;
const char* pendingBendLabel = "target";
bool positionControlActive = false;
bool clampsEngaged = false; // Manual operator clamp confirmation (Yes/No)
long targetEncoderPos = 0;
float targetAngle = 0.0;

// Saved Setpoint Variable
float savedSetpoint = 0.0;

// Ramping Variables
int currentPWM = 0;
MotorDIR outputDIR = STOP; // Direction actually being driven (lags currentDIR while ramping/reversing)
int targetPWM = 0;

// Holding Bend Variables
const int HOLDING_PWM = 100;                // Max forward PWM while holding (must be >= BEND_MIN_PWM)
const unsigned long HOLD_DURATION_MS = 500; // Hold duration in milliseconds
bool isHolding = false;
unsigned long holdTimer = 0;

// Direction logic constants (swap HIGH/LOW if motor direction is reversed)
const uint8_t DIR_FORWARD = HIGH;
const uint8_t DIR_REVERSE = LOW;

// Short status message shown on the OLED (mirrors key serial events)
char uiMessage[22] = "";
unsigned long uiMessageUntil = 0;
const unsigned long UI_MESSAGE_MS = 3000;

void showMsg(const char* msg) {
  strncpy(uiMessage, msg, sizeof(uiMessage) - 1);
  uiMessage[sizeof(uiMessage) - 1] = '\0';
  uiMessageUntil = millis() + UI_MESSAGE_MS;
}
bool msgActive() { return millis() < uiMessageUntil; }

// --- Helper Functions ---
const char* getDirString(MotorDIR dir) {
  switch (dir) {
    case FORWARD: return "FORWARD";
    case REVERSE: return "REVERSE";
    default:      return "STOP";
  }
}

// --- Encoder Interrupt Service Routines (Flipped Sign) ---
void readEncoderA() {
  if (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B)) {
    encoderpos--;
  } else {
    encoderpos++;
  }
}

void readEncoderB() {
  if (digitalRead(ENCODER_PIN_A) != digitalRead(ENCODER_PIN_B)) {
    encoderpos--;
  } else {
    encoderpos++;
  }
}

// --- Motor Control & Ramping Update ---
void requestMotor(MotorDIR dir, int desiredPWM) {
  currentDIR = dir;
  targetPWM = constrain(desiredPWM, 0, 255);
  // Forward is the bending direction: anything under BEND_MIN_PWM would just stall.
  if (dir == FORWARD && targetPWM > 0 && targetPWM < BEND_MIN_PWM) {
    targetPWM = BEND_MIN_PWM;
  }
  if (dir == STOP) {
    targetPWM = 0;
  }
}

// Light forward holding torque. Deliberately bypasses the BEND_MIN_PWM floor: this is for
// holding a position, not for bending.
void requestHoldTorque(int pwm) {
  currentDIR = FORWARD;
  targetPWM = constrain(pwm, 0, 255);
}

// Unloaded move: low PWM is fine in either direction, so the BEND_MIN_PWM floor is bypassed
void requestFreeMove(MotorDIR dir, int pwm) {
  currentDIR = dir;
  targetPWM = constrain(pwm, 0, 255);
}

bool limitBlocks(MotorDIR d) {
  return (d == REVERSE && revLimitActive) || (d == FORWARD && fwdLimitActive);
}

// Call this every loop iteration. Output ramps up and down smoothly, and never changes
// direction (or drops straight to zero) at speed: it slows to zero first, then flips.
// Emergency stops (stopMotor / haltOutput) still cut instantly.
void updateMotorRamp() {
  MotorDIR wantDIR = currentDIR;
  int wantPWM = (wantDIR == STOP) ? 0 : targetPWM;

  // Backstop: never drive into a pressed limit switch
  if (limitBlocks(outputDIR)) {
    currentPWM = 0;
    outputDIR = STOP;
    analogWrite(MOTOR_PWM, 0);
    return;
  }
  if (limitBlocks(wantDIR)) {
    wantDIR = STOP; // Refuse to start or flip toward a pressed switch
    wantPWM = 0;
  }

  if (wantDIR == STOP || (outputDIR != STOP && wantDIR != outputDIR)) {
    // Soft stop, or reversing: slow to zero in the current direction first
    currentPWM = max(0, currentPWM - DECEL_STEP);
    if (currentPWM == 0) outputDIR = wantDIR; // Flip at zero (STOP stays STOP)
  } else {
    if (outputDIR == STOP) outputDIR = wantDIR;

    if (wantDIR == FORWARD && wantPWM >= BEND_MIN_PWM && currentPWM < BEND_MIN_PWM) {
      // Below BEND_MIN_PWM a forward bend can't move the metal, so hop across that zone
      currentPWM = min(BEND_MIN_PWM, currentPWM + KICK_STEP);
    } else if (currentPWM < wantPWM) {
      currentPWM = min(wantPWM, currentPWM + ACCEL_STEP);
    } else if (currentPWM > wantPWM) {
      currentPWM = max(wantPWM, currentPWM - DECEL_STEP);
    }
  }

  // Apply direction and ramped PWM to hardware driver
  if (outputDIR == FORWARD) {
    digitalWrite(MOTOR_DIR, DIR_FORWARD);
    analogWrite(MOTOR_PWM, currentPWM);
  } else if (outputDIR == REVERSE) {
    digitalWrite(MOTOR_DIR, DIR_REVERSE);
    analogWrite(MOTOR_PWM, currentPWM);
  } else {
    analogWrite(MOTOR_PWM, 0);
  }
}

// Immediately cut motor output without clearing any control state
void haltOutput() {
  requestMotor(STOP, 0);
  currentPWM = 0;
  outputDIR = STOP;
  analogWrite(MOTOR_PWM, 0);
}

// Holding torque around a target. error = target - position (counts).
// Springback (error > 0) gets a proportional forward push; in the deadband the motor keeps a
// light constant torque instead of switching on and off.
void holdTorque(long error) {
  if (error > DEAD_BAND) {
    requestMotor(FORWARD, (int)constrain(error * 15, BEND_MIN_PWM, HOLDING_PWM));
  } else if (error < -DEAD_BAND) {
    requestMotor(REVERSE, 30); // Drifted past the target: nudge back gently
  } else {
    requestHoldTorque(HOLD_TORQUE_PWM);
  }
}

void stopMotor() {
  haltOutput();
  isHoming = false;
  pendingBend = false; // Any stop/abort cancels a queued bend
  pendingMove = false;
  freeMoveActive = false;
  positionControlActive = false;
  isHolding = false;
  isDwelling = false;
  currentMoveStage = STAGE_IDLE;
}

// Controlled finish: same state reset as stopMotor(), but PWM ramps down instead of cutting
void softStopAll() {
  requestMotor(STOP, 0);
  isHoming = false;
  pendingBend = false;
  pendingMove = false;
  freeMoveActive = false;
  positionControlActive = false;
  isHolding = false;
  isDwelling = false;
  currentMoveStage = STAGE_IDLE;
}

// Transition from positioning into the springback holding phase
void beginHold() {
  positionControlActive = false;
  currentMoveStage = STAGE_IDLE;
  isDwelling = false;
  isHolding = true;
  holdTimer = millis();
}

// Called when the final leg has used up its correction passes.
// Accepts the position if it is reasonably close, otherwise aborts.
void settleOrAbort(long absError) {
  if (absError <= 2 * DEAD_BAND) {
    showMsg("ACCEPTED NEAR TARGET"); Serial.print("WARNING: Max correction passes reached; accepting position (error ");
    Serial.print(absError);
    Serial.println(" counts).");
    beginHold();
  } else {
    stopMotor();
    showMsg("CANT SETTLE-STOPPED"); Serial.print("ERROR: Could not settle on target after max passes (error ");
    Serial.print(absError);
    Serial.println(" counts). Motion stopped.");
  }
}

// Start a two-stage (75% then 100%) move to the given absolute angle
void startStagedMove(float angle, const char* label) {
  finalTargetAngle = angle;
  targetAngle = finalTargetAngle * 0.75f; // Intermediate target at 75%
  targetEncoderPos = (long)(targetAngle * COUNTS_PER_DEGREE);
  currentMoveStage = STAGE_FIRST_LEG;
  positionControlActive = true;
  isDwelling = false;
  positionPasses = 0;
  lastCorrectionDir = STOP;
  isHolding = false;

  Serial.print("Staged Bend: Moving to ");
  Serial.print(label);
  Serial.print(" 75% intermediate angle: ");
  Serial.print(targetAngle);
  Serial.print("° (encoder target: ");
  Serial.print(targetEncoderPos);
  Serial.print(" counts, Final Target: ");
  Serial.print(finalTargetAngle);
  Serial.println("°)");
}

// Begin the unloaded move itself (homing already done)
void startFreeMove(float angle) {
  finalTargetAngle = angle;
  targetAngle = angle;
  targetEncoderPos = (long)(angle * COUNTS_PER_DEGREE);
  positionControlActive = false;
  isHolding = false;
  isDwelling = false;
  currentMoveStage = STAGE_IDLE;
  freeMovePasses = 0;
  freeMoveLastDir = STOP;
  freeMoveActive = true;
  Serial.print("Moving to position: ");
  Serial.print(angle);
  Serial.print("° (encoder target: ");
  Serial.print(targetEncoderPos);
  Serial.println(" counts)");
}

// Bend routine entry point: home first (the arm may have been moved while clamping),
// then run the staged bend automatically once the home switch zeroes the encoder.
void startBendWithHoming(float angle, const char* label) {
  positionControlActive = false;
  isHolding = false;
  isDwelling = false;
  currentMoveStage = STAGE_IDLE;
  pendingBendAngle = angle;
  pendingBendLabel = label;
  freeMoveActive = false;
  pendingMove = false;
  isHoming = true;
  pendingBend = true; // Set after clearing state above; stopMotor() would reset it
  Serial.print("Bend requested (");
  Serial.print(angle);
  Serial.println("°). Homing first...");
}

// Position command entry point: home first, then move (unloaded, low PWM) to the angle
void startMoveWithHoming(float angle) {
  positionControlActive = false;
  isHolding = false;
  isDwelling = false;
  freeMoveActive = false;
  currentMoveStage = STAGE_IDLE;
  pendingMoveAngle = angle;
  isHoming = true;
  pendingBend = false;
  pendingMove = true;
  Serial.print("Position requested (");
  Serial.print(angle);
  Serial.println("°). Homing first...");
}

// Plain homing (no queued bend)
void startHoming() {
  Serial.println("Starting Homing Routine...");
  pendingBend = false;
  pendingMove = false;
  freeMoveActive = false;
  isHoming = true;
  positionControlActive = false;
  isHolding = false;
  isDwelling = false;
  currentMoveStage = STAGE_IDLE;
}

// Operator clamp confirmation (shared by serial Y/N and the OLED menu)
void setClamps(bool on) {
  clampsEngaged = on;
  if (on) {
    showMsg("CLAMPS ON");
    Serial.println("CLAMP STATUS: YES (All clamps engaged. Motion permitted.)");
  } else {
    stopMotor();
    showMsg("CLAMPS OFF");
    Serial.println("CLAMP STATUS: NO (Clamps disengaged. Motion locked.)");
  }
}

// --- Serial Command Processing ---
// Safe to call at any time (including during homing). While homing, only
// S (abort), N (clamps off) and Y are honored; other motion commands are rejected.
void processSerial(bool isRevLimitHit, bool isFwdLimitHit, float currentAngle) {
  if (Serial.available() <= 0) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  // Multi-character / specific commands first
  if (input.equalsIgnoreCase("SV")) {
    savedSetpoint = finalTargetAngle; // Final target, not the 75% intermediate
    Serial.print("SUCCESS: Current target angle (");
    Serial.print(savedSetpoint);
    Serial.println("°) saved as Saved Setpoint.");
    return;
  }

  if (input.startsWith("+")) {
    float delta = input.substring(1).toFloat();
    if (delta == 0.0) delta = 0.5; // Default step if just '+' sent

    float newSetpoint = savedSetpoint + delta;
    if (newSetpoint > SOFT_MAX_ANGLE) {
      Serial.println("ERROR: Adjusted setpoint exceeds software max limit!");
    } else {
      savedSetpoint = newSetpoint;
      Serial.print("Updated Saved Setpoint: ");
      Serial.print(savedSetpoint, 1);
      Serial.println("°");
    }
    return;
  }

  if (input.startsWith("-")) {
    float delta = input.substring(1).toFloat();
    if (delta == 0.0) delta = 0.5; // Default step if just '-' sent

    float newSetpoint = savedSetpoint - delta;
    if (newSetpoint < SOFT_MIN_ANGLE) {
      Serial.println("ERROR: Adjusted setpoint exceeds software min limit!");
    } else {
      savedSetpoint = newSetpoint;
      Serial.print("Updated Saved Setpoint: ");
      Serial.print(savedSetpoint, 1);
      Serial.println("°");
    }
    return;
  }

  // Single character / prefixed commands
  char cmd = toupper(input.charAt(0));

  if (cmd == 'S') {
    showMsg("STOPPED"); Serial.println("Stopping Motor.");
    stopMotor();
    return;
  }
  if (cmd == 'Y') {
    setClamps(true);
    return;
  }
  if (cmd == 'N') {
    setClamps(false);
    return;
  }

  // Everything below starts new motion
  if (!clampsEngaged) {
    showMsg("SET CLAMPS FIRST"); Serial.println("ERROR: Cannot start motion - Please send 'Y' to verify clamps are engaged first!");
    return;
  }
  if (isHoming) {
    showMsg("HOMING IN PROGRESS"); Serial.println("ERROR: Homing in progress. Send 'S' to abort first.");
    return;
  }

  if (cmd == 'H') {
    startHoming();
  }
  else if (cmd == 'F') {
    if (isFwdLimitHit || currentAngle >= SOFT_MAX_ANGLE) {
      Serial.println("ERROR: Cannot move Forward - limit or max software bound reached!");
    } else {
      positionControlActive = false;
      isHolding = false;
      isDwelling = false;
      currentMoveStage = STAGE_IDLE;
      Serial.println("Manual Moving FORWARD...");
      requestMotor(FORWARD, POSITION_SPEED);
    }
  }
  else if (cmd == 'R') {
    if (isRevLimitHit || currentAngle <= SOFT_MIN_ANGLE) {
      Serial.println("ERROR: Cannot move Reverse - limit or min software bound reached!");
    } else {
      positionControlActive = false;
      isHolding = false;
      isDwelling = false;
      currentMoveStage = STAGE_IDLE;
      Serial.println("Manual Moving REVERSE...");
      requestMotor(REVERSE, POSITION_SPEED);
    }
  }
  else if (cmd == 'A') {
    float angleVal = input.substring(1).toFloat();
    if (angleVal < SOFT_MIN_ANGLE || angleVal > SOFT_MAX_ANGLE) {
      Serial.print("ERROR: Target angle ");
      Serial.print(angleVal);
      Serial.print("° is out of bounds! Software limits are [");
      Serial.print(SOFT_MIN_ANGLE);
      Serial.print("°, ");
      Serial.print(SOFT_MAX_ANGLE);
      Serial.println("°].");
    } else {
      startBendWithHoming(angleVal, "target");
    }
  }
  else if (cmd == 'P') {
    float angleVal = input.substring(1).toFloat();
    if (angleVal < SOFT_MIN_ANGLE || angleVal > SOFT_MAX_ANGLE) {
      Serial.print("ERROR: Target angle ");
      Serial.print(angleVal);
      Serial.print("° is out of bounds! Software limits are [");
      Serial.print(SOFT_MIN_ANGLE);
      Serial.print("°, ");
      Serial.print(SOFT_MAX_ANGLE);
      Serial.println("°].");
    } else {
      startMoveWithHoming(angleVal);
    }
  }
  else if (cmd == 'B') {
    if (savedSetpoint < SOFT_MIN_ANGLE || savedSetpoint > SOFT_MAX_ANGLE) {
      Serial.println("ERROR: Saved Setpoint is out of software bounds!");
    } else {
      startBendWithHoming(savedSetpoint, "Saved Setpoint");
    }
  }
}

// ===================== OLED + KY-040 Rotary Encoder + Buttons =====================
// Knob: turn to scroll the menu / adjust the angle, short push to select / confirm.
// Hold the knob (~0.7 s) to go back / cancel. While a move is running, ANY push stops it.
// Serial commands keep working alongside this.
enum UiState { UI_MENU, UI_SET_ANGLE, UI_CONFIRM };
enum AnglePurpose { PURPOSE_BEND, PURPOSE_SETPOINT };

UiState uiState = UI_MENU;
AnglePurpose anglePurpose = PURPOSE_BEND;

const int MENU_ITEMS = 5;
int menuIndex = 0;
float uiAngle = 0.0;            // Angle currently picked with the knob
float uiBendAngle = 0.0;        // Angle waiting for confirmation
const char* uiBendLabel = "target";

unsigned long lastDisplayMs = 0;
// Software I2C makes each OLED redraw block the loop for a long time, so redraw rarely
// and only when something on screen actually changed.
const unsigned long DISPLAY_INTERVAL_IDLE_MS = 100;
unsigned long lastDisplaySig = 0;

// --- Knob push switch: debounced, with short / long press detection ---
bool swRaw = HIGH;
bool swStable = HIGH;
bool swDown = false;
bool swConsumed = false;        // Press already acted on (stop / long press): ignore its release
unsigned long swChangedAt = 0;
unsigned long swDownAt = 0;

// pressEdge: switch just went down. shortPress: released before LONG_PRESS_MS.
// longPress: held for LONG_PRESS_MS (fires once, while still held).
void readKnobSwitch(bool &pressEdge, bool &shortPress, bool &longPress) {
  unsigned long now = millis();
  bool raw = digitalRead(BTN_ENTER_PIN);
  if (raw != swRaw) {
    swRaw = raw;
    swChangedAt = now;
  }
  if ((now - swChangedAt) >= BUTTON_DEBOUNCE_MS && raw != swStable) {
    swStable = raw;
    if (swStable == LOW) {
      swDown = true;
      swConsumed = false;
      swDownAt = now;
      pressEdge = true;
    } else {
      if (swDown && !swConsumed) shortPress = true;
      swDown = false;
    }
  }
  if (swDown && !swConsumed && (now - swDownAt) >= LONG_PRESS_MS) {
    swConsumed = true;
    longPress = true;
  }
}

bool isBusy() {
  return isHoming || positionControlActive || isHolding || freeMoveActive;
}

void fmtAngle(char* out, float v) {
  dtostrf(v, 1, 1, out); // out needs >= 8 bytes
}

// --- KY-040 rotary encoder (quadrature decoded from a 1 kHz timer interrupt) ---
volatile long knobQuarterSteps = 0;
volatile uint8_t knobPrevState = 0;
unsigned long lastKnobMs = 0;

void knobPoll() {
  // Transition table: index = (previous AB << 2) | current AB. Invalid (bounce) transitions give 0.
  static const int8_t table[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
  uint8_t cur = (digitalRead(KNOB_CLK_PIN) << 1) | digitalRead(KNOB_DT_PIN);
  knobPrevState = ((knobPrevState << 2) | cur) & 0x0F;
  knobQuarterSteps += table[knobPrevState];
}

// Timer0 compare-A fires about once per millisecond alongside the Arduino millis() tick;
// it samples the knob and both limit switches.
// (Timer0 compare-A isn't used for PWM by this sketch: the motor PWM is on pin 8 / Timer4.)
// Samples both limit switches: debounced, latched, and cuts the motor instantly on a hit
// in the direction of travel, without waiting for the main loop.
void limitPoll() {
  static uint8_t revCount = 0;
  static uint8_t fwdCount = 0;

  if (digitalRead(REVERSE_LIMIT_PIN) == LOW) {
    if (revCount < LIMIT_DEBOUNCE_SAMPLES) revCount++;
  } else {
    revCount = 0;
  }
  bool revNow = (revCount >= LIMIT_DEBOUNCE_SAMPLES);
  if (revNow && !revLimitActive) {
    revLimitLatched = true;
    revLimitHitPos = encoderpos;
    if (outputDIR == REVERSE) analogWrite(MOTOR_PWM, 0);
  }
  revLimitActive = revNow;

  if (digitalRead(FORWARD_LIMIT_PIN) == LOW) {
    if (fwdCount < LIMIT_DEBOUNCE_SAMPLES) fwdCount++;
  } else {
    fwdCount = 0;
  }
  bool fwdNow = (fwdCount >= LIMIT_DEBOUNCE_SAMPLES);
  if (fwdNow && !fwdLimitActive) {
    fwdLimitLatched = true;
    if (outputDIR == FORWARD) analogWrite(MOTOR_PWM, 0);
  }
  fwdLimitActive = fwdNow;
}

ISR(TIMER0_COMPA_vect) {
  knobPoll();
  limitPoll();
}

// Returns whole clicks turned since the last call (positive = clockwise)
int readKnobDetents() {
  noInterrupts();
  long q = knobQuarterSteps;
  interrupts();
  long clicks = q / KNOB_STEPS_PER_DETENT;
  if (clicks != 0) {
    noInterrupts();
    knobQuarterSteps -= clicks * KNOB_STEPS_PER_DETENT;
    interrupts();
  }
  return (int)(clicks * KNOB_DIRECTION);
}

// Adjust an angle by knob clicks: 0.5° per click, larger steps when spun quickly
float adjustAngle(float angle, int clicks) {
  if (clicks == 0) return angle;
  unsigned long now = millis();
  unsigned long gap = (now - lastKnobMs) / (unsigned long)abs(clicks); // ms per click
  lastKnobMs = now;

  float step = 0.5f;
  if (gap < 50) step = 5.0f;
  else if (gap < 120) step = 1.0f;

  angle += clicks * step;
  angle = roundf(angle * 2.0f) / 2.0f;
  if (angle < 0.0f) angle = 0.0f;
  if (angle > SOFT_MAX_ANGLE) angle = SOFT_MAX_ANGLE;
  return angle;
}

void uiMenuSelect() {
  switch (menuIndex) {
    case 0: // Clamps toggle
      setClamps(!clampsEngaged);
      break;
    case 1: // Home
      if (!clampsEngaged) showMsg("SET CLAMPS FIRST");
      else startHoming();
      break;
    case 2: // Bend to a knob-selected angle
      if (!clampsEngaged) showMsg("SET CLAMPS FIRST");
      else { anglePurpose = PURPOSE_BEND; uiState = UI_SET_ANGLE; }
      break;
    case 3: // Set the saved setpoint
      anglePurpose = PURPOSE_SETPOINT;
      uiAngle = savedSetpoint;
      uiState = UI_SET_ANGLE;
      break;
    case 4: // Bend to the saved setpoint
      if (!clampsEngaged) showMsg("SET CLAMPS FIRST");
      else if (savedSetpoint < SOFT_MIN_ANGLE || savedSetpoint > SOFT_MAX_ANGLE) showMsg("SETPOINT OUT OF RANGE");
      else {
        uiBendAngle = savedSetpoint;
        uiBendLabel = "Saved Setpoint";
        uiState = UI_CONFIRM;
      }
      break;
  }
}

void uiHandleInput() {
  bool pressEdge = false, shortPress = false, longPress = false;
  readKnobSwitch(pressEdge, shortPress, longPress);
  int clicks = readKnobDetents(); // Always drained so turns during motion don't queue up

  // While moving, any push of the knob is an immediate stop (turning does nothing)
  if (isBusy()) {
    if (pressEdge) {
      stopMotor();
      swConsumed = true; // The release of this press must not count as a select
      showMsg("STOPPED");
      Serial.println("Stopping Motor (knob pressed).");
      uiState = UI_MENU;
    }
    return;
  }

  // Hold the knob = back / cancel
  if (longPress) {
    uiState = UI_MENU;
    return;
  }

  bool enterPressed = shortPress;

  switch (uiState) {
    case UI_MENU:
      menuIndex = constrain(menuIndex + clicks, 0, MENU_ITEMS - 1);
      if (enterPressed) uiMenuSelect();
      break;

    case UI_SET_ANGLE:
      uiAngle = adjustAngle(uiAngle, clicks);
      if (enterPressed) {
        if (anglePurpose == PURPOSE_SETPOINT) {
          savedSetpoint = uiAngle;
          showMsg("SETPOINT SAVED");
          Serial.print("Saved Setpoint set from knob: ");
          Serial.print(savedSetpoint, 1);
          Serial.println("°");
          uiState = UI_MENU;
        } else {
          uiBendAngle = uiAngle;
          uiBendLabel = "target";
          uiState = UI_CONFIRM;
        }
      }
      break;

    case UI_CONFIRM:
      if (enterPressed) {
        uiState = UI_MENU;
        if (!clampsEngaged) {
          showMsg("SET CLAMPS FIRST");
        } else {
          startBendWithHoming(uiBendAngle, uiBendLabel);
        }
      }
      break;
  }
}

void uiDraw(float currentAngle) {
  char buf[26];
  char num[12];

  u8g2.clearBuffer();

  if (isBusy()) {
    // --- Running screen (drawn once when a move starts; never refreshed during the move,
    // because an OLED redraw blocks the control loop and ruins position control) ---
    bool homingOnly = isHoming && !pendingBend && !pendingMove;
    bool moveOnly = freeMoveActive || pendingMove;
    float tgt = isHoming ? (pendingBend ? pendingBendAngle : (pendingMove ? pendingMoveAngle : 0.0f)) : finalTargetAngle;

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, homingOnly ? "HOMING..." : (moveOnly ? "MOVING TO POSITION..." : "RUNNING BEND..."));

    if (!homingOnly) {
      u8g2.setFont(u8g2_font_10x20_tf);
      fmtAngle(num, tgt);
      snprintf(buf, sizeof(buf), "%s deg", num);
      u8g2.drawStr(0, 40, buf);
    }

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 62, "PUSH KNOB = STOP");
  }
  else if (uiState == UI_MENU) {
    // --- Menu ---
    u8g2.setFont(u8g2_font_6x10_tf);
    if (msgActive()) {
      u8g2.drawStr(0, 10, uiMessage);
    } else {
      fmtAngle(num, currentAngle);
      snprintf(buf, sizeof(buf), "Ang:%s Clamps:%s", num, clampsEngaged ? "YES" : "NO");
      u8g2.drawStr(0, 10, buf);
    }

    for (int i = 0; i < MENU_ITEMS; i++) {
      switch (i) {
        case 0: snprintf(buf, sizeof(buf), "%c Clamps: %s", i == menuIndex ? '>' : ' ', clampsEngaged ? "YES" : "NO"); break;
        case 1: snprintf(buf, sizeof(buf), "%c Home", i == menuIndex ? '>' : ' '); break;
        case 2: snprintf(buf, sizeof(buf), "%c Bend to angle", i == menuIndex ? '>' : ' '); break;
        case 3: fmtAngle(num, savedSetpoint); snprintf(buf, sizeof(buf), "%c Set saved: %s", i == menuIndex ? '>' : ' ', num); break;
        default: fmtAngle(num, savedSetpoint); snprintf(buf, sizeof(buf), "%c Bend saved: %s", i == menuIndex ? '>' : ' ', num); break;
      }
      u8g2.drawStr(0, 22 + i * 10, buf);
    }
  }
  else if (uiState == UI_SET_ANGLE) {
    // --- Angle selection ---
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, anglePurpose == PURPOSE_BEND ? "BEND ANGLE" : "SET SAVED ANGLE");
    u8g2.setFont(u8g2_font_10x20_tf);
    fmtAngle(num, uiAngle);
    snprintf(buf, sizeof(buf), "%s deg", num);
    u8g2.drawStr(0, 40, buf);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 62, "PUSH=OK  HOLD=BACK");
  }
  else {
    // --- Confirm ---
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "START BEND?");
    u8g2.setFont(u8g2_font_10x20_tf);
    fmtAngle(num, uiBendAngle);
    snprintf(buf, sizeof(buf), "%s deg", num);
    u8g2.drawStr(0, 40, buf);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 52, "Homes first, then bends");
    u8g2.drawStr(0, 62, "PUSH=GO  HOLD=CANCEL");
  }

  u8g2.sendBuffer();
}

void uiInit() {
  pinMode(BTN_ENTER_PIN, INPUT_PULLUP);

  pinMode(KNOB_CLK_PIN, INPUT_PULLUP);
  pinMode(KNOB_DT_PIN, INPUT_PULLUP);
  knobPrevState = (digitalRead(KNOB_CLK_PIN) << 1) | digitalRead(KNOB_DT_PIN);
  OCR0A = 0xAF;          // Compare point inside Timer0's cycle
  TIMSK0 |= _BV(OCIE0A); // Enable the ~1 kHz knob sampling interrupt

  u8g2.begin();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 24, "BEND CONTROLLER");
  u8g2.drawStr(0, 40, "Set clamps, then Home");
  u8g2.sendBuffer();
}

// Cheap fingerprint of everything the screen shows; redraw only when it changes
unsigned long uiSignature(float currentAngle) {
  unsigned long sig = 17;
  sig = sig * 31 + (unsigned long)uiState;
  sig = sig * 31 + (unsigned long)(menuIndex + 1);
  sig = sig * 31 + (unsigned long)(long)(uiAngle * 10.0f);
  sig = sig * 31 + (unsigned long)(long)(uiBendAngle * 10.0f);
  sig = sig * 31 + (unsigned long)(long)(currentAngle * 10.0f);
  sig = sig * 31 + (unsigned long)(long)(savedSetpoint * 10.0f);
  sig = sig * 31 + (unsigned long)(long)(finalTargetAngle * 10.0f);
  sig = sig * 31 + (unsigned long)(clampsEngaged ? 1 : 2);
  sig = sig * 31 + (unsigned long)(isHoming ? 1 : 2);
  sig = sig * 31 + (unsigned long)(isHolding ? 1 : 2);
  sig = sig * 31 + (unsigned long)(isDwelling ? 1 : 2);
  sig = sig * 31 + (unsigned long)(positionControlActive ? 1 : 2);
  sig = sig * 31 + (unsigned long)(pendingBend ? 1 : 2);
  sig = sig * 31 + (unsigned long)currentMoveStage;
  sig = sig * 31 + (unsigned long)currentPWM;
  sig = sig * 31 + (unsigned long)(msgActive() ? 1 : 2);
  sig = sig * 31 + uiMessageUntil;
  return sig;
}

bool busyScreenDrawn = false;

void uiService(float currentAngle) {
  uiHandleInput();

  if (isBusy()) {
    // Draw the running screen once, before the motor gets going, then leave the display
    // alone until the move ends. Redrawing mid-move blocks the loop long enough for the
    // arm to overshoot the target.
    if (!busyScreenDrawn) {
      busyScreenDrawn = true;
      uiDraw(currentAngle);
      lastDisplaySig = 0; // Force a fresh redraw once the move is over
    }
    return;
  }
  busyScreenDrawn = false;

  if (millis() - lastDisplayMs >= DISPLAY_INTERVAL_IDLE_MS) {
    lastDisplayMs = millis();
    unsigned long sig = uiSignature(currentAngle);
    if (sig != lastDisplaySig) {
      lastDisplaySig = sig;
      uiDraw(currentAngle);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Pin Configurations
  pinMode(REVERSE_LIMIT_PIN, INPUT_PULLUP);
  pinMode(FORWARD_LIMIT_PIN, INPUT_PULLUP);
  pinMode(LED_ARD, OUTPUT);

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);

  // Attach Encoder Interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), readEncoderB, CHANGE);

  stopMotor();
  uiInit();
  Serial.println("Production Controller Ready.");
  Serial.println("Commands:");
  Serial.println("  Y - Clamps Engaged (YES) | N - Clamps Disengaged (NO)");
  Serial.println("  H - Start Homing Routine");
  Serial.println("  F - Manual Forward | R - Manual Reverse | S - Stop");
  Serial.println("  A<angle> - Home, then staged bend to Absolute Angle (e.g., A45)");
  Serial.println("  P<angle> - Home, then low-PWM position move, no sheet metal (e.g., P45)");
  Serial.println("  SV       - Save current target angle as Saved Setpoint");
  Serial.println("  B        - Move to Saved Setpoint");
  Serial.println("  +<val> / -<val> - Adjust Saved Setpoint by float (e.g., +0.5, -1.0)");
}

void loop() {
  // --- 1. Read Inputs & Sensors ---
  // A switch counts as hit if it is pressed now OR was pressed at any point since the
  // last loop pass (latched by the interrupt, so brief actuations aren't missed).
  noInterrupts();
  bool revEdgeSeen = revLimitLatched;
  bool fwdEdgeSeen = fwdLimitLatched;
  revLimitLatched = false;
  fwdLimitLatched = false;
  interrupts();
  bool isRevLimitHit = revLimitActive || revEdgeSeen;
  bool isFwdLimitHit = fwdLimitActive || fwdEdgeSeen;

  // LED ON if manual clamps are verified YES
  digitalWrite(LED_ARD, clampsEngaged ? HIGH : LOW);

  noInterrupts();
  long currentPosition = encoderpos;
  interrupts();
  float currentAngle = (float)currentPosition / COUNTS_PER_DEGREE;

  // Operator interface: buttons, knob, OLED (STOP is polled every loop, even while homing)
  uiService(currentAngle);

  // --- 2. Automated Homing Routine Handler ---
  if (isHoming) {
    // Serial stays live during homing so the operator can abort with S (or N)
    processSerial(isRevLimitHit, isFwdLimitHit, currentAngle);
    if (!isHoming) {
      delay(20);
      return; // Aborted by operator
    }

    if (isRevLimitHit) {
      // Capture the queued bend before stopMotor() clears it
      bool launchBend = pendingBend;
      bool launchMove = pendingMove;
      float moveAngle = pendingMoveAngle;
      float bendAngle = pendingBendAngle;
      const char* bendLabel = pendingBendLabel;

      stopMotor();
      // Zero at the exact position where the switch tripped (captured in the interrupt),
      // not wherever the arm happened to stop. If the arm was already sitting on the
      // switch, zero at the current position.
      noInterrupts();
      long zeroAt = revEdgeSeen ? revLimitHitPos : encoderpos;
      encoderpos -= zeroAt;
      interrupts();
      targetEncoderPos = 0;
      targetAngle = 0.0;
      finalTargetAngle = 0.0;
      showMsg("HOMED"); Serial.println("Homing Complete. Position zeroed.");

      if (launchBend && clampsEngaged) {
        startStagedMove(bendAngle, bendLabel);
      } else if (launchMove && clampsEngaged) {
        startFreeMove(moveAngle);
      }
    } else {
      // Move toward home (Reverse) safely with ramping
      requestMotor(REVERSE, HOMING_SPEED);
      updateMotorRamp();
    }
    delay(20);
    return;
  }

  // --- 3. Limit Switch & Software Limit Protections ---
  if (isRevLimitHit && currentDIR == REVERSE) {
    stopMotor();
    showMsg("REV LIMIT HIT"); Serial.println("WARNING: Reverse Limit Triggered! Motion stopped.");
  }
  if (isFwdLimitHit && currentDIR == FORWARD) {
    stopMotor();
    showMsg("FWD LIMIT HIT"); Serial.println("WARNING: Forward Limit Triggered! Motion stopped.");
  }

  // Software Limit Checks during movement
  if (currentAngle <= SOFT_MIN_ANGLE && currentDIR == REVERSE) {
    stopMotor();
    showMsg("SOFT MIN LIMIT"); Serial.println("WARNING: Software Min Limit Reached!");
  }
  if (currentAngle >= SOFT_MAX_ANGLE && currentDIR == FORWARD) {
    stopMotor();
    showMsg("SOFT MAX LIMIT"); Serial.println("WARNING: Software Max Limit Reached!");
  }

  // --- 3.5. Stall Detection Logic (Universal Jam Protection) ---
  // Not evaluated while dwelling, homing, or holding: the hold phase intentionally
  // pushes against springback and may sit still at PWM values above the threshold.
  if (currentDIR != STOP && !isDwelling && !isHoming && !isHolding && currentMoveStage != STAGE_PAUSE) {
    if (millis() - lastStallCheckTime >= STALL_CHECK_INTERVAL) {
      long encoderDelta = abs(currentPosition - lastStallEncoderPos);

      // Forward at/above BEND_MIN_PWM (or reverse above its low threshold) should move
      // the arm; if the encoder still isn't changing, count it toward a stall trip.
      int stallThreshold = (currentDIR == FORWARD) ? FORWARD_STALL_PWM_THRESHOLD : REVERSE_STALL_PWM_THRESHOLD;
      bool underPower = (currentDIR == FORWARD) ? (currentPWM >= stallThreshold) : (currentPWM > stallThreshold);
      if (freeMoveActive) {
        // Unloaded move: even low forward PWM should turn the encoder
        underPower = currentPWM > REVERSE_STALL_PWM_THRESHOLD;
      }

      if (underPower && encoderDelta < MIN_EXPECTED_COUNTS) {
        stallCounter++;
        if (stallCounter >= STALL_TRIP_COUNT) {
          stopMotor();
          stallCounter = 0;
          showMsg("STALL! STOPPED"); Serial.println("CRITICAL ERROR: Motor Stall Detected! Motion stopped to prevent damage.");
        }
      } else {
        stallCounter = 0; // Reset counter if normal movement is detected
      }

      lastStallCheckTime = millis();
      lastStallEncoderPos = currentPosition;
    }
  } else {
    // Reset when motor is stopped, homing, dwelling, or holding
    stallCounter = 0;
    lastStallCheckTime = millis();
    lastStallEncoderPos = currentPosition;
  }

  // --- 4a. Free move (P command): unloaded, low PWM, no staging / dwell / hold ---
  if (freeMoveActive) {
    if (!clampsEngaged) {
      stopMotor();
      showMsg("CLAMPS OFF-STOPPED"); Serial.println("ERROR: Clamps disengaged during move! Stopped.");
    } else {
      long fmError = targetEncoderPos - currentPosition;
      long fmAbs = abs(fmError);

      if (fmAbs <= IN_POSITION_WINDOW) {
        softStopAll();
        showMsg("AT POSITION");
        Serial.print("Position reached. Angle: ");
        Serial.println(currentAngle, 1);
      } else {
        MotorDIR fmDir = (fmError > 0) ? FORWARD : REVERSE;

        // Direction reversals mean overshoot / hunting: give up after MAX_POSITION_PASSES
        if (freeMoveLastDir != STOP && fmDir != freeMoveLastDir) {
          freeMovePasses++;
          if (freeMovePasses >= MAX_POSITION_PASSES) {
            softStopAll();
            showMsg("POSITION SETTLED");
            Serial.print("WARNING: Max correction passes reached; stopping (error ");
            Serial.print(fmAbs);
            Serial.println(" counts).");
            fmDir = STOP;
          }
        }

        if (fmDir != STOP) {
          freeMoveLastDir = fmDir;

          // Cruise at FREE_MOVE_PWM, taper to FREE_MOVE_MIN_PWM on approach
          int fmSpeed = FREE_MOVE_PWM;
          if (fmAbs < APPROACH_RAMP_COUNTS) {
            fmSpeed = map(fmAbs, 0, APPROACH_RAMP_COUNTS, FREE_MOVE_MIN_PWM, FREE_MOVE_PWM);
          }

          if (fmDir == FORWARD && (isFwdLimitHit || currentAngle >= SOFT_MAX_ANGLE)) {
            stopMotor();
            showMsg("FWD LIMIT-STOPPED"); Serial.println("ERROR: Forward limit or software max reached during move.");
          } else if (fmDir == REVERSE && (isRevLimitHit || currentAngle <= SOFT_MIN_ANGLE)) {
            stopMotor();
            showMsg("REV LIMIT-STOPPED"); Serial.println("ERROR: Reverse limit or software min reached during move.");
          } else {
            requestFreeMove(fmDir, fmSpeed);
          }
        }
      }
    }
  }

  // --- 4. Closed-Loop Position Controller ---
  // No early returns in this block: serial commands (S / N) must stay responsive
  // during every stage, including the pause and dwell.
  if (positionControlActive) {
    if (!clampsEngaged) {
      stopMotor();
      showMsg("CLAMPS OFF-STOPPED"); Serial.println("ERROR: Clamps disengaged during positioning! Stopped.");
    }
    else if (currentMoveStage == STAGE_PAUSE) {
      // Pause between the 75% leg and the final leg: keep light holding torque on
      holdTorque(targetEncoderPos - currentPosition);

      if (millis() - stagePauseTimer >= STAGE_PAUSE_DURATION_MS) {
        targetAngle = finalTargetAngle;
        targetEncoderPos = (long)(targetAngle * COUNTS_PER_DEGREE);
        currentMoveStage = STAGE_FINAL_LEG;
        positionPasses = 0;
        lastCorrectionDir = STOP;
        Serial.print("Pause complete. Moving to final target: ");
        Serial.print(targetAngle);
        Serial.println("°");
      }
    }
    else if (isDwelling) {
      // Dwell at target to let backlash/springback settle, then re-check.
      // Holding torque stays on so springback doesn't undo the bend during the dwell.
      holdTorque(targetEncoderPos - currentPosition);

      if (millis() - dwellTimer >= DWELL_TIME_MS) {
        isDwelling = false;
        long dwellAbsError = abs(targetEncoderPos - currentPosition);

        if (dwellAbsError <= IN_POSITION_WINDOW) {
          Serial.print("Pos: ");
          Serial.print(currentPosition);
          Serial.print(" | Target Reached. Starting Hold Duration. Angle: ");
          Serial.println(targetAngle);
          beginHold();
        } else if (positionPasses < MAX_POSITION_PASSES) {
          positionPasses++;
          Serial.print("Dwell re-check: off by ");
          Serial.print(dwellAbsError);
          Serial.print(" counts. Correcting (pass ");
          Serial.print(positionPasses);
          Serial.print("/");
          Serial.print(MAX_POSITION_PASSES);
          Serial.println(").");
          lastCorrectionDir = STOP;
        } else {
          settleOrAbort(dwellAbsError);
        }
      }
    }
    else {
      long error = targetEncoderPos - currentPosition;
      long absError = abs(error);

      if (absError <= IN_POSITION_WINDOW) {
        // Target reached window
        if (currentMoveStage == STAGE_FIRST_LEG) {
          currentMoveStage = STAGE_PAUSE;
          stagePauseTimer = millis();
          holdTorque(error);
          Serial.println("Reached 75% intermediate target. Pausing...");
        }
        else if (currentMoveStage == STAGE_FINAL_LEG) {
          // Stop and dwell before the final position check / hold
          isDwelling = true;
          dwellTimer = millis();
          holdTorque(error);
        }
      }
      else {
        MotorDIR neededDir = (error > 0) ? FORWARD : REVERSE;

        // Count direction reversals on the final leg (overshoot / hunting)
        if (currentMoveStage == STAGE_FINAL_LEG && lastCorrectionDir != STOP && neededDir != lastCorrectionDir) {
          positionPasses++;
          if (positionPasses >= MAX_POSITION_PASSES) {
            settleOrAbort(absError);
            lastCorrectionDir = STOP;
            neededDir = STOP;
          }
        }

        if (neededDir != STOP) {
          lastCorrectionDir = neededDir;

          if (neededDir == FORWARD) {
            if (isFwdLimitHit || currentAngle >= SOFT_MAX_ANGLE) {
              stopMotor();
              showMsg("FWD LIMIT-STOPPED"); Serial.println("ERROR: Forward limit or software max reached during positioning.");
            } else {
              // High-torque approach: full speed far away, ramp down inside the
              // approach window, but never below APPROACH_MIN_PWM.
              int commandedSpeed = POSITION_SPEED;
              if (absError < APPROACH_RAMP_COUNTS) {
                commandedSpeed = map(absError, 0, APPROACH_RAMP_COUNTS, APPROACH_MIN_PWM, POSITION_SPEED);
              }
              requestMotor(FORWARD, commandedSpeed);
            }
          } else {
            if (isRevLimitHit || currentAngle <= SOFT_MIN_ANGLE) {
              stopMotor();
              showMsg("REV LIMIT-STOPPED"); Serial.println("ERROR: Reverse limit or software min reached during positioning.");
            } else {
              // Reverse corrections scale with error so a small overshoot gets a
              // gentle nudge instead of a hard jolt.
              int commandedSpeed = POSITION_SPEED;
              if (absError < APPROACH_RAMP_COUNTS) {
                commandedSpeed = map(absError, 0, APPROACH_RAMP_COUNTS, REVERSE_CORRECT_MIN_PWM, POSITION_SPEED);
              }
              requestMotor(REVERSE, commandedSpeed);
            }
          }
        }
      }
    }
  }
  else if (!isHolding) {
    positionPasses = 0; // Reset pass counter when position control is idle
    lastCorrectionDir = STOP;
  }

  // --- Closed-Loop Holding Phase Handler ---
  if (isHolding) {
    if (millis() - holdTimer >= HOLD_DURATION_MS) {
      softStopAll(); // Ramp the torque off smoothly
      showMsg("BEND DONE"); Serial.println("Hold duration complete. Motor ramping to stop.");
    } else {
      // Error against the final target (positive = slipped backward by springback)
      holdTorque(targetEncoderPos - currentPosition);
    }
  }

  // --- 5. Process Serial Commands ---
  processSerial(isRevLimitHit, isFwdLimitHit, currentAngle);

  // --- 6. Apply Ramping & Update Motor Output ---
  updateMotorRamp();

  // --- 7. Telemetry Output ---
  static bool lastRevState = false;
  static bool lastFwdState = false;
  static bool lastClampState = false;

  if (currentPosition != lastReportedPosition || isRevLimitHit != lastRevState || isFwdLimitHit != lastFwdState || clampsEngaged != lastClampState) {
    Serial.print("Pos: ");
    Serial.print(currentPosition);
    Serial.print(" | Angle: ");
    Serial.print(currentAngle, 1);
    Serial.print("° | Saved Setpoint: ");
    Serial.print(savedSetpoint, 1);
    Serial.print("° | REV: ");
    Serial.print(isRevLimitHit ? "HIT" : "OK");
    Serial.print(" | FWD: ");
    Serial.print(isFwdLimitHit ? "HIT" : "OK");
    Serial.print(" | Clamps: ");
    Serial.print(clampsEngaged ? "YES" : "NO");
    Serial.print(" | PWM: ");
    Serial.print(currentPWM);
    Serial.print(" | State: ");
    Serial.println(getDirString(currentDIR));

    lastReportedPosition = currentPosition;
    lastRevState = isRevLimitHit;
    lastFwdState = isFwdLimitHit;
    lastClampState = clampsEngaged;
  }

  delay(20);
}
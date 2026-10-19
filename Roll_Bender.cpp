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

// Operator interface (OLED + potentiometer + two buttons)
// OLED on the hardware I2C pins (Mega: SDA 20 / SCL 21, Uno: SDA A4 / SCL A5)
const int POT_PIN = A0;        // Potentiometer wiper
const int BTN_ENTER_PIN = 4;   // Pushbutton to GND: select / confirm
const int BTN_STOP_PIN = 6;    // Pushbutton to GND: stop / back
const unsigned long BUTTON_DEBOUNCE_MS = 30;

// Most 1.3" 128x64 I2C OLEDs use the SH1106 controller (0.96" ones use SSD1306).
// If the image is shifted ~2 px or has a garbage column, swap to the SSD1306 line.
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// --- Calibration & Limits Constants ---
const float COUNTS_PER_DEGREE = 6.6; // Tuned empirically

// Software Limits (Virtual boundaries before hitting physical switches)
const float SOFT_MIN_ANGLE = -1.0;   // Just below home (0°)
const float SOFT_MAX_ANGLE = 131;    // Adjust based on your maximum safe range

// Motion Settings
const int HOMING_SPEED = 50;
const int POSITION_SPEED = 130;
const int DEAD_BAND = 4;          // Allowable error in counts (hold phase)
const int RAMP_STEP = 10;         // PWM step increment for soft start/stop
const long IN_POSITION_WINDOW = 3; // Counts: "target reached" window

// Approach speeds
// Below BEND_MIN_PWM the motor cannot bend the metal (forward = loaded direction),
// so every FORWARD command is kept at or above it. Reverse is unloaded and may run lower.
const int BEND_MIN_PWM = 90;
const int APPROACH_MIN_PWM = 115;  // Forward torque floor while bending (must be >= BEND_MIN_PWM)
const int REVERSE_CORRECT_MIN_PWM = 60; // Gentle floor for reverse corrections (overshoot recovery)
const long APPROACH_RAMP_COUNTS = 30;   // Start slowing down inside this many counts

// --- Variables ---
volatile long encoderpos = 0;
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

// Call this every loop iteration to smoothly handle acceleration/deceleration
void updateMotorRamp() {
  // Don't ramp through the dead zone on forward moves: PWM below BEND_MIN_PWM can't
  // bend the metal, so jump straight to the bending floor and ramp from there.
  if (currentDIR == FORWARD && targetPWM >= BEND_MIN_PWM && currentPWM < BEND_MIN_PWM) {
    currentPWM = BEND_MIN_PWM;
  }

  if (currentPWM < targetPWM) {
    currentPWM = min(targetPWM, currentPWM + RAMP_STEP);
  } else if (currentPWM > targetPWM) {
    currentPWM = max(targetPWM, currentPWM - RAMP_STEP);
  }

  // Apply direction and ramped PWM to hardware driver
  if (currentDIR == FORWARD) {
    digitalWrite(MOTOR_DIR, DIR_FORWARD);
    analogWrite(MOTOR_PWM, currentPWM);
  } else if (currentDIR == REVERSE) {
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
  analogWrite(MOTOR_PWM, 0);
}

void stopMotor() {
  haltOutput();
  isHoming = false;
  pendingBend = false; // Any stop/abort cancels a queued bend
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

// Bend routine entry point: home first (the arm may have been moved while clamping),
// then run the staged bend automatically once the home switch zeroes the encoder.
void startBendWithHoming(float angle, const char* label) {
  positionControlActive = false;
  isHolding = false;
  isDwelling = false;
  currentMoveStage = STAGE_IDLE;
  pendingBendAngle = angle;
  pendingBendLabel = label;
  isHoming = true;
  pendingBend = true; // Set after clearing state above; stopMotor() would reset it
  Serial.print("Bend requested (");
  Serial.print(angle);
  Serial.println("°). Homing first...");
}

// Plain homing (no queued bend)
void startHoming() {
  Serial.println("Starting Homing Routine...");
  pendingBend = false;
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
  else if (cmd == 'B') {
    if (savedSetpoint < SOFT_MIN_ANGLE || savedSetpoint > SOFT_MAX_ANGLE) {
      Serial.println("ERROR: Saved Setpoint is out of software bounds!");
    } else {
      startBendWithHoming(savedSetpoint, "Saved Setpoint");
    }
  }
}

// ===================== OLED + Potentiometer + Buttons =====================
// Pot: scrolls the menu / sets the angle.  ENTER: select / confirm.  STOP: stop / back.
// Serial commands keep working alongside this.
enum UiState { UI_MENU, UI_SET_ANGLE, UI_CONFIRM };
enum AnglePurpose { PURPOSE_BEND, PURPOSE_SETPOINT };

UiState uiState = UI_MENU;
AnglePurpose anglePurpose = PURPOSE_BEND;

const int MENU_ITEMS = 5;
int menuIndex = 0;
float uiAngle = 0.0;            // Angle currently picked with the pot
float uiBendAngle = 0.0;        // Angle waiting for confirmation
const char* uiBendLabel = "target";

int potFiltered = 0;
unsigned long lastDisplayMs = 0;
const unsigned long DISPLAY_INTERVAL_MS = 150; // Each OLED redraw blocks the loop ~25 ms

struct Button {
  uint8_t pin;
  bool stable;
  bool lastRaw;
  unsigned long changedAt;
};
Button btnEnter = { (uint8_t)BTN_ENTER_PIN, HIGH, HIGH, 0 };
Button btnStop  = { (uint8_t)BTN_STOP_PIN,  HIGH, HIGH, 0 };

// Returns true once per debounced press
bool buttonPressed(Button &b) {
  bool raw = digitalRead(b.pin);
  unsigned long now = millis();
  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.changedAt = now;
  }
  if ((now - b.changedAt) >= BUTTON_DEBOUNCE_MS && raw != b.stable) {
    b.stable = raw;
    if (b.stable == LOW) return true;
  }
  return false;
}

bool isBusy() {
  return isHoming || positionControlActive || isHolding;
}

void fmtAngle(char* out, float v) {
  dtostrf(v, 1, 1, out); // out needs >= 8 bytes
}

void updatePot() {
  int raw = analogRead(POT_PIN);
  potFiltered = (potFiltered * 3 + raw) / 4;
}

// Menu selection with hysteresis so the cursor doesn't flicker on zone edges
void updateMenuIndex() {
  const int zone = 1024 / MENU_ITEMS;
  const int hyst = 20;
  int lo = menuIndex * zone - hyst;
  int hi = (menuIndex + 1) * zone + hyst;
  if (potFiltered < lo || potFiltered >= hi) {
    menuIndex = constrain(potFiltered / zone, 0, MENU_ITEMS - 1);
  }
}

// Full pot travel = 0° .. SOFT_MAX_ANGLE in 0.5° steps
float potToAngle() {
  float a = (potFiltered / 1023.0f) * SOFT_MAX_ANGLE;
  a = roundf(a * 2.0f) / 2.0f;
  if (a < 0.0f) a = 0.0f;
  if (a > SOFT_MAX_ANGLE) a = SOFT_MAX_ANGLE;
  return a;
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
    case 2: // Bend to a pot-selected angle
      if (!clampsEngaged) showMsg("SET CLAMPS FIRST");
      else { anglePurpose = PURPOSE_BEND; uiState = UI_SET_ANGLE; }
      break;
    case 3: // Set the saved setpoint
      anglePurpose = PURPOSE_SETPOINT;
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
  bool stopPressed = buttonPressed(btnStop);
  bool enterPressed = buttonPressed(btnEnter);

  // STOP works at all times: aborts motion, or backs out of a screen
  if (stopPressed) {
    if (isBusy()) {
      stopMotor();
      showMsg("STOPPED");
      Serial.println("Stopping Motor (STOP button).");
    }
    uiState = UI_MENU;
    return;
  }

  // While moving, only STOP does anything
  if (isBusy()) return;

  switch (uiState) {
    case UI_MENU:
      updateMenuIndex();
      if (enterPressed) uiMenuSelect();
      break;

    case UI_SET_ANGLE:
      uiAngle = potToAngle();
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
    // --- Running screen ---
    const char* st = "";
    if (isHoming)               st = pendingBend ? "HOMING (then bend)" : "HOMING";
    else if (isHolding)         st = "HOLDING";
    else if (isDwelling)        st = "SETTLING";
    else if (currentMoveStage == STAGE_FIRST_LEG) st = "BENDING (75%)";
    else if (currentMoveStage == STAGE_PAUSE)     st = "PAUSE";
    else if (currentMoveStage == STAGE_FINAL_LEG) st = "BENDING (FINAL)";

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, st);

    u8g2.setFont(u8g2_font_10x20_tf);
    fmtAngle(num, currentAngle);
    snprintf(buf, sizeof(buf), "%s deg", num);
    u8g2.drawStr(0, 36, buf);

    float tgt = isHoming ? (pendingBend ? pendingBendAngle : 0.0f) : finalTargetAngle;
    fmtAngle(num, tgt);
    u8g2.setFont(u8g2_font_6x10_tf);
    snprintf(buf, sizeof(buf), "Tgt %s  PWM %d", num, currentPWM);
    u8g2.drawStr(0, 50, buf);

    u8g2.drawStr(0, 62, msgActive() ? uiMessage : "STOP = ABORT");
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
    u8g2.drawStr(0, 62, "ENTER=OK  STOP=BACK");
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
    u8g2.drawStr(0, 62, "ENTER=GO  STOP=CANCEL");
  }

  u8g2.sendBuffer();
}

void uiInit() {
  pinMode(BTN_ENTER_PIN, INPUT_PULLUP);
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
  potFiltered = analogRead(POT_PIN);

  u8g2.begin();
  Wire.setClock(400000); // Faster I2C keeps each redraw short

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 24, "BEND CONTROLLER");
  u8g2.drawStr(0, 40, "Set clamps, then Home");
  u8g2.sendBuffer();
}

void uiService(float currentAngle) {
  updatePot();
  uiHandleInput();
  if (millis() - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs = millis();
    uiDraw(currentAngle);
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
  Serial.println("  A<angle> - Move to Absolute Angle (e.g., A45)");
  Serial.println("  SV       - Save current target angle as Saved Setpoint");
  Serial.println("  B        - Move to Saved Setpoint");
  Serial.println("  +<val> / -<val> - Adjust Saved Setpoint by float (e.g., +0.5, -1.0)");
}

void loop() {
  // --- 1. Read Inputs & Sensors ---
  bool isRevLimitHit = (digitalRead(REVERSE_LIMIT_PIN) == LOW);
  bool isFwdLimitHit = (digitalRead(FORWARD_LIMIT_PIN) == LOW);

  // LED ON if manual clamps are verified YES
  digitalWrite(LED_ARD, clampsEngaged ? HIGH : LOW);

  noInterrupts();
  long currentPosition = encoderpos;
  interrupts();
  float currentAngle = (float)currentPosition / COUNTS_PER_DEGREE;

  // Operator interface: buttons, pot, OLED (STOP is polled every loop, even while homing)
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
      float bendAngle = pendingBendAngle;
      const char* bendLabel = pendingBendLabel;

      stopMotor();
      noInterrupts();
      encoderpos = 0; // Zero position at home
      interrupts();
      targetEncoderPos = 0;
      targetAngle = 0.0;
      finalTargetAngle = 0.0;
      showMsg("HOMED"); Serial.println("Homing Complete. Position zeroed.");

      if (launchBend && clampsEngaged) {
        startStagedMove(bendAngle, bendLabel);
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
  if (currentDIR != STOP && !isDwelling && !isHoming && !isHolding) {
    if (millis() - lastStallCheckTime >= STALL_CHECK_INTERVAL) {
      long encoderDelta = abs(currentPosition - lastStallEncoderPos);

      // Forward at/above BEND_MIN_PWM (or reverse above its low threshold) should move
      // the arm; if the encoder still isn't changing, count it toward a stall trip.
      int stallThreshold = (currentDIR == FORWARD) ? FORWARD_STALL_PWM_THRESHOLD : REVERSE_STALL_PWM_THRESHOLD;
      bool underPower = (currentDIR == FORWARD) ? (currentPWM >= stallThreshold) : (currentPWM > stallThreshold);

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

  // --- 4. Closed-Loop Position Controller ---
  // No early returns in this block: serial commands (S / N) must stay responsive
  // during every stage, including the pause and dwell.
  if (positionControlActive) {
    if (!clampsEngaged) {
      stopMotor();
      showMsg("CLAMPS OFF-STOPPED"); Serial.println("ERROR: Clamps disengaged during positioning! Stopped.");
    }
    else if (currentMoveStage == STAGE_PAUSE) {
      // Pause between the 75% leg and the final leg
      haltOutput();

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
      // Dwell at target to let backlash/springback settle, then re-check
      haltOutput();

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
          haltOutput();
          Serial.println("Reached 75% intermediate target. Pausing...");
        }
        else if (currentMoveStage == STAGE_FINAL_LEG) {
          // Stop and dwell before the final position check / hold
          isDwelling = true;
          dwellTimer = millis();
          haltOutput();
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
      stopMotor();
      showMsg("BEND DONE"); Serial.println("Hold duration complete. Motor stopped.");
    } else {
      // Error against the final target (positive = slipped backward by springback)
      long holdError = targetEncoderPos - currentPosition;

      if (holdError > DEAD_BAND) {
        // Scale PWM with error size, between BEND_MIN_PWM and HOLDING_PWM
        int correctivePWM = constrain(holdError * 15, BEND_MIN_PWM, HOLDING_PWM);
        requestMotor(FORWARD, correctivePWM);
      }
      else if (holdError < -DEAD_BAND) {
        // Drifted forward past the target: nudge back gently
        requestMotor(REVERSE, 30);
      }
      else {
        // Within deadband: forward PWM under BEND_MIN_PWM does nothing useful, so
        // don't drive; corrections resume if springback exceeds the deadband.
        requestMotor(STOP, 0);
      }
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

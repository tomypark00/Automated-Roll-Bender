#include <Arduino.h>

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

// --- Calibration & Limits Constants ---
const float COUNTS_PER_DEGREE = 6.6; // Tuned empirically

// Software Limits (Virtual boundaries before hitting physical switches)
const float SOFT_MIN_ANGLE = -1.0;   // Just below home (0°)
const float SOFT_MAX_ANGLE = 131;  // Adjust based on your maximum safe range

// Motion Settings
const int HOMING_SPEED = 50;
const int POSITION_SPEED = 130;
const int DEAD_BAND = 2; // Allowable error in counts
const int RAMP_STEP = 5; // PWM step increment for soft start/stop

// --- Variables ---
volatile long encoderpos = 0;
long lastReportedPosition = 0;

// Backlash Correction & Dwell Variables
int positionPasses = 0;
const int MAX_POSITION_PASSES = 5; // Automatically re-runs position checks to clear backlash
unsigned long dwellTimer = 0;
bool isDwelling = false;
const unsigned long DWELL_TIME_MS = 300; // Dwell duration at target before final lock

// --- Stall Detection Variables ---
unsigned long lastStallCheckTime = 0;
long lastStallEncoderPos = 0;
const unsigned long STALL_CHECK_INTERVAL = 200; // Check every 200ms
const long MIN_EXPECTED_COUNTS = 2; // Minimum counts expected to move in that window
const int STALL_TRIP_COUNT = 3;     // Number of consecutive checks before triggering stall
int stallCounter = 0;

// Assign names using enum integer constants
enum MotorDIR { STOP, FORWARD, REVERSE };
MotorDIR currentDIR = STOP;

// --- Two-Stage Bending Variables ---
enum MoveStage { STAGE_IDLE, STAGE_FIRST_LEG, STAGE_PAUSE, STAGE_FINAL_LEG };
MoveStage currentMoveStage = STAGE_IDLE;
float finalTargetAngle = 0.0;
unsigned long stagePauseTimer = 0;
const unsigned long STAGE_PAUSE_DURATION_MS = 500; // Pause duration between 75% and final (0.5s)

// Control States
bool isHoming = false;
bool positionControlActive = false;
bool clampsEngaged = false; // Manual operator clamp confirmation (Yes/No)
long targetEncoderPos = 0;
float targetAngle = 0.0;

// Saved Setpoint Variable
float savedSetpoint = 0.0;

// Ramping Variables
int currentPWM = 0;
int targetPWM = 0;

//Holding Bend Variables
const int HOLDING_PWM = 100;                  // Low PWM to fight springback
const unsigned long HOLD_DURATION_MS = 500;   // Hold duration in milliseconds (e.g., 0.5 seconds)
bool isHolding = false;
unsigned long holdTimer = 0;

// Direction logic constants (swap HIGH/LOW if motor direction is reversed)
const uint8_t DIR_FORWARD = HIGH;
const uint8_t DIR_REVERSE = LOW;

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
  if (dir == STOP) {
    targetPWM = 0;
  }
}

// Call this every loop iteration to smoothly handle acceleration/deceleration
void updateMotorRamp() {
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

void stopMotor() {
  requestMotor(STOP, 0);
  currentPWM = 0; 
  analogWrite(MOTOR_PWM, 0);
  isHoming = false;
  positionControlActive = false;
  isHolding = false;
  currentMoveStage = STAGE_IDLE; // Reset stage
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

  // --- 2. Automated Homing Routine Handler ---
  if (isHoming) {
    if (isRevLimitHit) {
      stopMotor();
      noInterrupts();
      encoderpos = 0; // Zero position at home
      interrupts();
      targetEncoderPos = 0;
      targetAngle = 0.0;
      isHoming = false;
      Serial.println("Homing Complete. Position zeroed.");
      return;
    } else {
      // Move toward home (Reverse) safely with ramping
      requestMotor(REVERSE, HOMING_SPEED);
      updateMotorRamp();
      delay(20);
      return;
    }
  }

  // --- 3. Limit Switch & Software Limit Protections ---
  if (isRevLimitHit && currentDIR == REVERSE) {
    stopMotor();
    Serial.println("WARNING: Reverse Limit Triggered! Motion stopped.");
  }
  if (isFwdLimitHit && currentDIR == FORWARD) {
    stopMotor();
    Serial.println("WARNING: Forward Limit Triggered! Motion stopped.");
  }

  // Software Limit Checks during movement
  if (currentAngle <= SOFT_MIN_ANGLE && currentDIR == REVERSE) {
    stopMotor();
    Serial.println("WARNING: Software Min Limit Reached!");
  }
  if (currentAngle >= SOFT_MAX_ANGLE && currentDIR == FORWARD) {
    stopMotor();
    Serial.println("WARNING: Software Max Limit Reached!");
  }
  // --- 3.5. Stall Detection Logic ---
  if (currentDIR != STOP && !isDwelling && !isHoming) {
    if (millis() - lastStallCheckTime >= STALL_CHECK_INTERVAL) {
      long encoderDelta = abs(currentPosition - lastStallEncoderPos);
      
      // If motor is trying to move (PWM > 50), but encoder barely moved
      if (currentPWM > 50 && encoderDelta < MIN_EXPECTED_COUNTS) {
        stallCounter++;
        if (stallCounter >= STALL_TRIP_COUNT) {
          stopMotor();
          Serial.println("CRITICAL ERROR: Motor Stall Detected! Motion stopped to prevent damage.");
        }
      } else {
        stallCounter = 0; // Reset counter if normal movement is detected
      }
      
      lastStallCheckTime = millis();
      lastStallEncoderPos = currentPosition;
    }
  } else {
    // Reset when motor is stopped or dwelling/homing
    stallCounter = 0;
    lastStallCheckTime = millis();
    lastStallEncoderPos = currentPosition;
  }
  // --- 4. Closed-Loop Position Controller ---
  if (positionControlActive) {
    if (!clampsEngaged) {
      stopMotor();
      Serial.println("ERROR: Clamps disengaged during positioning! Stopped.");
      positionControlActive = false;
      positionPasses = 0;
      currentMoveStage = STAGE_IDLE;
    } else {
      
      // Handle the intermediate Pause Stage between 75% and 100%
      if (currentMoveStage == STAGE_PAUSE) {
        // Stop motor directly without wiping control state
        requestMotor(STOP, 0);
        currentPWM = 0;
        analogWrite(MOTOR_PWM, 0);
        
        if (millis() - stagePauseTimer >= STAGE_PAUSE_DURATION_MS) {
          // Transition to final leg (100% of target)
          targetAngle = finalTargetAngle;
          targetEncoderPos = (long)(targetAngle * COUNTS_PER_DEGREE);
          currentMoveStage = STAGE_FINAL_LEG;
          isDwelling = false;
          positionPasses = 0;
          Serial.print("Pause complete. Moving to final target: ");
          Serial.print(targetAngle);
          Serial.println("°");
        }
        return; // Skip rest of loop iteration during pause
      }

      long error = targetEncoderPos - currentPosition;
      long absError = abs(error);

      // Handle Dwell Phase if waiting at target between passes
      if (isDwelling) {
        // Stop motor directly without wiping control state
        requestMotor(STOP, 0);
        currentPWM = 0;
        analogWrite(MOTOR_PWM, 0);
        
        if (millis() - dwellTimer >= DWELL_TIME_MS) {
          isDwelling = false;
          // Re-check error after dwell to see if backlash settled it out of bounds
          long recheckError = targetEncoderPos - encoderpos;
          if (abs(recheckError) <= DEAD_BAND) {
            positionPasses++;
            if (positionPasses >= MAX_POSITION_PASSES) {
              positionControlActive = false;
              positionPasses = 0;
              currentMoveStage = STAGE_IDLE;
              isHolding = true;          // Start holding phase
              holdTimer = millis();      // Record start time
              requestMotor(FORWARD, HOLDING_PWM); // Apply holding torque
              
              Serial.print("Pos: ");
              Serial.print(currentPosition);
              Serial.print(" | Target Reached and Settled. Holding position. Angle: ");
              Serial.println(targetAngle);
            } else {
              // Start next pass to correct any residual backlash
            }
          }
        }
      } 
      else if (absError <= DEAD_BAND) {
        // Target reached for this leg
        if (currentMoveStage == STAGE_FIRST_LEG) {
          // Reached 75% intermediate target! Trigger pause stage.
          currentMoveStage = STAGE_PAUSE;
          stagePauseTimer = millis();
          // Stop motor directly without wiping control state
          requestMotor(STOP, 0);
          currentPWM = 0;
          analogWrite(MOTOR_PWM, 0);
          Serial.println("Reached 75% intermediate target. Pausing...");
        } else if (currentMoveStage == STAGE_FINAL_LEG) {
          // Reached final target, initiate dwell timer for multi-pass verification
          isDwelling = true;
          dwellTimer = millis();
        }
      } 
      else {
        // Full speed/torque maintained right up to the dead band
        int commandedSpeed = POSITION_SPEED;

        if (error > 0) {
          if (isFwdLimitHit || currentAngle >= SOFT_MAX_ANGLE) {
            stopMotor();
            positionControlActive = false;
            positionPasses = 0;
            currentMoveStage = STAGE_IDLE;
            Serial.println("ERROR: Forward limit or software max reached during positioning.");
          } else {
            requestMotor(FORWARD, commandedSpeed);
          }
        } else {
          if (isRevLimitHit || currentAngle <= SOFT_MIN_ANGLE) {
            stopMotor();
            positionControlActive = false;
            positionPasses = 0;
            currentMoveStage = STAGE_IDLE;
            Serial.println("ERROR: Reverse limit or software min reached during positioning.");
          } else {
            requestMotor(REVERSE, commandedSpeed);
          }
        }
      }
    }
  } else {
    positionPasses = 0; // Reset pass counter when position control is idle
  }

  // --- Holding Phase Handler ---
  if (isHolding) {
    if (millis() - holdTimer >= HOLD_DURATION_MS) {
      stopMotor();
      isHolding = false;
      Serial.println("Hold duration complete. Motor stopped.");
    }
  }
  // --- 5. Process Serial Commands ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      // Handle multi-character / specific commands first
      if (input.equalsIgnoreCase("SV")) {
        savedSetpoint = targetAngle;
        Serial.print("SUCCESS: Current target angle (");
        Serial.print(savedSetpoint);
        Serial.println("°) saved as Saved Setpoint.");
      }
      else if (input.startsWith("+")) {
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
      } 
      else if (input.startsWith("-")) {
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
      }
      else {
        // Single character / prefixed commands
        char cmd = toupper(input.charAt(0));

        if (cmd == 'S') {
          Serial.println("Stopping Motor.");
          stopMotor();
        }
        else if (cmd == 'Y') {
          clampsEngaged = true;
          Serial.println("CLAMP STATUS: YES (All clamps engaged. Motion permitted.)");
        }
        else if (cmd == 'N') {
          clampsEngaged = false;
          stopMotor();
          Serial.println("CLAMP STATUS: NO (Clamps disengaged. Motion locked.)");
        }
        else {
          // Safety Check: Enforce Clamps = YES before initiating any new motion
          if (!clampsEngaged) {
            Serial.println("ERROR: Cannot start motion - Please send 'Y' to verify clamps are engaged first!");
          } else {
            if (cmd == 'H') {
              Serial.println("Starting Homing Routine...");
              isHoming = true;
              positionControlActive = false;
            } 
            else if (cmd == 'F') {
              if (isFwdLimitHit || currentAngle >= SOFT_MAX_ANGLE) {
                Serial.println("ERROR: Cannot move Forward - limit or max software bound reached!");
              } else {
                positionControlActive = false;
                Serial.println("Manual Moving FORWARD...");
                requestMotor(FORWARD, POSITION_SPEED);
              }
            } 
            else if (cmd == 'R') {
              if (isRevLimitHit || currentAngle <= SOFT_MIN_ANGLE) {
                Serial.println("ERROR: Cannot move Reverse - limit or min software bound reached!");
              } else {
                positionControlActive = false;
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
                finalTargetAngle = angleVal;
                targetAngle = finalTargetAngle * 0.75f; // Set intermediate target to 75%
                targetEncoderPos = (long)(targetAngle * COUNTS_PER_DEGREE);
                currentMoveStage = STAGE_FIRST_LEG;
                positionControlActive = true;
                isDwelling = false;      
                positionPasses = 0;      
                isHolding = false;
                Serial.print("Staged Bend: Moving to 75% intermediate angle: ");
                Serial.print(targetAngle);
                Serial.print("° (Final Target: ");
                Serial.print(finalTargetAngle);
                Serial.println("°)");
                Serial.print(targetEncoderPos);
                Serial.println(")");
              }
            } 
            else if (cmd == 'B') {
              if (savedSetpoint < SOFT_MIN_ANGLE || savedSetpoint > SOFT_MAX_ANGLE) {
                Serial.println("ERROR: Saved Setpoint is out of software bounds!");
              } else {
                finalTargetAngle = savedSetpoint;
                targetAngle = finalTargetAngle * 0.75f; // Set intermediate target to 75%
                targetEncoderPos = (long)(targetAngle * COUNTS_PER_DEGREE);
                currentMoveStage = STAGE_FIRST_LEG;
                positionControlActive = true;
                isDwelling = false;      
                positionPasses = 0;      
                isHolding = false;
                Serial.print("Staged Bend: Moving to Saved Setpoint 75% intermediate angle: ");
                Serial.print(targetAngle);
                Serial.print("° (Final Target: ");
                Serial.print(finalTargetAngle);
                Serial.println("°)");
              }
            }
          }
        }
      }
    }
  }

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
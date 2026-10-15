#include <Arduino.h>

//Encoder
const int ENCODER_PIN_A = 2;  // Phase A (White Wire)
const int ENCODER_PIN_B = 3;  // Phase B (Green Wire)
//Limit switch
const int REVERSE_LIMIT_PIN = 5;
const int FORWARD_LIMIT_PIN = 10;
//Motor
const int MOTOR_PWM = 8;
const int MOTOR_DIR = 7;
//Onboard LED
const int LED_ARD = 13;

//Variables
volatile long encoderpos = 0;
long lastReportedPosition = 0;

//Assign names to using enum integer constants. STOP = 0, FORWARD = 1, REVERSE = 2
enum MotorDIR {STOP, FORWARD, REVERSE};
MotorDIR currentDIR = STOP;

// Direction logic constants (swap HIGH/LOW if motor direction is reversed)
const uint8_t DIR_FORWARD = HIGH;
const uint8_t DIR_REVERSE = LOW;

// --- Encoder Interrupt Service Routines ---
void readEncoderA() {
  if (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B)) {
    encoderpos++;
  } else {
    encoderpos--;
  }
}
void readEncoderB() {
  if (digitalRead(ENCODER_PIN_A) != digitalRead(ENCODER_PIN_B)) {
    encoderpos++;
  } else {
    encoderpos--;
  }
}

// --- Helper Functions for Cytron MDD10A Control ---
void setMotor(MotorDIR dir, int speedPWM) {
  currentDIR = dir;
  speedPWM = constrain(speedPWM, 0, 255);

  if (dir == FORWARD) {
    digitalWrite(MOTOR_DIR, DIR_FORWARD);
    analogWrite(MOTOR_PWM, speedPWM);
  } else if (dir == REVERSE) {
    digitalWrite(MOTOR_DIR, DIR_REVERSE);
    analogWrite(MOTOR_PWM, speedPWM);
  } else { // STOP
    analogWrite(MOTOR_PWM, 0);
  }
}

void stopMotor() {
  setMotor(STOP, 0);
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
  Serial.println("Dual-Limit Cytron MDD10A Controller Ready.");
  Serial.println("Send 'F' (Forward), 'R' (Reverse), or 'S' (Stop) via Serial Monitor.");
}

void loop() {
  // --- 1. Check Dual Limit Switch States ---
  bool isRevLimitHit = (digitalRead(REVERSE_LIMIT_PIN) == LOW);
  bool isFwdLimitHit = (digitalRead(FORWARD_LIMIT_PIN) == LOW);

  // Turn on LED if either limit switch is triggered
  digitalWrite(LED_ARD, (isRevLimitHit || isFwdLimitHit) ? HIGH : LOW);

  // Home Switch Logic (Reverse Limit)
  if (isRevLimitHit) {
    noInterrupts();
    encoderpos = 0; // Reset home position
    interrupts();

    if (currentDIR == REVERSE) {
      stopMotor();
      Serial.println("WARNING: Reverse (Home) Limit Switch Triggered! Stopped.");
    }
  }

  // Far-End Limit Switch Logic (Forward Limit)
  if (isFwdLimitHit) {
    if (currentDIR == FORWARD) {
      stopMotor();
      Serial.println("WARNING: Forward Limit Switch Triggered! Stopped.");
    }
  }

  // --- 2. Process Serial Commands with Interlocks ---
  if (Serial.available() > 0) {
    char cmd = toupper(Serial.read());

    if (cmd == 'F') {
      if (isFwdLimitHit) {
        Serial.println("ERROR: Cannot move FORWARD - Forward limit switch is pressed!");
      } else {
        Serial.println("Moving FORWARD...");
        setMotor(FORWARD, 120); // Speed: 200/255
      }
    } else if (cmd == 'R') {
      if (isRevLimitHit) {
        Serial.println("ERROR: Cannot move REVERSE - Reverse limit switch is pressed!");
      } else {
        Serial.println("Moving REVERSE...");
        setMotor(REVERSE, 120);
      }
    } else if (cmd == 'S') {
      Serial.println("Stopping Motor.");
      stopMotor();
    }
  }

  // --- 3. Telemetry Output ---
  noInterrupts();
  long currentPosition = encoderpos;
  interrupts();

  static bool lastRevState = false;
  static bool lastFwdState = false;

  if (currentPosition != lastReportedPosition || isRevLimitHit != lastRevState || isFwdLimitHit != lastFwdState) {
    Serial.print("Position: ");
    Serial.print(currentPosition);
    Serial.print(" | REV Limit: ");
    Serial.print(isRevLimitHit ? "TRIGGERED" : "CLEAR");
    Serial.print(" | FWD Limit: ");
    Serial.print(isFwdLimitHit ? "TRIGGERED" : "CLEAR");
    Serial.print(" | Current Direction: ");
    Serial.println(currentDIR);
    lastReportedPosition = currentPosition;
    lastRevState = isRevLimitHit;
    lastFwdState = isFwdLimitHit;
  }

  delay(20);
}

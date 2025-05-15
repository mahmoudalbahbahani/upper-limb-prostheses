#include <Arduino.h>
#include "HX711.h"

// HX711 wiring
#define LOADCELL_DOUT_PIN 4
#define LOADCELL_SCK_PIN  5

// Pump and valve control
#define PUMP_PIN   12
#define VALVE_PIN  13

// Turn-off button pin
#define BUTTON_PIN 14  // Connect to a momentary push button

HX711 scale;

// Calibration and thresholds
float calibration_factor = -7050.0;
float currentForce = 0;
float previousForce = 0;

float threshold = 1.0;
float forceChangeThreshold = 0.5;  // Ignores small changes (fixes jitter)
float maxForceN = 20.0;

int minDurationMs = 100;
int maxDurationMs = 800;

bool useLogMapping = false;
float a = 1000;
float b = 0.5;



// State control
unsigned long actionStartTime = 0;
int actionDuration = 0;
enum ActionState { IDLE, INFLATING, DEFLATING };
ActionState currentState = IDLE;

void setup() {
  Serial.begin(115200);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare();

  pinMode(PUMP_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Button is active LOW

  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);

  // Initial full deflate
  Serial.println("System initializing...");
  digitalWrite(VALVE_PIN, HIGH);
  delay(1000);  // OK to block during setup
  digitalWrite(VALVE_PIN, LOW);
  delay(1000);
  Serial.println("Ready.");
}

int calculateDuration(float force) {
  int duration = 0;
  if (useLogMapping) {
    duration = a * log(b * force + 1);
  } else {
    duration = map(force, 0, maxForceN, minDurationMs, maxDurationMs);
  }
  return constrain(duration, minDurationMs, maxDurationMs);
}

void stopAll() {
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);
  currentState = IDLE;
}

void loop() {
  // 🛑 Safety Off Button
  if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("OFF button pressed → stopping system");
    stopAll();
    delay(1000);  // Debounce
    return;
  }

  // Read force
  currentForce = scale.get_units();
  currentForce = constrain(currentForce, 0, maxForceN);

  Serial.print("Force: ");
  Serial.print(currentForce);
  Serial.print(" N");

  // Check if an action is running (pump or valve)
  if (currentState == INFLATING || currentState == DEFLATING) {
    if (millis() - actionStartTime >= actionDuration) {
      stopAll();  // Stop pump or valve
      Serial.println(" → Action complete");
    }
    return;
  }

  // Only act if force > threshold
  if (currentForce > threshold) {
    float diff = currentForce - previousForce;

    if (abs(diff) > forceChangeThreshold) {
      if (diff > 0) {
        // Force increased → inflate
        actionDuration = calculateDuration(diff);
        digitalWrite(PUMP_PIN, HIGH);
        currentState = INFLATING;
        actionStartTime = millis();
        Serial.print(" → Inflating for ");
        Serial.print(actionDuration);
        Serial.println(" ms");
      } else {
        // Force decreased → deflate
        actionDuration = calculateDuration(-diff);
        digitalWrite(VALVE_PIN, HIGH);
        currentState = DEFLATING;
        actionStartTime = millis();
        Serial.print(" → Deflating for ");
        Serial.print(actionDuration);
        Serial.println(" ms");
      }
    } else {
      Serial.println(" → Small change, holding pressure");
    }
  } else {
    // Below threshold → full deflate for safety
    Serial.println(" → Low force, releasing pressure");
    digitalWrite(VALVE_PIN, HIGH);
    currentState = DEFLATING;
    actionDuration = 300;
    actionStartTime = millis();
  }

  previousForce = currentForce;
}




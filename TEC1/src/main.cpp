#include <Arduino.h>
#include <Wire.h>
#include <TSYS01.h>
#include <Adafruit_AHTX0.h>

// H-Bridge pins
#define IN1 10
#define IN2 11
#define ENA 9  // PWM

// Limits
#define HEAT_TARGET 40.0
#define COOL_TARGET 18.0
#define TEMP_TOLERANCE 0.5

#define PWM_MIN 0
#define PWM_MAX 255

TSYS01 tsys01;
Adafruit_AHTX0 aht20;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!tsys01.init()) {
    Serial.println("TSYS01 not detected!");
    while (1);
  }
  if (!aht20.begin()) {
    Serial.println("AHT20 not detected!");
    while (1);
  }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  ledcSetup(0, 5000, 8);        // Channel 0, 5kHz, 8-bit
  ledcAttachPin(ENA, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(0, 0);

  Serial.println("Thermal feedback system initialized.");
}

void loop() {
  // Read temperatures
  tsys01.read();
  float objectTemp = tsys01.temperature();

  sensors_event_t humidity, temp;
  aht20.getEvent(&humidity, &temp);
  float skinTemp = temp.temperature;

  Serial.print("Object: ");
  Serial.print(objectTemp);
  Serial.print(" °C | Skin: ");
  Serial.print(skinTemp);
  Serial.println(" °C");

  int pwmValue = 0;

  // Heating
  if (objectTemp >= HEAT_TARGET) {
    if (skinTemp < HEAT_TARGET - TEMP_TOLERANCE) {
      pwmValue = map(objectTemp, HEAT_TARGET, 50, PWM_MIN, PWM_MAX);
      pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      ledcWrite(0, pwmValue);
      Serial.print("Heating to max 40°C | PWM: ");
      Serial.println(pwmValue);
    } else {
      // Stop heating
      ledcWrite(0, 0);
      Serial.println("Heat limit reached (40°C) – stopping");
    }
  }
  // Cooling
  else if (objectTemp <= COOL_TARGET) {
    if (skinTemp > COOL_TARGET + TEMP_TOLERANCE) {
      pwmValue = map(objectTemp, 0, COOL_TARGET, PWM_MIN, PWM_MAX);
      pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      ledcWrite(0, pwmValue);
      Serial.print("Cooling to min 18°C | PWM: ");
      Serial.println(pwmValue);
    } else {
      // Stop cooling
      ledcWrite(0, 0);
      Serial.println("Cool limit reached (18°C) – stopping");
    }
  }
  // Neutral zone
  else {
    if (abs(objectTemp - skinTemp) > TEMP_TOLERANCE) {
      if (objectTemp > skinTemp) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        pwmValue = map(objectTemp - skinTemp, 0, 10, PWM_MIN, PWM_MAX);
        pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
        ledcWrite(0, pwmValue);
        Serial.print("Heating | PWM: ");
        Serial.println(pwmValue);
      } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        pwmValue = map(skinTemp - objectTemp, 0, 10, PWM_MIN, PWM_MAX);
        pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
        ledcWrite(0, pwmValue);
        Serial.print("Cooling | PWM: ");
        Serial.println(pwmValue);
      }
    } else {
      // Target matched
      ledcWrite(0, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      Serial.println("Skin matches object temp – idle");
    }
  }

  delay(500);  // Update rate
}



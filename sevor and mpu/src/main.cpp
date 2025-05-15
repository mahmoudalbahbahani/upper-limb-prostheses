#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <ESP32Servo.h>

MPU9250_asukiaaa imu;

Servo flexionServo;     // Controls elbow flexion (0–145°)
Servo supinationServo;  // Controls rotation (90° to -90° mapped to servo range)

float pitch_offset = 0;
float roll_offset = 0;
float yaw_offset = 0;
float yaw = 0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(11, 10);  // Your I2C pins (SDA, SCL)

  imu.setWire(&Wire);
  imu.beginAccel();
  imu.beginGyro();
  imu.beginMag();

  delay(2000);  // Allow MPU to stabilize

  imu.accelUpdate();
  imu.gyroUpdate();

  // Initial offsets
  pitch_offset = atan2(imu.accelX(), sqrt(imu.accelY() * imu.accelY() + imu.accelZ() * imu.accelZ())) * 180.0 / PI;
  roll_offset = atan2(-imu.accelY(), imu.accelZ()) * 180.0 / PI;
  lastTime = millis();

  Serial.println("Sensor zeroed. Starting...");

  // Attach servos to pins
  flexionServo.attach(5);     // Flexion/Extension
  supinationServo.attach(6);  // Supination/Pronation
}

void loop() {
  imu.accelUpdate();
  imu.gyroUpdate();

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float pitch = atan2(imu.accelX(), sqrt(imu.accelY() * imu.accelY() + imu.accelZ() * imu.accelZ())) * 180.0 / PI - pitch_offset;
  float roll  = atan2(-imu.accelY(), imu.accelZ()) * 180.0 / PI - roll_offset;

  yaw += imu.gyroZ() * dt;

  // === Flexion: Roll -> 0 to 145 degrees ===
  float flexionAngle = constrain(map(roll, 0, 145, 0, 180), 0, 180);
  flexionServo.write(flexionAngle);

  // === Supination/Pronation: Pitch -> 0 to 180 ===
  float supAngle = constrain(map(pitch, -90, 90, 0, 180), 0, 180);
  supinationServo.write(supAngle);

  // Debug output
  Serial.print("Roll: "); Serial.print(roll, 2); Serial.print("°, Servo: "); Serial.println(flexionAngle);
  Serial.print(" | Pitch: "); Serial.print(pitch, 2); Serial.print("°, Servo: "); Serial.println(supAngle);
  Serial.println("________________________________");



  delay(1000);
}

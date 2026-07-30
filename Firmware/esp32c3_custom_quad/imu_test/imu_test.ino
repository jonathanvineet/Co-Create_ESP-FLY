#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SDA_PIN 0
#define SCL_PIN 1

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  Serial.println("MPU6050 Found!");
}

void loop() {
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  Serial.print("Accel X: ");
  Serial.print(a.acceleration.x);
  Serial.print("  Y: ");
  Serial.print(a.acceleration.y);
  Serial.print("  Z: ");
  Serial.println(a.acceleration.z);

  Serial.print("Gyro X: ");
  Serial.print(g.gyro.x);
  Serial.print("  Y: ");
  Serial.print(g.gyro.y);
  Serial.print("  Z: ");
  Serial.println(g.gyro.z);

  Serial.print("Temp: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.println("----------------------");

  delay(500);
}
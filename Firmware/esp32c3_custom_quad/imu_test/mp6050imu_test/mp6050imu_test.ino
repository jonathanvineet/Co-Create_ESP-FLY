#include <Wire.h>

#define SDA_PIN 1
#define SCL_PIN 0
#define IMU_ADDR 0x68

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  if (Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)1) != 1)
    return 0xFF;

  return Wire.read();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Wake IMU
  writeReg(0x6B, 0x00);
  delay(100);

  uint8_t id = readReg(0x75);

  Serial.print("WHO_AM_I = 0x");
  Serial.println(id, HEX);

  if (id == 0x68)
    Serial.println("Detected MPU6050");
  else if (id == 0x70)
    Serial.println("Detected MPU6500");
  else {
    Serial.println("Unknown IMU!");
    while (1);
  }

  // Gyro ±250 dps
  writeReg(0x1B, 0x00);

  // Accel ±2g
  writeReg(0x1C, 0x00);

  // Low-pass filter
  writeReg(0x1A, 0x03);

  Serial.println("IMU Ready!");
}

void loop() {

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  if (Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)14) != 14) {
    Serial.println("Read Failed");
    delay(500);
    return;
  }

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  int16_t temp = (Wire.read() << 8) | Wire.read();

  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  Serial.println("----------------------");

  Serial.print("Accel X: ");
  Serial.print(ax / 16384.0);
  Serial.print(" g\tY: ");
  Serial.print(ay / 16384.0);
  Serial.print(" g\tZ: ");
  Serial.print(az / 16384.0);
  Serial.println(" g");

  Serial.print("Gyro X: ");
  Serial.print(gx / 131.0);
  Serial.print(" dps\tY: ");
  Serial.print(gy / 131.0);
  Serial.print(" dps\tZ: ");
  Serial.print(gz / 131.0);
  Serial.println(" dps");

  float temperature = (temp / 333.87) + 21.0;   // Correct for MPU6500

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  delay(250);
}
#include <Wire.h>
#include <string.h>

#define SDA_PIN 1
#define SCL_PIN 0

// Found by scanning at startup instead of being hardcoded. AD0 low = 0x68,
// AD0 high = 0x69, and the clone boards don't always strap it the same way.
uint8_t imuAddr = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.endTransmission(false);

  if (Wire.requestFrom((uint8_t)imuAddr, (uint8_t)1) != 1)
    return 0xFF;

  return Wire.read();
}

// Reads WHO_AM_I (0x75) from a specific address without touching imuAddr,
// so the scan can identify a candidate before committing to it.
uint8_t probeWhoAmI(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0)
    return 0xFF;

  if (Wire.requestFrom(addr, (uint8_t)1) != 1)
    return 0xFF;

  return Wire.read();
}

// Never returns NULL -- Serial.print() on a null char* dereferences it and
// panics, which on native-USB parts turns into a silent reset loop.
const char *imuName(uint8_t id) {
  switch (id) {
    case 0x68: return "MPU6050";
    case 0x70: return "MPU6500";
    case 0x71: return "MPU9250";
    case 0x73: return "MPU9255";
    case 0x74: return "MPU6555";
    case 0x75: return "MPU6515";
    case 0x98: return "MPU6050 clone";
    default:   return "unknown";
  }
}

bool isKnownImu(uint8_t id) {
  return id != 0xFF && id != 0x00 && strcmp(imuName(id), "unknown") != 0;
}

// Walks the whole 7-bit address range, prints everything that ACKs, and
// returns the first address whose WHO_AM_I looks like an MPU. Scanning the
// full bus (rather than just 0x68/0x69) also shows up any other device that
// might be contending for the bus.
uint8_t findImu() {
  uint8_t found = 0;

  Serial.println("Scanning I2C bus...");

  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0)
      continue;

    Serial.print("  device at 0x");
    Serial.print(addr, HEX);

    // Wake it before asking who it is -- a sleeping MPU still ACKs its
    // address but this keeps behaviour consistent with the init below.
    Wire.beginTransmission(addr);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    uint8_t id = probeWhoAmI(addr);

    Serial.print("  WHO_AM_I=0x");
    Serial.print(id, HEX);
    Serial.print(" -> ");
    Serial.print(imuName(id));

    if (isKnownImu(id) && !found) {
      found = addr;
      Serial.print("  [using this one]");
    }

    Serial.println();
  }

  return found;
}

// A bare `while (1);` starves the task watchdog, which panics and resets --
// on a native-USB part that becomes an invisible boot loop. Idle politely
// instead, and keep repeating why we stopped so it's visible whenever the
// serial monitor happens to connect.
void halt(const char *why) {
  while (1) {
    Serial.print("HALTED: ");
    Serial.println(why);
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000); // give native USB CDC time to enumerate before the first print

  Serial.println();
  Serial.println("=== MPU test booting ===");
  Serial.flush();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  imuAddr = findImu();

  if (imuAddr == 0)
    halt("no IMU found on the bus");

  Serial.print("Using IMU at 0x");
  Serial.println(imuAddr, HEX);

  // Wake IMU
  writeReg(0x6B, 0x00);
  delay(100);

  uint8_t id = readReg(0x75);

  Serial.print("WHO_AM_I = 0x");
  Serial.print(id, HEX);
  Serial.print("  (");
  Serial.print(imuName(id)); // safe: never NULL
  Serial.println(")");

  // Gyro ±250 dps
  writeReg(0x1B, 0x00);

  // Accel ±2g
  writeReg(0x1C, 0x00);

  // Low-pass filter
  writeReg(0x1A, 0x03);

  Serial.println("IMU Ready!");
}

void loop() {

  Wire.beginTransmission(imuAddr);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  if (Wire.requestFrom((uint8_t)imuAddr, (uint8_t)14) != 14) {
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
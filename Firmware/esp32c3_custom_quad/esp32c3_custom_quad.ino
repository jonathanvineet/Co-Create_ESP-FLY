/*
 * ESP32-C3 brushed micro-quad flight controller
 *
 * Hardware:
 *   - ESP32-C3 (single core, Arduino-ESP32 core)
 *   - MPU6050 (I2C, raw register access, no external IMU library)
 *   - 4x AO3400 logic-level N-MOSFET, low-side switching 4x 615 coreless motors
 *   - 4x SS14 schottky flyback diode across each motor
 *   - 4x gate resistor in series with each MOSFET gate
 *   - 1S 550mAh battery
 *
 * Control link: WiFi AP + UDP, 9-byte binary packet (see controller.py):
 *   uint8_t  armed
 *   int16_t  throttle  (0..1000)
 *   int16_t  pitch     (-500..500)
 *   int16_t  roll      (-500..500)
 *   int16_t  yaw       (-500..500)
 *   little-endian, matches Python struct format "<Bhhhh"
 *
 * SAFETY: remove propellers/motors from load, or restrain the frame, before
 * powering on for the first time. Verify motor direction and mixing signs
 * with props off before ever arming with props on.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Wire.h>
#include <string.h>

// ---------- Pin configuration ----------
// NOTE: GPIO3/GPIO4 were tried for I2C on this board and never ACKed
// (confirmed dead/unreliable on this specific ESP32-C3 SuperMini unit --
// root cause undetermined, possibly a bad header pin or internal fault).
// I2C was moved to GPIO0/GPIO1, confirmed working via the standalone
// imu_test sketch, chosen specifically so the existing motor wiring
// (FL=6, FR=7, BL=20, BR=10) did not need to be redone.
//
// NOTE: the standalone imu_test sketch has SDA/SCL the other way round
// (SDA=1, SCL=0) from what was originally hardcoded here, and that is the
// combination that actually works on the bench. Rather than pick one, the
// startup probe below tries both orders and both addresses and keeps
// whichever one ACKs -- so the same binary works on either wiring.
//
// NOTE: GPIO20 is the default UART0 RXD pin on most ESP32-C3 dev boards
// (used by the USB-serial bridge for flashing/Serial monitor). Driving a
// motor on GPIO20 can interfere with flashing/serial log while a motor is
// PWMing on it. Keep this in mind if you lose the ability to reflash.
// These now match the standalone imu_test sketch, which is the combination
// confirmed working on the bench. Order matters beyond correctness: the
// startup probe tries this pair FIRST, so on a good board it succeeds on
// the very first attempt and never has to Wire.end()/re-begin() -- tearing
// the bus down and back up does not always cleanly release GPIO0/GPIO1 on
// this core, which can leave the bus in a worse state than it started.
#define PIN_SDA        1
#define PIN_SCL        0
#define PIN_MOTOR_FL   6    // front-left
#define PIN_MOTOR_FR   7    // front-right
#define PIN_MOTOR_BL   20   // back-left
#define PIN_MOTOR_BR   10   // back-right
#define PIN_LED        8    // status LED (optional; GPIO8 is a strapping pin,
                             // only driven after boot completes so this is safe)

const int MOTOR_PINS[4] = {PIN_MOTOR_FL, PIN_MOTOR_FR, PIN_MOTOR_BL, PIN_MOTOR_BR};
const char *MOTOR_NAMES[4] = {"FL", "FR", "BL", "BR"};

// ---------- WiFi / UDP ----------
const char *AP_SSID = "esp32c3-quad";
const char *AP_PASS = "quadquad123"; // >= 8 chars required by WiFi AP
const uint16_t UDP_PORT = 4210;
WiFiUDP udp;
WebServer webServer(80);

// ---------- Control packet ----------
#pragma pack(push, 1)
struct ControlPacket {
  uint8_t armed;
  int16_t throttle; // 0..1000
  int16_t pitch;    // -500..500
  int16_t roll;     // -500..500
  int16_t yaw;      // -500..500
};
#pragma pack(pop)

volatile ControlPacket rxPacket = {0, 0, 0, 0, 0};
volatile unsigned long lastPacketMillis = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 300;

bool armed = false;
int imuFailStreak = 0;
const int IMU_FAIL_DISARM_THRESHOLD = 25; // ~100ms of consecutive failures at 250Hz
const int IMU_REINIT_INTERVAL = 250;      // retry bus recovery ~1s apart at 250Hz

// ---------- IMU: MPU6050 and MPU6500/6555/9250-family clones ----------
// The "MPU6050" modules being used are a mix of genuine MPU6050s and clone
// boards that actually report as an MPU6500/6555. Both are register
// compatible for everything this flight controller touches (power
// management, sample rate, DLPF, gyro/accel full-scale, and the 14-byte
// burst starting at ACCEL_XOUT_H), so the only thing that has to adapt is
// the identification and the temperature conversion -- and temperature is
// unused here. So: probe, accept the whole family, otherwise identical.
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_WHO_AM_I     0x75

const float ACCEL_SCALE = 16384.0f; // LSB/g at +-2g
const float GYRO_SCALE  = 131.0f;   // LSB/(deg/s) at +-250deg/s

// Resolved at boot by imuProbe(). AD0 low = 0x68, AD0 high = 0x69.
uint8_t mpuAddr = 0x68;
uint8_t mpuWhoAmI = 0x00;
int imuSdaPin = PIN_SDA;
int imuSclPin = PIN_SCL;
bool imuPresent = false;

float pitchAngle = 0.0f, rollAngle = 0.0f;
float gyroXoff = 0, gyroYoff = 0, gyroZoff = 0;

// WHO_AM_I values seen across the genuine part and the clones that ship on
// "MPU6050" breakout boards. All are register compatible for our usage.
const char *imuNameFor(uint8_t id) {
  switch (id) {
    case 0x68: return "MPU6050";
    case 0x69: return "MPU6050 (alt id)";
    case 0x70: return "MPU6500";
    case 0x71: return "MPU9250";
    case 0x73: return "MPU9255";
    case 0x74: return "MPU6555";
    case 0x75: return "MPU6515";
    case 0x98: return "MPU6050 clone";
    default:   return NULL;
  }
}

void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Returns 0xFF on a failed read (0xFF is not a valid WHO_AM_I for this family).
uint8_t mpuReadReg(uint8_t reg) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom((uint8_t)mpuAddr, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

void mpuInit() {
  mpuWriteReg(REG_PWR_MGMT_1, 0x00);   // wake up
  delay(100);
  mpuWriteReg(REG_SMPLRT_DIV, 0x04);   // 1kHz / (1+4) = 200Hz sample rate
  mpuWriteReg(REG_CONFIG, 0x03);       // DLPF ~44Hz bandwidth
  mpuWriteReg(REG_GYRO_CONFIG, 0x00);  // +-250 deg/s
  mpuWriteReg(REG_ACCEL_CONFIG, 0x00); // +-2g
}

// Try one (SDA, SCL, address) combination: bring the bus up, wake the part,
// and see whether WHO_AM_I comes back as something we recognise.
bool imuTryBus(int sda, int scl, uint8_t addr) {
  Wire.end();
  Wire.begin(sda, scl);
  Wire.setClock(100000); // lower clock for noise immunity against motor PWM
  delay(10);

  mpuAddr = addr;
  mpuWriteReg(REG_PWR_MGMT_1, 0x00); // wake before reading WHO_AM_I
  delay(100);

  uint8_t id = mpuReadReg(REG_WHO_AM_I);
  if (imuNameFor(id) == NULL) return false;

  imuSdaPin = sda;
  imuSclPin = scl;
  mpuWhoAmI = id;
  return true;
}

// Probe both pin orders and both addresses. The wiring on these boards has
// been ambiguous (see the pin notes above) and the clones sometimes sit on
// 0x69, so trying all four combinations is cheap insurance at boot.
bool imuProbe() {
  const int pinOrders[2][2] = {{PIN_SDA, PIN_SCL}, {PIN_SCL, PIN_SDA}};
  const uint8_t addrs[2] = {0x68, 0x69};

  for (int p = 0; p < 2; p++) {
    for (int a = 0; a < 2; a++) {
      if (imuTryBus(pinOrders[p][0], pinOrders[p][1], addrs[a])) {
        Serial.print("IMU found: ");
        Serial.print(imuNameFor(mpuWhoAmI));
        Serial.print(" (WHO_AM_I=0x");
        Serial.print(mpuWhoAmI, HEX);
        Serial.print(") addr=0x");
        Serial.print(mpuAddr, HEX);
        Serial.print(" SDA=");
        Serial.print(imuSdaPin);
        Serial.print(" SCL=");
        Serial.println(imuSclPin);
        return true;
      }
    }
  }

  Serial.println("IMU NOT FOUND on either pin order / address. "
                  "Arming will be refused.");
  return false;
}

// Bus recovery: re-run the full init on the already-known good pins/address.
// Used when the IMU stops responding mid-flight-loop (motor PWM noise can
// wedge the bus), so the link can come back without a power cycle.
void imuReinit() {
  Wire.end();
  Wire.begin(imuSdaPin, imuSclPin);
  Wire.setClock(100000);
  delay(10);
  mpuInit();
}

// Motor PWM switching noise on the I2C bus can corrupt an occasional
// transaction. Retry a few times before giving up -- a clean retry is much
// cheaper than a false disarm.
const int MPU_READ_RETRIES = 3;

// Last failure detail, surfaced in the [DBG] line so a failing bus can be
// diagnosed from the serial monitor instead of guessed at.
//   lastTxErr: return of endTransmission() -- 0 ok, 1 data too long,
//              2 NACK on address, 3 NACK on data, 4 other, 5 timeout.
//              2 = nothing is answering at that address (wiring/pullups).
//              5 = bus wedged, slave holding SDA (noise / interrupted xfer).
//   lastRxCount: bytes actually returned by requestFrom() (want 14).
uint8_t lastTxErr = 0;
int lastRxCount = 0;

bool mpuReadRawOnce(int16_t &ax, int16_t &ay, int16_t &az,
                     int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(REG_ACCEL_XOUT_H);
  lastTxErr = Wire.endTransmission(false);
  if (lastTxErr != 0) return false;
  lastRxCount = Wire.requestFrom((uint8_t)mpuAddr, (uint8_t)14);
  if (lastRxCount != 14) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temperature, unused
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  for (int attempt = 0; attempt < MPU_READ_RETRIES; attempt++) {
    if (mpuReadRawOnce(ax, ay, az, gx, gy, gz)) return true;
  }
  return false;
}

void mpuCalibrateGyro() {
  const int N = 500;
  long sx = 0, sy = 0, sz = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < N; i++) {
    if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
      sx += gx; sy += gy; sz += gz;
    }
    delay(3);
  }
  gyroXoff = sx / (float)N;
  gyroYoff = sy / (float)N;
  gyroZoff = sz / (float)N;
}

// ---------- Simple PID ----------
struct PID {
  float kp, ki, kd;
  float integral = 0, prevError = 0;
  float integralLimit, outputLimit;

  float update(float error, float dt) {
    integral += error * dt;
    integral = constrain(integral, -integralLimit, integralLimit);
    float derivative = (error - prevError) / dt;
    prevError = error;
    float out = kp * error + ki * integral + kd * derivative;
    return constrain(out, -outputLimit, outputLimit);
  }

  void reset() { integral = 0; prevError = 0; }
};

// Angle-mode stabilization for pitch/roll, rate-mode for yaw.
// Start conservative; tune kp first, then kd, then ki, with props off.
PID pitchPID = {3.0f, 0.02f, 0.8f, 0, 0, 200.0f, 400.0f};
PID rollPID  = {3.0f, 0.02f, 0.8f, 0, 0, 200.0f, 400.0f};
PID yawPID   = {2.0f, 0.0f,  0.0f, 0, 0, 200.0f, 300.0f};

// ---------- Motor output ----------
const int PWM_FREQ = 20000; // 20kHz, above audible whine
const int PWM_RES  = 10;    // 10-bit duty, 0..1023
const int PWM_MAX_DUTY = (1 << PWM_RES) - 1;

void motorsInit() {
  for (int i = 0; i < 4; i++) {
    ledcAttach(MOTOR_PINS[i], PWM_FREQ, PWM_RES);
  }
}

// Writes one duty value per motor, indexed the same as MOTOR_PINS/MOTOR_NAMES.
void motorsWriteArr(float m[4]) {
  for (int i = 0; i < 4; i++) {
    float v = constrain(m[i], 0, 1000);
    ledcWrite(MOTOR_PINS[i], (int)(v * PWM_MAX_DUTY / 1000));
  }
}

void motorsWrite(float fl, float fr, float bl, float br) {
  float m[4] = {fl, fr, bl, br};
  motorsWriteArr(m);
}

void motorsStop() {
  motorsWrite(0, 0, 0, 0);
}

// ---------- Interactive serial motor test mode ----------
// These are single low-side MOSFET switched brushed motors: there is no
// electrical reverse. "Forward" ramps the selected motor's speed up,
// "backward" ramps it back down toward 0 -- this is for confirming wiring,
// motor position, and spin direction on the bench, not direction reversal.
//
// Serial commands (type a single character + Enter, or just the char if
// your terminal sends immediately):
//   t        toggle test mode on/off (only allowed while disarmed)
//   1..4     select motor (1=FL 2=FR 3=BL 4=BR)
//   f        forward: ramp selected motor's throttle up by TEST_STEP
//   b        backward: ramp selected motor's throttle down by TEST_STEP
//   s        stop selected motor (set it to 0)
//   a        stop all motors (all to 0)
//   ?        print current status
bool testMode = false;
int testSelected = 0; // index 0..3
float testThrottle[4] = {0, 0, 0, 0};
const float TEST_STEP = 50.0f;

void printTestStatus() {
  Serial.print("[TEST] selected=");
  Serial.print(MOTOR_NAMES[testSelected]);
  Serial.print("  FL=");
  Serial.print(testThrottle[0]);
  Serial.print(" FR=");
  Serial.print(testThrottle[1]);
  Serial.print(" BL=");
  Serial.print(testThrottle[2]);
  Serial.print(" BR=");
  Serial.println(testThrottle[3]);
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;

    if (c == 't') {
      if (!armed) {
        testMode = !testMode;
        if (testMode) {
          for (int i = 0; i < 4; i++) testThrottle[i] = 0;
          motorsStop();
          Serial.println("[TEST] test mode ON (flight control disabled). "
                          "1-4 select motor, f/b ramp up/down, s stop, a stop all, t exit.");
        } else {
          motorsStop();
          Serial.println("[TEST] test mode OFF, back to normal flight control.");
        }
      } else {
        Serial.println("[TEST] cannot toggle test mode while armed.");
      }
      continue;
    }

    if (!testMode) continue; // ignore other test commands unless in test mode

    switch (c) {
      case '1': case '2': case '3': case '4':
        testSelected = (c - '1');
        printTestStatus();
        break;
      case 'f':
        testThrottle[testSelected] = constrain(testThrottle[testSelected] + TEST_STEP, 0, 1000);
        motorsWriteArr(testThrottle);
        printTestStatus();
        break;
      case 'b':
        testThrottle[testSelected] = constrain(testThrottle[testSelected] - TEST_STEP, 0, 1000);
        motorsWriteArr(testThrottle);
        printTestStatus();
        break;
      case 's':
        testThrottle[testSelected] = 0;
        motorsWriteArr(testThrottle);
        printTestStatus();
        break;
      case 'a':
        for (int i = 0; i < 4; i++) testThrottle[i] = 0;
        motorsWriteArr(testThrottle);
        printTestStatus();
        break;
      case '?':
        printTestStatus();
        break;
      default:
        break;
    }
  }
}

// ---------- Setup ----------
// Bisect switch. The standalone imu_test sketch reads this same IMU
// flawlessly, and the two big differences here are (a) WiFi AP + web server
// running on this single-core chip and (b) a 250Hz read rate instead of 4Hz.
// Set this to 1 to bring the radio up as normal; set it to 0 to build a
// no-radio version that is otherwise identical to the flight firmware.
//
// If the IMU loop is solid with this at 0 and fails at 1, the fault is
// WiFi/loop-timing contention on the I2C bus, not the IMU or the wiring.
// Flight control is unusable with the radio off (no link => never arms),
// so this is a bench diagnostic only -- leave it at 1 to fly.
#define ENABLE_WIFI 1

unsigned long lastLoopMicros = 0;
const unsigned long LOOP_PERIOD_US = 4000; // 250Hz control loop

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  motorsInit();
  motorsStop();

  // Probe leaves the bus configured on whichever pins/address answered.
  imuPresent = imuProbe();
  if (imuPresent) {
    mpuInit();
    Serial.println("Calibrating gyro, keep the frame still...");
    mpuCalibrateGyro();
    Serial.println("Gyro calibration done.");
  } else {
    // Keep booting so the WiFi/web UI and the serial motor test mode are
    // still reachable for debugging -- but arming stays blocked below.
    Serial.println("Skipping gyro calibration (no IMU). "
                    "Serial motor test mode still available.");
  }

#if ENABLE_WIFI
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  udp.begin(UDP_PORT);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("UDP port: ");
  Serial.println(UDP_PORT);

  webServer.on("/", handleWebRoot);
  webServer.on("/control", handleWebControl);
  webServer.begin();
#else
  WiFi.mode(WIFI_OFF);
  Serial.println("*** ENABLE_WIFI=0: radio OFF, IMU-diagnostic build. ***");
  Serial.println("*** No control link -- will never arm. Bench use only. ***");
  Serial.println("Web control UI: http://192.168.4.1/");
#endif

  lastLoopMicros = micros();
  lastPacketMillis = millis();
}

// Shared by both the UDP path and the HTTP/web-UI path: writes a new
// control packet into rxPacket and marks the link as alive.
void applyControlPacket(const ControlPacket &pkt) {
  noInterrupts();
  memcpy((void *)&rxPacket, &pkt, sizeof(pkt));
  interrupts();
  lastPacketMillis = millis();
}

// ---------- UDP receive ----------
void pollUdp() {
  int packetSize = udp.parsePacket();
  if (packetSize == sizeof(ControlPacket)) {
    ControlPacket pkt;
    udp.read((uint8_t *)&pkt, sizeof(pkt));
    applyControlPacket(pkt);
  } else if (packetSize > 0) {
    udp.flush(); // discard malformed packet
  }
}

// ---------- Web control UI ----------
// Self-contained page (no external CDN/network calls -- the AP has no
// internet route, so any external <script src="..."> would just fail to
// load). Two touch joysticks, Mode 2 layout: left stick = throttle/yaw,
// right stick = pitch/roll. Both spring back to center (0) on release,
// including throttle -- safer default for a touchscreen with no physical
// detents than a throttle that holds its last value.
const char *INDEX_HTML = R"HTMLPAGE(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>ESP32-C3 Quad Control</title>
<style>
  html, body {
    margin: 0; padding: 0; height: 100%; background: #111; color: #eee;
    font-family: -apple-system, sans-serif; overflow: hidden; touch-action: none;
    user-select: none; -webkit-user-select: none;
  }
  #topbar {
    position: fixed; top: 0; left: 0; right: 0; padding: 8px 12px;
    display: flex; justify-content: space-between; align-items: center;
    font-size: 14px; z-index: 10;
  }
  #status { color: #4caf50; }
  #status.bad { color: #f44336; }
  #armBtn {
    padding: 10px 18px; font-size: 16px; font-weight: bold; border: none;
    border-radius: 8px; background: #444; color: #eee;
  }
  #armBtn.armed { background: #d32f2f; }
  #stage {
    position: absolute; top: 0; left: 0; width: 100%; height: 100%;
    display: flex; justify-content: space-between; align-items: center;
    padding: 0 20px; box-sizing: border-box;
  }
  .stick-base {
    width: 42vw; height: 42vw; max-width: 220px; max-height: 220px;
    border-radius: 50%; background: #222; border: 2px solid #444;
    position: relative;
  }
  .stick-knob {
    width: 40%; height: 40%; border-radius: 50%; background: #666;
    position: absolute; top: 30%; left: 30%;
    box-shadow: 0 0 8px rgba(255,255,255,0.15);
  }
  #readout {
    position: fixed; bottom: 6px; left: 0; right: 0; text-align: center;
    font-size: 12px; color: #888;
  }
</style>
</head>
<body>
<div id="topbar">
  <span id="status">link ok</span>
  <button id="armBtn">ARM</button>
</div>
<div id="stage">
  <div class="stick-base" id="leftBase"><div class="stick-knob" id="leftKnob"></div></div>
  <div class="stick-base" id="rightBase"><div class="stick-knob" id="rightKnob"></div></div>
</div>
<div id="readout">t=0 p=0 r=0 y=0</div>
<script>
var armed = false;
var throttle = 0, yaw = 0, pitch = 0, roll = 0;
var lastAckMs = 0;

function makeStick(baseEl, knobEl, onMove) {
  var touchId = null;
  var cx = 0, cy = 0, radius = 1;
  function start(x, y) {
    var rect = baseEl.getBoundingClientRect();
    cx = rect.left + rect.width / 2;
    cy = rect.top + rect.height / 2;
    radius = rect.width / 2;
  }
  function move(x, y) {
    var dx = x - cx, dy = y - cy;
    var dist = Math.sqrt(dx * dx + dy * dy);
    if (dist > radius) { dx = dx * radius / dist; dy = dy * radius / dist; }
    knobEl.style.left = (30 + (dx / radius) * 30) + "%";
    knobEl.style.top  = (30 + (dy / radius) * 30) + "%";
    onMove(dx / radius, dy / radius);
  }
  function reset() {
    knobEl.style.left = "30%";
    knobEl.style.top = "30%";
    onMove(0, 0);
  }
  baseEl.addEventListener("touchstart", function(e) {
    e.preventDefault();
    var t = e.changedTouches[0];
    touchId = t.identifier;
    start(t.clientX, t.clientY);
    move(t.clientX, t.clientY);
  }, { passive: false });
  baseEl.addEventListener("touchmove", function(e) {
    e.preventDefault();
    for (var i = 0; i < e.changedTouches.length; i++) {
      var t = e.changedTouches[i];
      if (t.identifier === touchId) move(t.clientX, t.clientY);
    }
  }, { passive: false });
  baseEl.addEventListener("touchend", function(e) {
    e.preventDefault();
    for (var i = 0; i < e.changedTouches.length; i++) {
      if (e.changedTouches[i].identifier === touchId) { touchId = null; reset(); }
    }
  }, { passive: false });
}

// Left stick: y -> throttle (0..1000, up = more), x -> yaw (-500..500)
makeStick(document.getElementById("leftBase"), document.getElementById("leftKnob"),
  function(nx, ny) {
    throttle = Math.round(Math.max(0, -ny) * 1000);
    yaw = Math.round(nx * 500);
  });

// Right stick: y -> pitch (-500..500, up = forward), x -> roll (-500..500)
makeStick(document.getElementById("rightBase"), document.getElementById("rightKnob"),
  function(nx, ny) {
    pitch = Math.round(-ny * 500);
    roll = Math.round(nx * 500);
  });

document.getElementById("armBtn").addEventListener("click", function() {
  armed = !armed;
  this.classList.toggle("armed", armed);
  this.textContent = armed ? "DISARM" : "ARM";
});

function sendControl() {
  var url = "/control?armed=" + (armed ? 1 : 0) +
            "&throttle=" + throttle + "&pitch=" + pitch +
            "&roll=" + roll + "&yaw=" + yaw;
  fetch(url).then(function(r) {
    lastAckMs = Date.now();
  }).catch(function() {});

  var statusEl = document.getElementById("status");
  if (Date.now() - lastAckMs > 500) {
    statusEl.textContent = "no link";
    statusEl.classList.add("bad");
  } else {
    statusEl.textContent = "link ok";
    statusEl.classList.remove("bad");
  }
  document.getElementById("readout").textContent =
    "t=" + throttle + " p=" + pitch + " r=" + roll + " y=" + yaw;
}
setInterval(sendControl, 50); // 20Hz
</script>
</body>
</html>
)HTMLPAGE";

void handleWebRoot() {
  webServer.send(200, "text/html", INDEX_HTML);
}

void handleWebControl() {
  ControlPacket pkt;
  pkt.armed = webServer.arg("armed").toInt() != 0 ? 1 : 0;
  pkt.throttle = constrain(webServer.arg("throttle").toInt(), 0, 1000);
  pkt.pitch = constrain(webServer.arg("pitch").toInt(), -500, 500);
  pkt.roll = constrain(webServer.arg("roll").toInt(), -500, 500);
  pkt.yaw = constrain(webServer.arg("yaw").toInt(), -500, 500);
  applyControlPacket(pkt);
  webServer.send(200, "text/plain", "ok");
}

// ---------- Main loop ----------
void loop() {
#if ENABLE_WIFI
  pollUdp();
  webServer.handleClient();
#endif
  handleSerialCommands();

  unsigned long nowUs = micros();
  if (nowUs - lastLoopMicros < LOOP_PERIOD_US) return;
  float dt = (nowUs - lastLoopMicros) / 1000000.0f;
  lastLoopMicros = nowUs;

  if (testMode) {
    // Test mode owns the motors directly via handleSerialCommands();
    // flight control, arming, and the IMU-driven mixer are all skipped.
    armed = false;
    digitalWrite(PIN_LED, LOW);
    return;
  }

  bool linkOk = (millis() - lastPacketMillis) < FAILSAFE_TIMEOUT_MS;

  ControlPacket pkt;
  noInterrupts();
  memcpy(&pkt, (const void *)&rxPacket, sizeof(pkt));
  interrupts();

  // Level-triggered arm: becomes armed the moment the armed flag is set
  // AND throttle is observed low, whenever that happens (not just on the
  // exact 0->1 transition instant, which is prone to missing the window
  // if throttle has already ramped up by the time packets start arriving,
  // e.g. right after boot). Still can't arm at high throttle.
  bool armedFlag = linkOk && imuPresent && (pkt.armed != 0);
  if (armedFlag && !armed && pkt.throttle < 50) {
    armed = true;
    pitchPID.reset();
    rollPID.reset();
    yawPID.reset();
  } else if (!armedFlag) {
    armed = false;
  }

  if (!linkOk) {
    armed = false;
  }

  digitalWrite(PIN_LED, armed ? HIGH : LOW);

  // --- Debug: print link/arm/packet state a few times a second ---
  static unsigned long lastDebugMs = 0;
  if (millis() - lastDebugMs > 200) {
    lastDebugMs = millis();
    Serial.print("[DBG] linkOk="); Serial.print(linkOk);
    Serial.print(" armedFlag=");   Serial.print(armedFlag);
    Serial.print(" armed=");       Serial.print(armed);
    Serial.print(" pkt.armed=");   Serial.print(pkt.armed);
    Serial.print(" throttle=");    Serial.print(pkt.throttle);
    Serial.print(" msSinceLastPkt="); Serial.print(millis() - lastPacketMillis);
    Serial.print(" imuFailStreak=");  Serial.print(imuFailStreak);
    Serial.print(" txErr=");          Serial.print(lastTxErr);
    Serial.print(" rxCount=");        Serial.println(lastRxCount);
  }

  // --- IMU update ---
  int16_t ax, ay, az, gx, gy, gz;
  if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
    imuFailStreak = 0;
    float gxDeg = (gx - gyroXoff) / GYRO_SCALE;
    float gyDeg = (gy - gyroYoff) / GYRO_SCALE;
    float gzDeg = (gz - gyroZoff) / GYRO_SCALE;

    float axg = ax / ACCEL_SCALE;
    float ayg = ay / ACCEL_SCALE;
    float azg = az / ACCEL_SCALE;

    float accelPitch = atan2(-axg, sqrt(ayg * ayg + azg * azg)) * 180.0f / PI;
    float accelRoll  = atan2(ayg, azg) * 180.0f / PI;

    pitchAngle = 0.98f * (pitchAngle + gyDeg * dt) + 0.02f * accelPitch;
    rollAngle  = 0.98f * (rollAngle + gxDeg * dt) + 0.02f * accelRoll;

    if (!armed) {
      motorsStop();
    } else {
      float throttle = pkt.throttle;
      float pitchSetpoint = pkt.pitch * (30.0f / 500.0f); // +-30 deg max
      float rollSetpoint  = pkt.roll  * (30.0f / 500.0f);
      float yawRateCmd    = pkt.yaw;                       // rate command

      float pitchOut = pitchPID.update(pitchSetpoint - pitchAngle, dt);
      float rollOut  = rollPID.update(rollSetpoint - rollAngle, dt);
      float yawOut   = yawPID.update(yawRateCmd - gzDeg, dt);

      // Standard X mixing.
      float fl = throttle + pitchOut - rollOut - yawOut;
      float fr = throttle + pitchOut + rollOut + yawOut;
      float bl = throttle - pitchOut - rollOut + yawOut;
      float br = throttle - pitchOut + rollOut - yawOut;

      // Only allow spin-up once throttle is above a small deadband,
      // so idle stick with props on doesn't twitch motors.
      if (throttle < 20) {
        motorsStop();
      } else {
        motorsWrite(fl, fr, bl, br);
      }
    }
  } else {
    // IMU read failed. A single dropped I2C transaction (e.g. from WiFi
    // background activity briefly delaying the loop on this single-core
    // chip) shouldn't fully disarm -- only disarm after several
    // consecutive failures, which indicates a real, sustained IMU loss.
    imuFailStreak++;
    if (imuFailStreak >= IMU_FAIL_DISARM_THRESHOLD) {
      armed = false;
      // Sustained loss: the bus itself is likely wedged (motor PWM noise can
      // leave a slave mid-transaction holding SDA). Re-init periodically so
      // it can recover without a power cycle, rather than failing forever.
      if (imuFailStreak % IMU_REINIT_INTERVAL == 0) {
        Serial.println("[IMU] sustained read failure, re-initialising bus...");
        imuReinit();
      }
    }
    motorsStop(); // always safe to skip motor output for this one cycle
  }
}

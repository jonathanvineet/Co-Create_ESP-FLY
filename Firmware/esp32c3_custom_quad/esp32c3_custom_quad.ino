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
// NOTE: GPIO20 is the default UART0 RXD pin on most ESP32-C3 dev boards
// (used by the USB-serial bridge for flashing/Serial monitor). Driving a
// motor on GPIO20 can interfere with flashing/serial log while a motor is
// PWMing on it. Keep this in mind if you lose the ability to reflash.
#define PIN_SDA        0
#define PIN_SCL        1
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

// ---------- MPU6050 ----------
#define MPU_ADDR 0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

const float ACCEL_SCALE = 16384.0f; // LSB/g at +-2g
const float GYRO_SCALE  = 131.0f;   // LSB/(deg/s) at +-250deg/s

float pitchAngle = 0.0f, rollAngle = 0.0f;
float gyroXoff = 0, gyroYoff = 0, gyroZoff = 0;

void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  mpuWriteReg(REG_PWR_MGMT_1, 0x00);   // wake up
  delay(100);
  mpuWriteReg(REG_SMPLRT_DIV, 0x04);   // 1kHz / (1+4) = 200Hz sample rate
  mpuWriteReg(REG_CONFIG, 0x03);       // DLPF ~44Hz bandwidth
  mpuWriteReg(REG_GYRO_CONFIG, 0x00);  // +-250 deg/s
  mpuWriteReg(REG_ACCEL_CONFIG, 0x00); // +-2g
}

// Motor PWM switching noise on the I2C bus can corrupt an occasional
// transaction. Retry a few times before giving up -- a clean retry is much
// cheaper than a false disarm.
const int MPU_READ_RETRIES = 3;

bool mpuReadRawOnce(int16_t &ax, int16_t &ay, int16_t &az,
                     int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;

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
unsigned long lastLoopMicros = 0;
const unsigned long LOOP_PERIOD_US = 4000; // 250Hz control loop

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  motorsInit();
  motorsStop();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000); // lower clock for noise immunity against motor PWM switching
  mpuInit();
  Serial.println("Calibrating gyro, keep the frame still...");
  mpuCalibrateGyro();
  Serial.println("Gyro calibration done.");

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
  Serial.println("Web control UI: http://192.168.4.1/");

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
  pollUdp();
  webServer.handleClient();
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
  bool armedFlag = linkOk && (pkt.armed != 0);
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
    Serial.print(" imuFailStreak=");  Serial.println(imuFailStreak);
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
    }
    motorsStop(); // always safe to skip motor output for this one cycle
  }
}

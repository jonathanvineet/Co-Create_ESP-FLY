# ESP32-C3 Custom Brushed Micro-Quad Firmware

Custom firmware for a tiny brushed quadcopter built from:

- 1x ESP32-C3 (single-core RISC-V dev board)
- 1x MPU6050 (accel + gyro, I2C)
- 4x AO3400 logic-level N-MOSFET (low-side motor switch)
- 4x SS14 schottky diode (flyback protection, one per motor)
- 4x resistor (gate resistor, one per MOSFET, ~100-220 ohm)
- 4x 615 coreless motor
- 1x 1S 550mAh battery

This is a from-scratch Arduino-ESP32 sketch, not a port of the `esp-drone`
project in this repo — that codebase assumes dual-core Xtensa FreeRTOS,
which doesn't map onto the single-core C3.

## IMU test sketch (`imu_test/imu_test.ino`)

A standalone sketch, separate from the flight firmware, for verifying the
MPU6050 in isolation. Flash it by itself (it's its own sketch folder, so
Arduino IDE/arduino-cli will treat it as an independent project). It:

1. Scans the I2C bus and lists every address that ACKs.
2. Reads the `WHO_AM_I` register and checks it equals `0x68` (the standard
   MPU6050 value).
3. Streams live accel (g) and gyro (deg/s) readings at ~5Hz, printing how
   many **microseconds** each I2C transaction took.

Use this if the main firmware's gyro calibration is taking way longer
than the expected ~1.5s (500 samples at ~3ms each) — that symptom means
individual I2C transactions are stalling, and this sketch shows you the
per-read timing directly. On a healthy bus, reads should take well under
1000us; tens of milliseconds per read (or an outright "no ACK" on the scan)
points to missing pull-ups, wrong VCC (3.3V vs 5V), a loose SDA/SCL/GND
wire, or an address conflict with another I2C device on the bus.

```
arduino-cli compile --fqbn esp32:esp32:esp32c3 imu_test/imu_test.ino
arduino-cli upload  --fqbn esp32:esp32:esp32c3 -p /dev/tty.usbXXXX imu_test/imu_test.ino
```

Remember to reflash the main `esp32c3_custom_quad.ino` afterward once the
IMU checks out — this test sketch doesn't drive motors or WiFi at all.

## Wiring

MOSFET low-side switch per motor:

```
BAT+ ---- motor ---- MOSFET drain
                       MOSFET source ---- BAT- (GND)
                       MOSFET gate ---- gate resistor ---- ESP32-C3 GPIO
SS14 diode: cathode to BAT+ side (motor+), anode to drain
            (across the motor, protects the MOSFET from inductive kickback)
```

GPIO assignment (edit at the top of the .ino if your board's pins differ):

| Signal      | GPIO |
|-------------|------|
| Motor FL    | 6    |
| Motor FR    | 7    |
| Motor BL    | 20   |
| Motor BR    | 10   |
| I2C SDA     | 0    |
| I2C SCL     | 1    |
| Status LED  | 8    |

Notes on these pins:

- **GPIO3/GPIO4 are intentionally unused.** They were the original I2C
  pins but never ACKed on this specific ESP32-C3 SuperMini unit even with
  confirmed-good VCC/GND/wiring — root cause undetermined (possibly a bad
  header pin on this board). I2C was moved to GPIO0/GPIO1 instead
  (confirmed working via the standalone `imu_test` sketch), specifically
  chosen so the motor wiring (already built on GPIO6/7/20/10) didn't need
  to be redone. If you're building this on a different or known-good
  board, GPIO3/4 should work fine — just re-verify with `imu_test` first.
- **GPIO20** is the default UART0 RXD pin on most ESP32-C3 dev boards
  (used by the onboard USB-serial bridge for flashing and `Serial`
  monitor). Driving motor BL's PWM on it works, but if you ever lose the
  ability to reflash/see serial output, this is the first pin to suspect —
  move motor BL to a free GPIO if that happens.
- **GPIO8** is a strapping pin (sampled only at reset to select boot
  mode). It's safe to drive as the status LED after boot completes, but
  don't wire anything to it that could pull it low externally before/during
  power-up.
- GPIO2 and GPIO9 (the other two C3 strapping pins) are left unused —
  don't repurpose them for motors/IMU without checking boot behavior.
- GPIO18/GPIO19 are avoided entirely — on ESP32-C3 these are the dedicated
  USB D-/D+ pins for the native USB-Serial-JTAG peripheral used for
  flashing/Serial monitor over the board's USB port.

Motor layout (X configuration, viewed from above, front = FL/FR side):

```
   FL (CCW)      FR (CW)
        \        /
         [ ESP32 ]
        /        \
   BL (CW)       BR (CCW)
```

Motor spin directions above are the conventional layout; wire your motors
so diagonal pairs (FL/BR, FR/BL) spin the same direction. If yaw response
is backwards after your first bench test, swap the spin direction of one
diagonal pair or flip the yaw mixing signs in the sketch.

## Firmware behavior

- Boots into WiFi AP mode: SSID `esp32c3-quad`, password `quadquad123`,
  drone IP `192.168.4.1`. Accepts control input from two independent
  sources at once, both feeding the same internal state:
  - UDP control packets on port 4210 (`controller.py`).
  - A built-in web page at `http://192.168.4.1/` with touch joysticks
    (see "Mobile web control UI" below) — no app install needed.
- Calibrates the gyro at boot — keep the frame still for ~1.5s after
  power-up (blocking, before you see the AP come up).
- Complementary filter fuses accel + gyro into pitch/roll angle estimate.
- Angle-mode PID stabilizes pitch/roll to the commanded stick angle
  (±30° max). Yaw is rate-controlled directly from gyro Z.
- Standard X-quad mixing to 4 independent PWM channels (LEDC, 20kHz,
  10-bit duty) driving the MOSFET gates.
- Arming is level-triggered: becomes armed as soon as the link is alive,
  the `armed` flag is set, AND throttle is observed below 50 -- at any
  point that condition holds, not just on one exact transition instant.
  Once armed, stays armed regardless of throttle until disarmed.
- Failsafe: if no packet (UDP or web) arrives for 300ms, the drone
  disarms immediately. IMU reads are retried a few times on failure (to
  tolerate occasional motor-PWM noise on the I2C bus) and only force a
  disarm after ~25 consecutive failures (~100ms of sustained IMU loss).
- Throttle deadband: below 20/1000 the mixer output is skipped and motors
  are forced to zero, so idle-stick jitter can't spin motors while armed.

## Mobile web control UI

Connect your phone's WiFi to `esp32c3-quad` (password `quadquad123`), then
open `http://192.168.4.1/` in the phone's browser. No app install, no
internet needed -- the page is served directly by the ESP32 and is fully
self-contained (no external scripts/CDN, since the AP has no internet
route anyway).

- Two touch joysticks, Mode 2 layout: **left stick** = throttle (up/down)
  + yaw (left/right), **right stick** = pitch (up/down) + roll (left/right).
- Both sticks spring back to center (0) on release, including throttle --
  deliberately, since a touchscreen has no physical detent to hold a
  throttle position safely. You must keep a finger on the left stick to
  maintain throttle.
- **ARM/DISARM** button top-right. Per the arming rule above, arm while
  the throttle stick is at rest (0) for it to take effect.
- Top-left status text flips to "no link" if the page hasn't gotten a
  response from the drone in over 500ms.
- Sends its state to `/control` (a plain HTTP GET with query params) at
  20Hz. This is separate from, and doesn't require, `controller.py` or a
  gamepad -- either control path can be used, and both write into the
  same internal control state.

This is a browser-based touch UI, not a native app -- browsers can't send
raw UDP directly, so the web page talks HTTP to the ESP32's own small web
server instead, which is why it's a separate code path (`/` and
`/control` handlers in the .ino) rather than reusing the UDP socket.

## Control packet format

9-byte little-endian struct, matches `controller.py`'s `struct.pack("<Bhhhh", ...)`:

| Field    | Type   | Range      |
|----------|--------|------------|
| armed    | uint8  | 0 or 1     |
| throttle | int16  | 0..1000    |
| pitch    | int16  | -500..500  |
| roll     | int16  | -500..500  |
| yaw      | int16  | -500..500  |

The web UI sends the same fields as HTTP GET query parameters to
`/control` (e.g. `/control?armed=1&throttle=500&pitch=0&roll=0&yaw=0`)
instead of the binary UDP packet, but they update the exact same internal
state.

## Building / flashing

Arduino IDE or arduino-cli, board package "esp32" (Espressif) **version 3.x**,
board "ESP32C3 Dev Module". No external libraries required — only
`WiFi.h`, `WiFiUdp.h`, and `Wire.h` from the ESP32 Arduino core.

The sketch uses the core-3.x LEDC API (`ledcAttach(pin, freq, res)` /
`ledcWrite(pin, duty)`). If you're on core 2.x, replace `motorsInit()`/
`motorsWrite()` with the older channel-based API
(`ledcSetup(channel, freq, res)`, `ledcAttachPin(pin, channel)`,
`ledcWrite(channel, duty)`).

```
arduino-cli compile --fqbn esp32:esp32:esp32c3 esp32c3_custom_quad.ino
arduino-cli upload  --fqbn esp32:esp32:esp32c3 -p /dev/tty.usbXXXX esp32c3_custom_quad.ino
```

### Flashing with esptool directly

If you'd rather not use `arduino-cli upload` (e.g. driver issues, or you
want a scripted flash), compile first, then flash the produced binaries
with `esptool.py` directly:

```
arduino-cli compile --fqbn esp32:esp32:esp32c3 \
  --build-path ./build esp32c3_custom_quad.ino

esptool.py --chip esp32c3 -p /dev/tty.usbXXXX -b 460800 \
  --before default_reset --after hard_reset write_flash -z \
  --flash_mode dio --flash_freq 80m --flash_size detect \
  0x0     ./build/esp32c3_custom_quad.ino.bootloader.bin \
  0x8000  ./build/esp32c3_custom_quad.ino.partitions.bin \
  0x10000 ./build/esp32c3_custom_quad.ino.bin
```

Adjust `/dev/tty.usbXXXX` to your board's serial port (`ls /dev/tty.*` on
macOS/Linux, a `COMx` port on Windows). If the board doesn't auto-reset
into the bootloader, hold BOOT, tap RESET, release BOOT right before the
`esptool.py` command runs.

## Interactive serial motor test mode

Before touching WiFi/flight control at all, use this to confirm each
motor is on the pin/position you expect and spins correctly — entirely
over USB serial, props off, no WiFi/controller needed.

Open the Serial Monitor at 115200 baud (line ending doesn't matter), then:

| Key | Action |
|-----|--------|
| `t` | toggle test mode on/off (only works while disarmed) |
| `1`/`2`/`3`/`4` | select motor: FL / FR / BL / BR |
| `f` | forward: ramp the selected motor's speed up one step |
| `b` | backward: ramp the selected motor's speed back down one step |
| `s` | stop the selected motor |
| `a` | stop all motors |
| `?` | print current status |

These are single low-side-MOSFET brushed motors, so there's no true
electrical reverse — "forward"/"backward" here just ramps that motor's PWM
duty up or down, so you can confirm wiring and physical spin direction.
Entering test mode disables the WiFi flight-control mixer entirely (and
you can't arm over UDP while it's on); toggle it off with `t` again to
return to normal flight control.

## First bring-up checklist (do this before ever attaching propellers)

1. Flash the firmware, open Serial Monitor at 115200 baud, confirm the
   gyro calibration message and the AP IP print out.
2. With props off and the frame restrained, press `t` to enter test mode,
   then select each motor (`1`-`4`) and press `f` a few times — confirm
   the correct physical motor spins for each selection, and note its spin
   direction. Press `t` again to exit test mode when done.
3. Connect your PC/phone WiFi to `esp32c3-quad`.
4. Run `python3 controller.py --keyboard` with **no propellers attached**
   and the frame restrained. This arms at a fixed low throttle with no
   stick input — confirm all 4 motors spin together as expected.
5. Tilt the frame by hand and confirm the stabilization PID pushes the
   correct motors harder (props still off) — e.g. tilting nose-down
   should increase rear motor commands, visible via Serial debug if you
   add temporary prints, or by feel on the MOSFET/motor current draw.
6. Only after verifying directions and basic stabilization response,
   attach propellers, and only power up with the frame restrained or on
   a bench stand for the first armed test with props.
7. Switch to `--joystick` mode with a real gamepad for actual flight
   testing; keyboard mode is too coarse for controlled flight.

## Tuning

Starting gains are conservative (`pitchPID`/`rollPID`/`yawPID` near the
top of the .ino). Tune with propellers on, frame restrained or in a safe
open area, one axis at a time:

1. Raise `kp` until you see fast correction with a slight oscillation,
   then back off ~20%.
2. Add `kd` to damp overshoot.
3. Add a small `ki` only if there's a persistent steady-state tilt.

Given the very light coreless motors and 550mAh 1S battery, expect flight
times in the few-minutes range and a very twitchy, light airframe —
start with reduced `outputLimit` values if the first hover attempt feels
too sensitive.

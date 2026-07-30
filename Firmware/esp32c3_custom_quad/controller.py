#!/usr/bin/env python3
"""
Simple UDP joystick controller for the ESP32-C3 custom quad firmware.

Sends the 9-byte ControlPacket struct ("<Bhhhh") to the drone's WiFi AP:
  armed:1, throttle:0..1000, pitch:-500..500, roll:-500..500, yaw:-500..500

Usage:
  1. Connect your PC/phone to the drone's WiFi AP (SSID "esp32c3-quad").
  2. python3 controller.py --joystick     # use a connected gamepad (needs pygame)
     python3 controller.py --keyboard     # interactive single-key control, no gamepad needed

Keyboard mode is for bench testing only (props off) -- it is not smooth
enough for real flight, but is useful to verify motors spin and directions
are correct. It sends its current armed/throttle state continuously (not a
one-shot timed sequence), so arm with 'a' and add throttle with '+' whenever
you're ready -- no timing window to miss.
"""

import argparse
import select
import socket
import struct
import sys
import termios
import time
import tty

DRONE_IP = "192.168.4.1"
DRONE_PORT = 4210
PACKET_FMT = "<Bhhhh"
SEND_HZ = 50


def send_packet(sock, armed, throttle, pitch, roll, yaw):
    throttle = max(0, min(1000, int(throttle)))
    pitch = max(-500, min(500, int(pitch)))
    roll = max(-500, min(500, int(roll)))
    yaw = max(-500, min(500, int(yaw)))
    packet = struct.pack(PACKET_FMT, 1 if armed else 0, throttle, pitch, roll, yaw)
    sock.sendto(packet, (DRONE_IP, DRONE_PORT))


def run_joystick(sock):
    import pygame

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick found.")
        sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Using joystick: {js.get_name()}")
    print("Hold button 0 to arm. Ctrl+C to quit.")

    period = 1.0 / SEND_HZ
    while True:
        pygame.event.pump()
        armed = js.get_button(0) == 1
        # Typical axis layout: axis1 = throttle (inverted), axis0 = roll,
        # axis3 = pitch (inverted), axis2 = yaw. Adjust for your controller.
        throttle = (1 - js.get_axis(1)) / 2 * 1000
        roll = js.get_axis(0) * 500
        pitch = -js.get_axis(3) * 500
        yaw = js.get_axis(2) * 500
        send_packet(sock, armed, throttle, pitch, roll, yaw)
        time.sleep(period)


def run_keyboard(sock):
    print("Keyboard test mode: props OFF. Ctrl+C or 'q' to quit.")
    print("  a       arm (resets throttle to 0 -- required by the firmware's")
    print("          arming safety check, which needs low throttle to arm)")
    print("  z       disarm")
    print("  + / -   increase / decrease throttle")
    print("  q       quit")
    print()
    print("State is sent continuously at all times, so there's no timing "
          "window to miss -- arm with 'a', watch the drone's Serial debug "
          "output for armed=1, then raise throttle with '+' once confirmed.")

    armed = False
    throttle = 0
    STEP = 50
    period = 1.0 / SEND_HZ

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1)
                if c == "a":
                    armed = True
                    throttle = 0
                    print("[ARMED] throttle reset to 0")
                elif c == "z":
                    armed = False
                    throttle = 0
                    print("[DISARMED]")
                elif c in ("+", "="):
                    throttle = min(1000, throttle + STEP)
                    print(f"throttle={throttle}")
                elif c in ("-", "_"):
                    throttle = max(0, throttle - STEP)
                    print(f"throttle={throttle}")
                elif c == "q":
                    break
            send_packet(sock, armed, throttle, 0, 0, 0)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        send_packet(sock, armed=False, throttle=0, pitch=0, roll=0, yaw=0)


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--joystick", action="store_true")
    group.add_argument("--keyboard", action="store_true")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if args.joystick:
            run_joystick(sock)
        else:
            run_keyboard(sock)
    except KeyboardInterrupt:
        pass
    finally:
        send_packet(sock, armed=False, throttle=0, pitch=0, roll=0, yaw=0)
        sock.close()


if __name__ == "__main__":
    main()

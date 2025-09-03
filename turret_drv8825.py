#!/usr/bin/env python3
"""
Tracking Turret - Raspberry Pi 5 & DRV8825 Stepper Driver
Compatible with:
- Raspberry Pi 5
- Python 3.x
- rpi5-drv8825 motor library

REQUIREMENTS:
============
Install the following packages before running this script:

# Computer Vision
opencv-python>=4.12.0.88
imutils>=0.5.4

# Motor Control (DRV8825 Stepper Driver)
RPI5-DRV8825>=1.0.0

# GPIO Control for Raspberry Pi 5
lgpio>=0.2.2.0

# Core dependencies (usually included with Python 3)
# These should be available by default but listed for completeness
# threading (built-in)
# time (built-in)
# sys (built-in)
# atexit (built-in)
# contextlib (built-in)
# termios (built-in on Unix systems)

INSTALLATION:
============
pip install opencv-python>=4.12.0.88 imutils>=0.5.4 RPI5-DRV8825>=1.0.0 lgpio>=0.2.2.0
"""

import cv2
import time
import threading
import atexit
import sys
import termios
import contextlib
import imutils

try:
    import lgpio as GPIO
    print("Using lgpio for GPIO control (Pi 5 compatible)")
except ImportError:
    print("Error: lgpio library not installed")
    sys.exit(1)

try:
    from RPI5_DRV8825 import DRV8825
    print("Using RPI5-DRV8825 motor driver")
except ImportError:
    print("Error: RPI5-DRV8825 not installed. Run: pip install RPI5-DRV8825")
    sys.exit(1)

### User Parameters ###
MOTOR_X_REVERSED = False
MOTOR_Y_REVERSED = False
MAX_STEPS_X = 30
MAX_STEPS_Y = 15
RELAY_PIN = 22
#######################

# Define motor pins (customize these to your wiring!)
X_PINS = {'dir': 13, 'step': 19, 'enable': 12}
Y_PINS = {'dir': 14, 'step': 18, 'enable': 23}
MODE_PINS = (16, 17, 20)

@contextlib.contextmanager
def raw_mode(file):
    """Enable raw keyboard input (no enter required)."""
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)


class VideoUtils:
    """Video capture and motion detection helpers."""

    @staticmethod
    def live_video(camera_port=0):
        cap = cv2.VideoCapture(camera_port)
        if not cap.isOpened():
            print("Error: Could not open camera")
            return
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            cv2.imshow("Video", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cap.release()
        cv2.destroyAllWindows()

    @staticmethod
    def find_motion(callback, camera_port=0, show_video=False):
        cam = cv2.VideoCapture(camera_port)
        if not cam.isOpened():
            print("Error: Could not open camera")
            return
        time.sleep(0.25)
        firstFrame, tempFrame = None, None
        count = 0
        while True:
            grabbed, frame = cam.read()
            if not grabbed:
                break
            frame = imutils.resize(frame, width=500)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)
            if firstFrame is None:
                print("Waiting for video to adjust...")
                if tempFrame is None:
                    tempFrame = gray
                    continue
                else:
                    delta = cv2.absdiff(tempFrame, gray)
                    tempFrame = gray
                    tst = cv2.threshold(delta, 5, 255, cv2.THRESH_BINARY)[1]
                    tst = cv2.dilate(tst, None, iterations=2)
                    if count > 30:
                        print("Done.\nWaiting for motion.")
                        if not cv2.countNonZero(tst) > 0:
                            firstFrame = gray
                        else:
                            continue
                    else:
                        count += 1
                        continue
            frameDelta = cv2.absdiff(firstFrame, gray)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=2)
            c = VideoUtils.get_best_contour(thresh.copy(), 5000)
            if c is not None:
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                callback(c, frame)
            if show_video:
                cv2.imshow("Security Feed", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        cam.release()
        cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        contours, _ = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_cnt = None
        best_area = threshold
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt


class Turret:
    """Turret control using DRV8825 stepper driver."""

    def __init__(self, friendly_mode=True):
        self.friendly_mode = friendly_mode
        # Initialize motors
        self.motor_x = DRV8825(dir_pin=X_PINS['dir'],
                               step_pin=X_PINS['step'],
                               enable_pin=X_PINS['enable'],
                               mode_pins=MODE_PINS)
        self.motor_y = DRV8825(dir_pin=Y_PINS['dir'],
                               step_pin=Y_PINS['step'],
                               enable_pin=Y_PINS['enable'],
                               mode_pins=MODE_PINS)
        self.motor_x.set_microstep(1)
        self.motor_y.set_microstep(1)
        self.motor_x.enable(True)
        self.motor_y.enable(True)
        self.current_x_steps = 0
        self.current_y_steps = 0
        # Relay setup
        self.gpio_chip = None
        try:
            self.gpio_chip = GPIO.gpiochip_open(0)
            GPIO.gpio_claim_output(self.gpio_chip, RELAY_PIN)
            GPIO.gpio_write(self.gpio_chip, RELAY_PIN, 0)
        except Exception as e:
            print(f"Warning: Could not setup relay pin {RELAY_PIN}: {e}")

    def calibrate(self):
        print("Calibrate Y (w=up, s=down, enter=finish):")
        self.__calibrate_axis("y")
        print("Calibrate X (a=left, d=right, enter=finish):")
        self.__calibrate_axis("x")

    def __calibrate_axis(self, axis):
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break
                    if axis == "x":
                        if ch == "a":
                            self.move_backward_x(5) if MOTOR_X_REVERSED else self.move_forward_x(5)
                        elif ch == "d":
                            self.move_forward_x(5) if MOTOR_X_REVERSED else self.move_backward_x(5)
                        elif ch == "\n":
                            break
                    else:
                        if ch == "w":
                            self.move_forward_y(5) if MOTOR_Y_REVERSED else self.move_backward_y(5)
                        elif ch == "s":
                            self.move_backward_y(5) if MOTOR_Y_REVERSED else self.move_forward_y(5)
                        elif ch == "\n":
                            break
            except (KeyboardInterrupt, EOFError):
                sys.exit(1)

    def motion_detection(self, show_video=False):
        VideoUtils.find_motion(self.__move_axis, show_video=show_video)

    def __move_axis(self, contour, frame):
        (v_h, v_w) = frame.shape[:2]
        (x, y, w, h) = cv2.boundingRect(contour)
        target_x = (2*MAX_STEPS_X * (x + w//2) / v_w) - MAX_STEPS_X
        target_y = (2*MAX_STEPS_Y * (y + h//2) / v_h) - MAX_STEPS_Y
        
        print(f"x: {target_x}, y: {target_y}")
        print(f"current x: {self.current_x_steps}, current y: {self.current_y_steps}")
        
        t_x = threading.Thread()
        t_y = threading.Thread()
        t_fire = threading.Thread()
        
        # move x
        if (target_x - self.current_x_steps) > 0:
            self.current_x_steps += 1
            if MOTOR_X_REVERSED:
                t_x = threading.Thread(target=self.move_forward_x, args=(2,))
            else:
                t_x = threading.Thread(target=self.move_backward_x, args=(2,))
        elif (target_x - self.current_x_steps) < 0:
            self.current_x_steps -= 1
            if MOTOR_X_REVERSED:
                t_x = threading.Thread(target=self.move_backward_x, args=(2,))
            else:
                t_x = threading.Thread(target=self.move_forward_x, args=(2,))
        
        # move y
        if (target_y - self.current_y_steps) > 0:
            self.current_y_steps += 1
            if MOTOR_Y_REVERSED:
                t_y = threading.Thread(target=self.move_backward_y, args=(2,))
            else:
                t_y = threading.Thread(target=self.move_forward_y, args=(2,))
        elif (target_y - self.current_y_steps) < 0:
            self.current_y_steps -= 1
            if MOTOR_Y_REVERSED:
                t_y = threading.Thread(target=self.move_forward_y, args=(2,))
            else:
                t_y = threading.Thread(target=self.move_backward_y, args=(2,))
        
        # fire if necessary
        if not self.friendly_mode:
            if abs(target_y - self.current_y_steps) <= 2 and abs(target_x - self.current_x_steps) <= 2:
                t_fire = threading.Thread(target=self.fire)
        
        t_x.start()
        t_y.start()
        t_fire.start()
        
        t_x.join()
        t_y.join()
        t_fire.join()

    def interactive(self):
        print("Interactive mode: w/s=up/down, a/d=left/right, enter=fire, q=quit")
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch or ch == "q":
                        break
                    if ch == "w":
                        self.move_forward_y(5) if MOTOR_Y_REVERSED else self.move_backward_y(5)
                    elif ch == "s":
                        self.move_backward_y(5) if MOTOR_Y_REVERSED else self.move_forward_y(5)
                    elif ch == "a":
                        self.move_backward_x(5) if MOTOR_X_REVERSED else self.move_forward_x(5)
                    elif ch == "d":
                        self.move_forward_x(5) if MOTOR_X_REVERSED else self.move_backward_x(5)
                    elif ch == "\n":
                        self.fire()
            except (KeyboardInterrupt, EOFError):
                pass

    def fire(self):
        try:
            if self.gpio_chip is not None:
                GPIO.gpio_write(self.gpio_chip, RELAY_PIN, 1)
                time.sleep(1)
                GPIO.gpio_write(self.gpio_chip, RELAY_PIN, 0)
        except Exception as e:
            print(f"Error firing: {e}")

    def move_forward_x(self, steps):
        self.motor_x.run(steps, True)
    def move_backward_x(self, steps):
        self.motor_x.run(steps, False)
    def move_forward_y(self, steps):
        self.motor_y.run(steps, True)
    def move_backward_y(self, steps):
        self.motor_y.run(steps, False)

    def shutdown(self):
        self.motor_x.enable(False)
        self.motor_y.enable(False)
        # Cleanup GPIO
        try:
            if self.gpio_chip is not None:
                GPIO.gpiochip_close(self.gpio_chip)
        except Exception as e:
            print(f"Warning: Error closing GPIO: {e}")


if __name__ == "__main__":
    t = Turret(friendly_mode=False)
    try:
        mode = input("Choose mode: (1) Motion Detection, (2) Interactive\n")
        if mode == "1":
            t.calibrate()
            if input("Live video? (y/n): ").lower() == "y":
                t.motion_detection(show_video=True)
            else:
                t.motion_detection()
        elif mode == "2":
            if input("Live video? (y/n): ").lower() == "y":
                live_thread = threading.Thread(target=VideoUtils.live_video)
                live_thread.daemon = True
                live_thread.start()
            t.interactive()
        else:
            print("Invalid choice.")
    finally:
        t.shutdown()

#!/usr/bin/env python3
"""
Tracking Turret - Raspberry Pi 5 & Python 3 Compatible Version

A motion tracking turret with improved compatibility for:
- Raspberry Pi 5
- Python 3.x
- Modern libraries

Original project: https://www.youtube.com/watch?v=HoRPWUl_sF8
"""

try:
    import cv2
except Exception as e:
    print("Warning: OpenCV not installed. To use motion detection, make sure you've properly configured OpenCV.")

import time
import threading
import atexit
import sys
import termios
import contextlib

import imutils
try:
    # Try new RPi.GPIO first (compatible with Pi 5)
    import RPi.GPIO as GPIO
except ImportError:
    try:
        # Fallback to lgpio for Pi 5 if RPi.GPIO doesn't work
        import lgpio as GPIO
        print("Using lgpio for GPIO control (Pi 5 compatible)")
    except ImportError:
        print("Error: No GPIO library found. Install RPi.GPIO or lgpio")
        sys.exit(1)

# Updated motor HAT import - try multiple options for compatibility
try:
    from adafruit_motor import stepper
    from adafruit_motorkit import MotorKit
    MOTOR_LIB = "circuitpython"
    print("Using CircuitPython libraries (recommended for Pi 5)")
except ImportError:
    try:
        from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
        MOTOR_LIB = "legacy"
        print("Using legacy Adafruit MotorHAT library")
    except ImportError:
        print("Error: No motor HAT library found. Install adafruit-circuitpython-motorkit or Adafruit-Motor-HAT-Python-Library")
        sys.exit(1)


### User Parameters ###

MOTOR_X_REVERSED = False
MOTOR_Y_REVERSED = False

MAX_STEPS_X = 30
MAX_STEPS_Y = 15

RELAY_PIN = 22

#######################


@contextlib.contextmanager
def raw_mode(file):
    """
    Magic function that allows key presses.
    :param file: Input file (usually sys.stdin)
    :return: Context manager for raw input mode
    """
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)


class VideoUtils(object):
    """
    Helper functions for video utilities.
    """
    @staticmethod
    def live_video(camera_port=0):
        """
        Opens a window with live video.
        :param camera_port: Camera port number
        :return: None
        """
        video_capture = cv2.VideoCapture(camera_port)
        
        if not video_capture.isOpened():
            print("Error: Could not open camera")
            return

        while True:
            # Capture frame-by-frame
            ret, frame = video_capture.read()
            
            if not ret:
                print("Error: Could not read frame")
                break

            # Display the resulting frame
            cv2.imshow('Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything is done, release the capture
        video_capture.release()
        cv2.destroyAllWindows()

    @staticmethod
    def find_motion(callback, camera_port=0, show_video=False):
        """
        Detect motion and call callback function when motion is found.
        :param callback: Function to call when motion is detected
        :param camera_port: Camera port number
        :param show_video: Whether to show video feed
        :return: None
        """
        camera = cv2.VideoCapture(camera_port)
        
        if not camera.isOpened():
            print("Error: Could not open camera")
            return
            
        time.sleep(0.25)

        # initialize the first frame in the video stream
        firstFrame = None
        tempFrame = None
        count = 0

        # loop over the frames of the video
        while True:
            # grab the current frame and initialize the occupied/unoccupied text
            (grabbed, frame) = camera.read()

            # if the frame could not be grabbed, then we have reached the end
            if not grabbed:
                break

            # resize the frame, convert it to grayscale, and blur it
            frame = imutils.resize(frame, width=500)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            # if the first frame is None, initialize it
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

            # compute the absolute difference between the current frame and first frame
            frameDelta = cv2.absdiff(firstFrame, gray)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

            # dilate the thresholded image to fill in holes, then find contours
            thresh = cv2.dilate(thresh, None, iterations=2)
            c = VideoUtils.get_best_contour(thresh.copy(), 5000)

            if c is not None:
                # compute the bounding box for the contour, draw it on the frame,
                # and update the text
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                callback(c, frame)

            # show the frame and record if the user presses a key
            if show_video:
                cv2.imshow("Security Feed", frame)
                key = cv2.waitKey(1) & 0xFF

                # if the 'q' key is pressed, break from the loop
                if key == ord("q"):
                    break

        # cleanup the camera and close any open windows
        camera.release()
        cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        """
        Find the best contour (largest area above threshold).
        :param imgmask: Image mask to find contours in
        :param threshold: Minimum area threshold
        :return: Best contour or None
        """
        # Updated for OpenCV 4.x - findContours returns only contours and hierarchy
        contours, hierarchy = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = threshold
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt


class Turret(object):
    """
    Class used for turret control.
    Compatible with Raspberry Pi 5 and modern libraries.
    """
    def __init__(self, friendly_mode=True):
        self.friendly_mode = friendly_mode

        # Initialize motor control based on available library
        if MOTOR_LIB == "circuitpython":
            # Use CircuitPython libraries (recommended for Pi 5)
            self.kit = MotorKit()
            self.sm_x = self.kit.stepper1
            self.sm_y = self.kit.stepper2
        else:
            # Use legacy Adafruit MotorHAT
            self.mh = Adafruit_MotorHAT()
            atexit.register(self.__turn_off_motors)
            
            # Stepper motor 1
            self.sm_x = self.mh.getStepper(200, 1)      # 200 steps/rev, motor port #1
            self.sm_x.setSpeed(5)                       # 5 RPM
            
            # Stepper motor 2
            self.sm_y = self.mh.getStepper(200, 2)      # 200 steps/rev, motor port #2
            self.sm_y.setSpeed(5)                       # 5 RPM

        self.current_x_steps = 0
        self.current_y_steps = 0

        # Relay setup
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(RELAY_PIN, GPIO.OUT)
            GPIO.output(RELAY_PIN, GPIO.LOW)
        except Exception as e:
            print(f"Warning: Could not setup GPIO pin {RELAY_PIN}: {e}")

    def calibrate(self):
        """
        Waits for input to calibrate the turret's axis
        :return: None
        """
        print("Please calibrate the tilt of the gun so that it is level. Commands: (w) moves up, "
              "(s) moves down. Press (enter) to finish.\n")
        self.__calibrate_y_axis()

        print("Please calibrate the yaw of the gun so that it aligns with the camera. Commands: (a) moves left, "
              "(d) moves right. Press (enter) to finish.\n")
        self.__calibrate_x_axis()

        print("Calibration finished.")

    def __calibrate_x_axis(self):
        """
        Waits for input to calibrate the x axis
        :return: None
        """
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break

                    elif ch == "a":
                        if MOTOR_X_REVERSED:
                            self.move_backward_x(5)
                        else:
                            self.move_forward_x(5)
                    elif ch == "d":
                        if MOTOR_X_REVERSED:
                            self.move_forward_x(5)
                        else:
                            self.move_backward_x(5)
                    elif ch == "\n":
                        break

            except (KeyboardInterrupt, EOFError):
                print("Error: Unable to calibrate turret. Exiting...")
                sys.exit(1)

    def __calibrate_y_axis(self):
        """
        Waits for input to calibrate the y axis.
        :return: None
        """
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break

                    if ch == "w":
                        if MOTOR_Y_REVERSED:
                            self.move_forward_y(5)
                        else:
                            self.move_backward_y(5)
                    elif ch == "s":
                        if MOTOR_Y_REVERSED:
                            self.move_backward_y(5)
                        else:
                            self.move_forward_y(5)
                    elif ch == "\n":
                        break

            except (KeyboardInterrupt, EOFError):
                print("Error: Unable to calibrate turret. Exiting...")
                sys.exit(1)

    def motion_detection(self, show_video=False):
        """
        Uses the camera to move the turret. OpenCV must be configured to use this.
        :param show_video: Whether to show video feed
        :return: None
        """
        VideoUtils.find_motion(self.__move_axis, show_video=show_video)

    def __move_axis(self, contour, frame):
        """
        Move turret axis based on detected contour.
        :param contour: Detected contour
        :param frame: Video frame
        :return: None
        """
        (v_h, v_w) = frame.shape[:2]
        (x, y, w, h) = cv2.boundingRect(contour)

        # Calculate target steps
        target_steps_x = (2*MAX_STEPS_X * (x + w // 2) / v_w) - MAX_STEPS_X
        target_steps_y = (2*MAX_STEPS_Y*(y+h//2) / v_h) - MAX_STEPS_Y

        print(f"x: {target_steps_x}, y: {target_steps_y}")
        print(f"current x: {self.current_x_steps}, current y: {self.current_y_steps}")

        t_x = threading.Thread()
        t_y = threading.Thread()
        t_fire = threading.Thread()

        # move x
        if (target_steps_x - self.current_x_steps) > 0:
            self.current_x_steps += 1
            if MOTOR_X_REVERSED:
                t_x = threading.Thread(target=self.move_forward_x, args=(2,))
            else:
                t_x = threading.Thread(target=self.move_backward_x, args=(2,))
        elif (target_steps_x - self.current_x_steps) < 0:
            self.current_x_steps -= 1
            if MOTOR_X_REVERSED:
                t_x = threading.Thread(target=self.move_backward_x, args=(2,))
            else:
                t_x = threading.Thread(target=self.move_forward_x, args=(2,))

        # move y
        if (target_steps_y - self.current_y_steps) > 0:
            self.current_y_steps += 1
            if MOTOR_Y_REVERSED:
                t_y = threading.Thread(target=self.move_backward_y, args=(2,))
            else:
                t_y = threading.Thread(target=self.move_forward_y, args=(2,))
        elif (target_steps_y - self.current_y_steps) < 0:
            self.current_y_steps -= 1
            if MOTOR_Y_REVERSED:
                t_y = threading.Thread(target=self.move_forward_y, args=(2,))
            else:
                t_y = threading.Thread(target=self.move_backward_y, args=(2,))

        # fire if necessary
        if not self.friendly_mode:
            if abs(target_steps_y - self.current_y_steps) <= 2 and abs(target_steps_x - self.current_x_steps) <= 2:
                t_fire = threading.Thread(target=self.fire)

        t_x.start()
        t_y.start()
        t_fire.start()

        t_x.join()
        t_y.join()
        t_fire.join()

    def interactive(self):
        """
        Starts an interactive session. Key presses determine movement.
        :return: None
        """
        self.move_forward_x(1)
        self.move_forward_y(1)

        print('Commands: Pivot with (a) and (d). Tilt with (w) and (s). Exit with (q)\n')
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch or ch == "q":
                        break

                    if ch == "w":
                        if MOTOR_Y_REVERSED:
                            self.move_forward_y(5)
                        else:
                            self.move_backward_y(5)
                    elif ch == "s":
                        if MOTOR_Y_REVERSED:
                            self.move_backward_y(5)
                        else:
                            self.move_forward_y(5)
                    elif ch == "a":
                        if MOTOR_X_REVERSED:
                            self.move_backward_x(5)
                        else:
                            self.move_forward_x(5)
                    elif ch == "d":
                        if MOTOR_X_REVERSED:
                            self.move_forward_x(5)
                        else:
                            self.move_backward_x(5)
                    elif ch == "\n":
                        self.fire()

            except (KeyboardInterrupt, EOFError):
                pass

    def fire(self):
        """
        Activate relay to fire.
        :return: None
        """
        try:
            GPIO.output(RELAY_PIN, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(RELAY_PIN, GPIO.LOW)
        except Exception as e:
            print(f"Error firing: {e}")

    def move_forward_x(self, steps):
        """
        Moves the X stepper motor forward the specified number of steps.
        :param steps: Number of steps to move
        :return: None
        """
        if MOTOR_LIB == "circuitpython":
            for _ in range(steps):
                self.sm_x.onestep(direction=stepper.FORWARD)
                time.sleep(0.01)
        else:
            self.sm_x.step(steps, Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.INTERLEAVE)

    def move_backward_x(self, steps):
        """
        Moves the X stepper motor backward the specified number of steps
        :param steps: Number of steps to move
        :return: None
        """
        if MOTOR_LIB == "circuitpython":
            for _ in range(steps):
                self.sm_x.onestep(direction=stepper.BACKWARD)
                time.sleep(0.01)
        else:
            self.sm_x.step(steps, Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.INTERLEAVE)

    def move_forward_y(self, steps):
        """
        Moves the Y stepper motor forward the specified number of steps.
        :param steps: Number of steps to move
        :return: None
        """
        if MOTOR_LIB == "circuitpython":
            for _ in range(steps):
                self.sm_y.onestep(direction=stepper.FORWARD)
                time.sleep(0.01)
        else:
            self.sm_y.step(steps, Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.INTERLEAVE)

    def move_backward_y(self, steps):
        """
        Moves the Y stepper motor backward the specified number of steps
        :param steps: Number of steps to move
        :return: None
        """
        if MOTOR_LIB == "circuitpython":
            for _ in range(steps):
                self.sm_y.onestep(direction=stepper.BACKWARD)
                time.sleep(0.01)
        else:
            self.sm_y.step(steps, Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.INTERLEAVE)

    def __turn_off_motors(self):
        """
        Recommended for auto-disabling motors on shutdown!
        :return: None
        """
        if MOTOR_LIB == "legacy":
            self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
            self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
            self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
            self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
        # CircuitPython motors are automatically released


if __name__ == "__main__":
    t = Turret(friendly_mode=False)

    user_input = input("Choose an input mode: (1) Motion Detection, (2) Interactive\n")

    if user_input == "1":
        t.calibrate()
        if input("Live video? (y, n)\n").lower() == "y":
            t.motion_detection(show_video=True)
        else:
            t.motion_detection()
    elif user_input == "2":
        if input("Live video? (y, n)\n").lower() == "y":
            live_thread = threading.Thread(target=VideoUtils.live_video)
            live_thread.daemon = True
            live_thread.start()
        t.interactive()
    else:
        print("Unknown input mode. Please choose a number (1) or (2)")

# import the necessary packages
from picamera.array import PiRGBArray  # Generates a 3D RGB array
from picamera import PiCamera  # Provides a Python interface for the RPi Camera Module
import time  # Provides time-related functions
import cv2  # OpenCV library
import numpy as np 
import RPi.GPIO as GPIO # Provides a Python interface for the RPi GPIO pins and PWM
import math
import pd # Class object for Proportional Derivative (PD) controller

### METHOD BLOCK BEGINS ###

def detect_object(image):
    # Arrays store the minimum and maximum values for each HSV value.
    minimum_values = np.array([7, 0, 0])
    maximum_values = np.array([150, 255, 255])

    # Converts the image from BGR to type HSV
    image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Filters image so only pixels with a colour that falls within the HSV value range (defined in arrays above) appear black on a white canvas
    image_HSV_specific = cv2.inRange(image_HSV, minimum_values, maximum_values)

    # Blurs image 
    blur_image = cv2.blur(image_HSV_specific, (8, 8))  # kernal of 8x8 used

    # cv2.imshow("Blur image", blur_image) # Debugging purposes 

    # Finds contours in image (which can be used to identify shapes). Sores contours in an array
    contours, _ = cv2.findContours(blur_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    # Sorts contours array in descending order based on the area of the enclosed shapes
    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

    # each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object.
    for contour in contours:
        shape_area = cv2.contourArea(contour)

        # If shape has an appropriate area
        if shape_area > 1800 and shape_area < 160000:
            perimeter = cv2.arcLength(contour, True)

            # accuracy parameter - maximum distance from contour to approximated contour
            episilon = 0.03 * perimeter
            verticies = cv2.approxPolyDP(contour, episilon, True)  # Adjust values as needed

            # Circles will still have verticies, but they will be higher than most shapes
            if len(verticies) > 6:

                # Gets parameters for a rectangle around object
                x, y, width, height = cv2.boundingRect(verticies)

                # Sets return data
                object_center_coordinates = [x + width / 2, y - height / 2]
                object_rectangle_width_height = [width, height]

                if object_center_coordinates[0] < (video_frame_width / 2 - 60):  # Left
                    object_position_relative = 0
                elif object_center_coordinates[0] > (video_frame_width / 2 + 60):  # Right
                    object_position_relative = 1
                else:
                    object_position_relative = 2

                # cv2.rectangle(image, (x, y), (x + width, y + height), (0, 255, 0), 3)  # Draws rectangle on image. For debugging purposes

                # Exits for loop (and function) after the ball is found to reduce execution time
                return object_position_relative, shape_area, object_center_coordinates, object_rectangle_width_height

    return -1, -1, [-1, -1], [-1, -1]  # If object not found


# Calculates straight-line distance from the cart camera to the object
def calc_distance(object_dimensions):
    focalLen = 612.8
    return (6 * focalLen) / object_dimensions[0]

# Calculates the angle between cart camera to the object
def calc_angle(obj_coordinates, distance):
    pixelDistanceFromCenter = abs(320 - obj_coordinates[0])
    angle = math.degrees(math.atan((pixelDistanceFromCenter * 0.015) / distance))
    return angle

# Macro called if object is not found in frame
def intelligent_search(position_relative, coordinates, distance, times):
    turn_direction = 0  # 1 for right, 0 for left
    motor_power = 0  # Desired motor power

    # Uses the past position of the object relative to the car to determine which direction it is headed in.
    if position_relative == 1 or position_relative == 0: # If ball was on the left or right of frame before leaving
        turn_direction = position_relative
    elif position_relative == 2: # If ball was in centre of frame before leaving

        # If the object happened to be in the centre of the frame when it left the image, the robot uses the past coordinates of the object to predict its trajectory
        # Coordinate system
        #           Top
        # (0, 0)              (x, 0)
        #
        #                               Right
        #
        # (0, y)              (x, y)
        # We only care about x value when determining trajectory (since we know the ball exited from the top)

        if coordinates[0][0] >= coordinates[1][0]:  # Object was moving to the right
            turn_direction = 1
        else:
            turn_direction = 0

    # Uses the past measurements of the object to determine appropriate motor power
    # Minimum value will be 80, and maximum will be 100

    # Methodology
    # We can estimate the speed with which the object moved by looking at the change in its x coordinates.
    # We can use the measured object distance to adjust this estimation to get the desired motor output
    estimated_speed = (coordinates[0][0] - coordinates[1][0]) / (times[0] - times[1])

    # Ensures we do not have a divide by zero case
    if distance != 0:
        scale_factor = 1 / distance
    else:
        scale_factor = 1

    motor_power = estimated_speed * scale_factor

    # Check so motors don't get over/underpowered
    if motor_power > 100:
        motor_power = 100
    elif motor_power < 80:
        motor_power = 80

    return motor_power, turn_direction

# Called when program terminates 
def kill_program():
    # Kills PWMs
    pwm_motor_1.stop()
    pwm_motor_2.stop()

    # Cleans up GPIO (sets all to LOW)
    GPIO.cleanup()
    print("Cleanup done")


### METHOD BLOCK ENDS ###


### MAIN BLOCK BEGINS ###

# Initialize the Pi camera
camera = PiCamera()

# Set the camera resolution
camera.resolution = (640, 480)

# Set camera parameters
camera.framerate = 32
video_frame_width = 640
video_frame_height = 480
video_brightness_adjustment = 7

raw_capture = PiRGBArray(camera, size=(640, 480))  # Generates a 3D RGB array and stores it in rawCapture
time.sleep(0.1)  # Wait a certain number of seconds to allow the camera time to warmup

angle_pd = pd.PD(5.5, 2, 80, 90)  # Constructs PD class for when the robot needs to rotate
forward_pd = pd.PD(12, 5, 65, 90)  # Constructs PD class for when the robot needs to move forwards and backwards

# Object Position Parameters
current_angle = 0
current_distance = 0

# Desired position for the object with respect to the cart
desired_angle = 0
desired_distance = 50

# Define ports for motor 1 (Left)
motor_1_forward = 24
motor_1_backward = 23
motor_1_en = 25

# Define ports for motor 2 (Right)
motor_2_forward = 22
motor_2_backward = 27
motor_2_en = 17

# Initialize GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_1_forward, GPIO.OUT)
GPIO.setup(motor_1_backward, GPIO.OUT)
GPIO.setup(motor_1_en, GPIO.OUT)
GPIO.output(motor_1_forward, GPIO.LOW)
GPIO.output(motor_1_backward, GPIO.LOW)
GPIO.setup(motor_2_forward, GPIO.OUT)
GPIO.setup(motor_2_backward, GPIO.OUT)
GPIO.setup(motor_2_en, GPIO.OUT)
GPIO.output(motor_2_forward, GPIO.LOW)
GPIO.output(motor_2_backward, GPIO.LOW)
pwm_motor_1 = GPIO.PWM(motor_1_en, 1000)
pwm_motor_2 = GPIO.PWM(motor_2_en, 1000)

# Starts PWMs
pwm_motor_1.start(75)
pwm_motor_2.start(70)

# Information about the object image within the frame
object_position_relative = 0  # 0 means left of center, 1 means right, 2 means center, -1 means not found
object_contour_area = 0  # Gives indication of how close the object is to the car
object_center_coordinates = [0, 0]  # Coordinates of object in the frame
object_rectangle_width_height = [0, 0]  # Width and height of bounding rectangle

# Stores past information about the object
past_position_relative = 0
past_coordinates = [[0, 0], [0, 0]]  # First row is most recent data, second row is second most recent data
past_distance = [0, 0]  # [0] = Most recent data; [1] = Second most recent data
past_angle = [0, 0]  # [0] = Most recent data; [1] = Second most recent data
measurement_time = [0, 0]  # Stores time that measurement was made

undetected_counter = 0  # Counts number of times cart did not detect ball in a row

for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):  # Iterates over each frame in the video stream

    image = frame.array  # Grab the raw NumPy array representing the image

    # Calls method to process image and identify object
    object_position_relative, object_contour_area, object_center_coordinates, object_rectangle_width_height = detect_object(image)

    time.sleep(0.05)

    raw_capture.truncate(0)  # Clear the video stream in preparation for the next frame

    if object_contour_area != -1:  # If object is found
        undetected_counter = 0

        # Calculates distances
        current_distance = calc_distance(object_rectangle_width_height)
        current_angle = calc_angle(object_center_coordinates, current_distance)

        # Stores the current values as past values (Also adjusts arrays to make room for the past values)
        measurement_time[1] = measurement_time[0]
        measurement_time[0] = time.perf_counter()
        past_coordinates[1][0] = past_coordinates[0][0]
        past_coordinates[1][1] = past_coordinates[0][1]
        past_coordinates[0][0] = object_center_coordinates[0]
        past_coordinates[0][1] = object_center_coordinates[1]
        past_position_relative = object_position_relative
        past_distance[1] = past_distance[0]
        past_angle[1] = past_angle[0]
        past_distance[0] = current_distance
        past_angle[0] = current_angle

        # Determines desired motor output using PIDs
        if object_position_relative == 0 or object_position_relative == 1: # If object is not centered, we want rotational motion
            motor_pwm = angle_pd.get_output(current_angle, desired_angle)
        elif object_position_relative == 2 or object_position_relative == -1: # If object is centered, we want translational motion
            motor_pwm = forward_pd.get_output(current_distance, desired_distance)

        # Powers motors
        if object_position_relative == 0:  # Turn left

            # All if statements follow a similar structure, so code will only be commented here to reduce redundancy.

            # Adjusts PWM values
            pwm_motor_2.ChangeDutyCycle(motor_pwm)
            time.sleep(0.05) # Gives time for PWM to adjust
            # Sets GPIO pins to HIGH and LOW to exhibit desired behaviour
            GPIO.output(motor_1_forward, GPIO.LOW)
            GPIO.output(motor_1_backward, GPIO.LOW)
            GPIO.output(motor_2_forward, GPIO.HIGH)
            GPIO.output(motor_2_backward, GPIO.LOW)
            time.sleep(0.15) # Lets motors run for 150 ms (This is done because of traction issues discussed in the report)
        elif object_position_relative == 1:  # Turn right
            pwm_motor_1.ChangeDutyCycle(motor_pwm)
            time.sleep(0.05)
            GPIO.output(motor_1_forward, GPIO.HIGH)
            GPIO.output(motor_1_backward, GPIO.LOW)
            GPIO.output(motor_2_forward, GPIO.LOW)
            GPIO.output(motor_2_backward, GPIO.LOW)
            time.sleep(0.15)
        elif object_position_relative == 2:  # Move forwards or backwards
            if current_distance > desired_distance:  # If car needs to move forwards
                pwm_motor_1.ChangeDutyCycle(motor_pwm)
                pwm_motor_2.ChangeDutyCycle(motor_pwm)
                time.sleep(0.05)
                GPIO.output(motor_1_forward, GPIO.HIGH)
                GPIO.output(motor_1_backward, GPIO.LOW)
                GPIO.output(motor_2_forward, GPIO.HIGH)
                GPIO.output(motor_2_backward, GPIO.LOW)
            elif current_distance < desired_distance:  # If car needs to move backwards
                pwm_motor_1.ChangeDutyCycle(motor_pwm)
                pwm_motor_2.ChangeDutyCycle(motor_pwm)
                time.sleep(0.05)
                GPIO.output(motor_1_forward, GPIO.LOW)
                GPIO.output(motor_1_backward, GPIO.HIGH)
                GPIO.output(motor_2_forward, GPIO.LOW)
                GPIO.output(motor_2_backward, GPIO.HIGH)
            time.sleep(0.2)

    else:  # Object not found
        undetected_counter = undetected_counter + 1 # Increments undetected counter

        if undetected_counter == 5: # Only starts searching macros when we have not detected the object in five consecutive frames
            undetected_counter = 0

            # Uses intelligent_search() to determine desired turn direction and motor power
            if measurement_time[0] != 0: # Condition here so there is no divide by zero case in the intelligent_search() method
                power, direction = intelligent_search(past_position_relative, past_coordinates, current_distance,measurement_time)
            else:
                power = 80
                direction = 0

            if direction == 0:  # Turn left
                pwm_motor_2.ChangeDutyCycle(power)
                time.sleep(0.05)
                GPIO.output(motor_1_forward, GPIO.LOW)
                GPIO.output(motor_1_backward, GPIO.LOW)
                GPIO.output(motor_2_forward, GPIO.HIGH)
                GPIO.output(motor_2_backward, GPIO.LOW)
            else:  # Turn Right
                pwm_motor_1.ChangeDutyCycle(power)
                time.sleep(0.05)
                GPIO.output(motor_1_forward, GPIO.HIGH)
                GPIO.output(motor_1_backward, GPIO.LOW)
                GPIO.output(motor_2_forward, GPIO.LOW)
                GPIO.output(motor_2_backward, GPIO.LOW)

            time.sleep(0.4) # Runs the macros for 400 ms before turning off the motors

    # Turns off motors before processing next image
    GPIO.output(motor_1_forward, GPIO.LOW)
    GPIO.output(motor_1_backward, GPIO.LOW)
    GPIO.output(motor_2_forward, GPIO.LOW)
    GPIO.output(motor_2_backward, GPIO.LOW)

    # If key 'q' is pressed, for loop is exited and program terminates
    if cv2.waitKey(0.3) & 0xFF == ord('q'):
        break


# Cleans up program and resets hardware
kill_program()

### MAIN BLOCK ENDS ###



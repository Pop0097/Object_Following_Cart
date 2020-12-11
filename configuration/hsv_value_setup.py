"""

NOTE: to make this file run, move it to the root directory. For some reason it doesn't work when it's in this directory.

Optimal HSV Values for Red Ball:
- Hue_min: 007
- Hue_max: 179
- Sat_min: 000
- Sat_max: 255
- Val_min: 000
- Val_max: 255

Template:
- Hue_min:
- Hue_max:
- Sat_min:
- Sat_max:
- Val_min:
- Val_max:

"""

import cv2
import numpy as np

def placeholder_method(empty_variable): # empty method
    pass 

# Creates a window that modifies the HSV values we are looking for.
cv2.namedWindow("TrackBar")
cv2.resizeWindow("TrackBar", 1000, 2000)
# Getting the hue (H) of the colour
cv2.createTrackbar("Hue Min", "TrackBar", 0, 179, placeholder_method)
cv2.createTrackbar("Hue Max", "TrackBar", 179, 179, placeholder_method)
# # Getting the saturation (S) of the colour
cv2.createTrackbar("Sat Min", "TrackBar", 0, 255, placeholder_method)
cv2.createTrackbar("Sat Max", "TrackBar", 255, 255, placeholder_method);
# # Getting the value (V) of the colour
cv2.createTrackbar("Val Min", "TrackBar", 0, 255, placeholder_method)
cv2.createTrackbar("Val Max", "TrackBar", 255, 255, placeholder_method)

image_path = 'image/ball.jpg'

while True:
    image = cv2.imread(image_path)  # Gets image from directory
    image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # Converts the image from blue green red (BGR) to HSV

    # Get values form the track bars
    hue_min = cv2.getTrackbarPos("Hue Min", "TrackBar")
    hue_max = cv2.getTrackbarPos("Hue Max", "TrackBar")
    sat_min = cv2.getTrackbarPos("Sat Min", "TrackBar")
    sat_max = cv2.getTrackbarPos("Sat Max", "TrackBar")
    val_min = cv2.getTrackbarPos("Val Min", "TrackBar")
    val_max = cv2.getTrackbarPos("Val Max", "TrackBar")

    # Arrays store the minimum and maximum values for each HSV value
    minimum_values = np.array([hue_min, sat_min, val_min])
    maximum_values = np.array([hue_max, sat_max, val_max])

    # Creates an image that displays the HSV colour indicated by the track bars
    HSV_specific_image = cv2.inRange(image_HSV, minimum_values, maximum_values)

    # Resizes images
    HSV_image_resized = cv2.resize(image_HSV, (1000, 600))
    HSV_specific_image_resized = cv2.resize(HSV_specific_image, (1000, 600))

    # Displays images
    cv2.imshow("HSV", HSV_specific_image_resized)
    cv2.imshow("HSV Specific", HSV_image_resized)

    if cv2.waitKey(1) & 0xFF == ord('q'): # If 'q' is pressed, code ends
        break


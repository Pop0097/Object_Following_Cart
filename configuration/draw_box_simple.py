# NOTE: to make this file run, move it to the root directory. For some reason it doesn't work when it's in this directory.

"""
This file is just made to get an idea of how to get the coordinates of the ball and draw a rectangle on the image.
"""

import cv2
import numpy as np

image_path = 'image/ball.jpg'
image = cv2.imread(image_path) # Gets image from directory
image = cv2.resize(image, (1000, 600))
image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # Converts the image from blue green red (BGR) to HSV

# Arrays store the minimum and maximum values for each HSV value. Refer to "hsv_value_setup.py"
minimum_values = np.array([7, 0, 0])
maximum_values = np.array([179, 255, 255])

image_HSV_specific = cv2.inRange(image_HSV, minimum_values, maximum_values)

contours, hierarchy = cv2.findContours(image_HSV_specific,  cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) # Finds contours in image (which can be used to identify shapes)

ball_coordinates = [0.0, 0.0]

for contour in contours:
    shape_area = cv2.contourArea(contour)
    if shape_area > 500: # If shape area is larger than 500 (makes sure random detectoins are not processed)
        perimeter = cv2.arcLength(contour, True)
        # print(shape_area)
        verticies = cv2.approxPolyDP(contour, 0.03*perimeter, True) # Adjust values as needed
        # print(len(verticies))
        if len(verticies) > 4: # Circles will still have verticies. If we can find a good balance then we are solid
            x, y, width, height = cv2.boundingRect(verticies) # Gets parameters for a rectangle around object
            ball_coordinates = [x, y]
            print(x, " ", y, " ", width, " ", height)
            cv2.rectangle(image, (x, y), (x+width, y+height), (0, 255, 0), 3) # Draws rectangle on image

while True:
    cv2.imshow("Image", image)
    # print(ball_coordinates)

    if cv2.waitKey(1) & 0xFF == ord('q'): # If 'q' is pressed, code ends
        break


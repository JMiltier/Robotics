# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2

# define the lower and upper boundaries of the colors in the HSV color space
# lower = {'red':(165, 50, 50), 'green':(36, 50, 50), 'blue':(97, 50, 50)}
# upper = {'red':(180,255,255), 'green':(75,255,255), 'blue':(127,255,255)}
lower = {'red':(165, 50, 50), 'green':(36, 50, 50), 'blue':(97, 50, 50), 'yellow':(15,50,50)}
upper = {'red':(180,255,255), 'green':(75,255,255), 'blue':(127,255,255), 'yellow':(35,255,255)}

# camera reference
camera = cv2.VideoCapture(0)
# ajustments for camera settings (Microsoft camera)
# brightness = 20
# camera.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
# exposure = 10
# camera.set(cv2.CAP_PROP_EXPOSURE, exposure)
# contrast = 20
# camera.set(cv2.CAP_PROP_CONTRAST, contrast)
# gain = 300
# camera.set(cv2.CAP_PROP_GAIN, gain)

object_color = "NULL"

# keep looping
while object_color == "NULL":
    # grab the current frame
    (grabbed, frame) = camera.read()

    # resize the frame, blur it, and convert it to the HSV color space
    frame = cv2.resize(frame, (1000,640))

    # webcam size to reshape
    height, width, channels = frame.shape
    # scale to zoom in (percentage), + to zoom in, - to zoom out
    scale = 100

    # crop prep
    centerX, centerY = int(height/2),int(width/2)
    radiusX, radiusY = int(scale*height/100), int(scale*width/100)

    minX, maxX = centerX-radiusX, centerX+radiusX
    minY, maxY = centerY-radiusY, centerY+radiusY

    cropped = frame[minX:maxX, minY:maxY]
    resize_crop = cv2.resize(cropped, (width, height))

    blurred = cv2.GaussianBlur(resize_crop, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    #for each color in dictionary check object in frame
    for key, value in upper.items():
        # construct a mask for the color from dictionary`1, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        kernel = np.ones((9,9),np.uint8)
        mask = cv2.inRange(hsv, lower[key], upper[key])
        maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernel)

        # find contours in the mask and initialize the current
        # (x, y) center of the object
        cnts = cv2.findContours(maskClose.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cv2.drawContours(resize_crop, cnts, -1, (0,255,255), 1)
        center = None

        for i in range(len(cnts)):
            x,y,w,h=cv2.boundingRect(cnts[i])
            cv2.rectangle(resize_crop,(x,y),(x+w,y+h),(255,255,100), 2)

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing object and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((obj_x, obj_y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size. Correct this value for your obect's size
            if radius > 0.5:
                object_color = key
                object_location = (obj_x, obj_y) #position of the center of the object in pixels (x,y)
                print("Object Color: %s" % object_color)
                print("Distance: ", object_location)

    cv2.imshow("Mask", maskClose)
    cv2.imshow("Cropped Frame", resize_crop)
    # cv2.imshow("Frame", frame)
    key = cv2.waitKey(5) & 0xFF

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()

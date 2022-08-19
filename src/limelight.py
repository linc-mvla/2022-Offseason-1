import cv2
import numpy as np
import random
import copy
    
# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):

    image[image == 0] = 1
    image[(image[:, :, 1].astype(float) / image[:, :, 2].astype(float)) < 2] = [0, 0, 0]

    img_threshold = cv2.inRange(image, (10, 54, 10), (255, 255, 255))

    contours, _ = cv2.findContours(img_threshold, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
    largestContour = np.array([[]])

    if len(contours) > 0:

        for contour in contours:
            epsilon = 0.1 * cv2.arcLength(contour, True)
            poly = cv2.approxPolyDP(contour, epsilon, True)

            llpython.append(len(poly)) #num corners

            for p in poly:
                if ((cv2.contourArea(contour) >= 1.0)):
                    llpython.append(p[0][0])
                    llpython.append(p[0][1])
                    cv2.circle(image, (p[0][0], p[0][1]), radius=1, color = (0, 0, 255), thickness=-1)

            M = cv.moments(contour)
            if M['m00'] != 0:
                llpython.append(int(M['m10']/M['m00']))
                llpython.append(int(M['m01']/M['m00']))

       
    # make sure to return a contour,
    # an image to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    return largetsContour, image, llpython
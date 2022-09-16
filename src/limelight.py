import cv2
import numpy as np
import random
import copy
    
# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):

    image[(image[:, :, 0]) == 0] = 1
    image[(image[:, :, 2]) == 0] = 1
    image[(image[:, :, 1].astype(float) / image[:, :, 2].astype(float)) < 1.3] = [0, 0, 0] #get rid of not green enough pixels (red)
    image[(image[:, :, 1].astype(float) / image[:, :, 0].astype(float)) < 1] = [0, 0, 0] #get rid of not green enough pixels (blue)

    img_threshold = cv2.inRange(image, (10, 54, 10), (255, 255, 255))

    contours, _ = cv2.findContours(img_threshold, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
    largestContour = np.array([[]])
    llpython = []

    if len(contours) > 0:
      #  cv2.drawContours(image, contours, -1, (100,0,255), 1)

        for contour in contours:
            #could do a thing where epsilon depends on contour area
            epsilon = 0.07 * cv2.arcLength(contour, True) #0.1
            poly = cv2.approxPolyDP(contour, epsilon, True)

            if ((cv2.contourArea(contour) <= 120)):
                llpython.append(len(poly)) #num corners

                for p in poly:
                    llpython.append(p[0][0])
                    llpython.append(p[0][1])
                    cv2.circle(image, (p[0][0], p[0][1]), radius=1, color = (0, 0, 255), thickness=-1)

                M = cv2.moments(contour)
                if M['m00'] != 0:
                    print("center:" + str(int(M['m10']/M['m00'])) + " " + str(int(M['m01']/M['m00'])))
                    llpython.append(int(M['m10']/M['m00']))
                    llpython.append(int(M['m01']/M['m00']))
                    cv2.circle(image, (int(M['m10']/M['m00']), int(M['m01']/M['m00'])), radius=1, color=(255, 0, 100), thickness=-1)
                else:
                    llpython.append(-1)
                    llpython.append(-1)

       
    # make sure to return a contour,
    # an image to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    return largestContour, image, llpython

#FOR RUNNING ON COMPUTER - COMMENT OUT WHEN RUNNING ON ROBOT
image = cv2.imread("limelight_test1.jpg")
_, img, llpython = runPipeline(image, [])
cv2.imshow("image", img)
print(llpython)
while True:
    image = cv2.imread("limelight_test1.jpg")
    cv2.imshow("image", img)
    if (cv2.waitKey(30) == 27): #27 is escape
        break 
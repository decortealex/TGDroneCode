import cv2
import numpy as np
import logging
from bebop import Bebop

logging.basicConfig(level=logging.DEBUG)

wnd = None
def video_frame(frame):
   plateLower = np.array([160, 170, 190], dtype="uint8")
   plateUpper = np.array([225, 230, 250], dtype="uint8")

   # determine which pixels fall within the blue boundaries
   # and then blur the binary image
   blue = cv2.inRange(frame, plateLower, plateUpper)
   blue = cv2.GaussianBlur(blue, (3, 3), 0)

   # find contours in the image
   a, cnts, hierarchy = cv2.findContours(blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

   # check to see if any contours were found
   if len(cnts) > 0:
       # sort the contours and find the largest one -- we
       # will assume this contour correspondes to the area
       # of my phone
       cnt = sorted(cnts, key=cv2.contourArea, reverse=True)[0]

       # compute the (rotated) bounding box around then
       # contour and then draw it
       rect = np.int32(cv2.boxPoints(cv2.minAreaRect(cnt)))
       print(rect);

       cv2.drawContours(frame, [rect], -1, (0, 255, 0), 2)

   # show the frame and the binary image
   cv2.imshow("Drone", frame)
   cv2.waitKey(10)
   cv2.imshow("Binary", blue)

def video_start():
    print("Starting video...")
    cv2.namedWindow("Drone")
    cv2.namedWindow("Binary")

def video_end():
    print("Ending video...")
    cv2.destroyWindow("Drone")
    cv2.destroyWindow("Binary")
    # Have to send waitKey several times on Unix to make window disappear
    for i in range(1, 5):
        cv2.waitKey(1)

print("Connecting to drone..")
drone = Bebop()
drone.video_callbacks(video_start, video_end, video_frame)
drone.videoEnable()
print("Connected.")
for i in xrange(10000):
    drone.update( );

print("Battery:", drone.battery)

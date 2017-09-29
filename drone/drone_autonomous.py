import pygame
# so we can use np to reference numpy
import numpy as np
# so we can easily add in whatever we need from the other codes, seeing as winston uses this.
import numpy as numpy
import cv2
from commands import *
import logging
from bebop import Bebop
from bardecoder import Decoder
from bardecoder import Barcode

logging.basicConfig(level=logging.DEBUG)

wnd = None
cnt = 0
f = open( "./images/video.h264", "wb" )
# frame =
# we need this to actually use this for the rectangle stuff
# if we're using that for movement.
def video_frame(frame):
    if drone.frameWidth == 0:
        drone.frameWidth = numpy.size(frame, 1)
    if drone.frameHeight == 0:
        drone.frameHeight = numpy.size(frame, 0)

    # Initialize variables to compare the current frame to
    if drone.thisFrame is None:
        drone.lastFrame = frame
    else:
        drone.lastFrame = drone.thisFrame
    drone.thisFrame = frame

    blueLower = np.array([100, 67, 0], dtype="uint8")
    blueUpper = np.array([255, 128, 50], dtype="uint8")

    # determine which pixels fall within the blue boundaries
    # and then blur the binary image

    blue = cv2.inRange(frame, blueLower, blueUpper)
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
        print("rect:",rect)
        print("cnt:", cnt)

        cv2.drawContours(frame, [rect], -1, (0, 255, 0), 2)
        xMid = (rect[4] - rect[3])/2
        yMid = (rect[1] - rect[4])/2
        rectMid = [xMid, yMid]


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

def scale(value, scaler):
    if abs(value) < 0.03:
        return 0
    return value * scaler

print("Connecting to drone..")
drone = Bebop()
drone.video_callbacks(video_start, video_end, video_frame)
drone.videoEnable()
print("Connected.")
for i in xrange(10000):
    drone.update();
print("Battery:", drone.battery)

pygame.init()
size = [100, 100]
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Drone Teleop")

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# -------- Main Program Loop -----------

MAX_SPEED = 70

while not done:
    try:
        # EVENT PROCESSING STEP
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop
        # if cv2.inRange(frame, blueLower, blueUpper):
        #     drone.land()
        # drone.takeoff()
        # drone.flyToAltitude(2.5)
        #     drone.update()

    except:
        print("Error")
        if drone.flyingState is None or drone.flyingState == 1: # if taking off then do emegency landing
            drone.emergency()
            drone.land()


# Close the window and quit.
pygame.quit()

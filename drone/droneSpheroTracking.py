import cv2
from bebop import Bebop
import pygame
import time
from commands import *
import numpy
import logging

def mergeSort(lst):
    if len(lst) <= 1:
        return lst
    mid = len(lst) // 2
    left = mergeSort(lst[:mid])
    right = mergeSort(lst[mid:])
    return merge(left, right)

def merge(left, right):
    if not left:
        return right
    if not right:
        return left
    if left[0] < right[0]:
        return [left[0]] + merge(left[1:], right)
    return [right[0]] + merge(left, right[1:])

cnt = 0
f = open( "./images/video.h264", "wb" )

logging.basicConfig(level=logging.DEBUG)

wnd = None
def video_frame(frame):
    # Initialize the frame size for drone adjustment
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

    # # Convert frames to grayscale and blur them
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # gray = cv2.GaussianBlur(gray, (21, 21), 0)
    #
    # grayLastFrame = cv2.cvtColor(drone.lastFrame, cv2.COLOR_BGR2GRAY)
    # grayLastFrame = cv2.GaussianBlur(grayLastFrame, (21, 21), 0)
    #
    # # compute the absolute difference between the current frame and the last frame
    # frameDelta = cv2.absdiff(grayLastFrame, gray)

    ret, thresh = cv2.threshold(frame, 127, 255, cv2.THRESH_BINARY)
    # edges = cv2.cvtColor(edges, cv2.COLOR_BGR2GRAY)

    # Find edges after motion detection
    edges = cv2.Canny(thresh, drone.minEdgeVal, drone.maxEdgeVal)

    if drone.pictureBoolean:
        drone.pictureBoolean = False
        cv2.imwrite("saved_image.jpg", edges)

    # Find sphero using circles
    if drone.findSphero:
        # Find circles after detecting edges
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1.2, 5,
                                   param1=50, param2=30, minRadius=drone.minCircleRadius, maxRadius=drone.maxCircleRadius)
        # circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1.2, 10, minRadius=drone.minCircleRadius, maxRadius=drone.maxCircleRadius)

        if circles is not None:
            circles = numpy.uint16(numpy.around(circles))
            listX = []
            listY = []
            listR = []

            for i in circles[0, :]:
                # # draw the outer circle
                # cv2.circle(edges, (i[0], i[1]), i[2], (255, 255, 255), 2)
                # # draw the center of the circle
                # cv2.circle(edges, (i[0], i[1]), 2, (255, 255, 255), 3)

                # Save the centers and radii
                listX.append(i[0])
                listY.append(i[1])
                listR.append(i[2])
                # print("Edges circle center at: " + str(i[0]) + ", " + str(i[1]))

            # Sort the centers and radii and print/draw the median
            sortedX = mergeSort(listX)
            sortedY = mergeSort(listY)
            sortedR = mergeSort(listR)

            medianX = sortedX[len(sortedX) // 2]
            medianY = sortedY[len(sortedY) // 2]
            medianR = sortedR[len(sortedR) // 2]

            drone.objectCenterX = medianX
            drone.objectCenterY = medianY

            cv2.circle(edges, (medianX, medianY), medianR, (255,255,255), 2)
            cv2.circle(edges, (medianX, medianY), 2, (255,255,255), 2)
            # print("Median edges circle center: " + str(medianX) + ", " + str(medianY) + " with radius " + str(medianR))

            drone.sinceLastSphero = 0
            drone.foundCircle = True
        else:
            # Fake a circle in the center if none found
            drone.objectCenterX = drone.frameWidth >> 1
            drone.objectCenterY = drone.frameHeight >> 1

            drone.sinceLastSphero += 1
            drone.foundCircle = False
    else:
        # Fake a circle in the center if none found
        drone.objectCenterX = drone.frameWidth >> 1
        drone.objectCenterY = drone.frameHeight >> 1
        drone.foundCircle = False

    # Find sphero using blobs if no circles found
    if drone.findSphero and not drone.foundCircle:
        kernel = numpy.ones((5, 5), numpy.uint8)
        edges = cv2.dilate(edges, kernel, iterations=1)
        edges = cv2.erode(edges, kernel, iterations=1)

        params = cv2.SimpleBlobDetector_Params()

        # Filter by Circularity
        # params.filterByCircularity = True
        # params.minCircularity = 0.6

        # Filter by Area.
        # params.filterByArea = True
        # params.minArea = 16

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(edges)

        if keypoints is not None:
            listX = []
            listY = []
            listR = []

            for keypoint in keypoints:
                # # draw the outer circle
                # cv2.circle(edges, (i[0], i[1]), i[2], (255, 255, 255), 2)
                # # draw the center of the circle
                # cv2.circle(edges, (i[0], i[1]), 2, (255, 255, 255), 3)

                # Save the centers and radii
                # print point.pt[0]
                listX.append(int(keypoint.pt[0]))
                listY.append(int(keypoint.pt[1]))
                listR.append(int(keypoint.size / 2))
                # print("Edges circle center at: " + str(i[0]) + ", " + str(i[1]))
                # print keypoint.pt

            if len(listX) > 0 and len(listY) > 0 and len(listR) > 0:
                # Sort the centers and radii and print/draw the median
                sortedX = mergeSort(listX)
                sortedY = mergeSort(listY)
                sortedR = mergeSort(listR)

                # print sortedX
                medianX = sortedX[len(sortedX) // 2]
                medianY = sortedY[len(sortedY) // 2]
                medianR = sortedR[len(sortedR) // 2]

                drone.objectCenterX = medianX
                drone.objectCenterY = medianY

                cv2.circle(edges, (medianX, medianY), medianR, (255,255,255), 2)
                cv2.circle(edges, (medianX, medianY), 2, (255,255,255), 2)
                # print("Median edges circle center: " + str(medianX) + ", " + str(medianY) + " with radius " + str(medianR))

                drone.sinceLastSphero = 0
            else:
                # Fake a circle in the center if none found
                drone.objectCenterX = drone.frameWidth >> 1
                drone.objectCenterY = drone.frameHeight >> 1

                drone.sinceLastSphero += 1
        else:
            # Fake a circle in the center if none found
            drone.objectCenterX = drone.frameWidth >> 1
            drone.objectCenterY = drone.frameHeight >> 1

            drone.sinceLastSphero += 1
    elif not drone.foundCircle:
        # Fake a circle in the center if none found
        drone.objectCenterX = drone.frameWidth >> 1
        drone.objectCenterY = drone.frameHeight >> 1
        drone.sinceLastSphero = 0

    cnt += 1
    cv2.imshow("Drone", frame)
    # cv2.imshow("Motion Detection", frameDelta)
    cv2.imshow("Threshold Edges", edges)
    cv2.imshow("Threshold", thresh)
    cv2.waitKey(10)

def video_start():
    print("Starting video...")
    cv2.namedWindow("Drone")
    cv2.namedWindow("Threshold Edges")
    cv2.namedWindow("Threshold")

def video_end():
    print("Ending video...")
    cv2.destroyWindow("Drone")
    cv2.destroyWindow("Threshold Edges")
    cv2.destroyWindow("Threshold")
    # Have to send waitKey several times on Unix to make window disappear
    for i in range(1, 5):
        cv2.waitKey(1)

def scale(value, scaler):
    if abs(value) < 0.03:
        return 0
    return value * scaler

def clip(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


# Video variables
f = open( "./images/video.h264", "wb" )
cnt = 0
frames = 0
lastFrames = 0

print("Connecting to drone...")
drone = Bebop()
drone.trim()
drone.video_callbacks(video_start, video_end, video_frame)
drone.videoEnable()
drone.minEdgeVal = 30
drone.maxEdgeVal = 50
print("Connected.")

pygame.init()
size = [100, 100]
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Drone Teleop")

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initializes joystick
if pygame.joystick.get_count() == 0:
    print("No joysticks found")
    done = True
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Initialized %s" % (joystick.get_name()))
    print("Number of buttons %d. Number of axis %d, Number of hats %d" %
          (joystick.get_numbuttons(), joystick.get_numaxes(),
           joystick.get_numhats()))

MAX_SPEED = 40

tilt = 0
tiltMin = -70
tiltMax = 40

pan = 0
panMin = -40
panMax = 40

secondsCounter = 0
frames = 0

printCounter = 0
spheroMoveCounter = 0

lastTime = time.time()

# -------- Main Program Loop -----------
while not done:
    try:
        userMovement = False

        # EVENT PROCESSING STEP
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop

        # Displays battery every 5 seconds
        nowTime = time.time()
        if (nowTime - lastTime) > 1:
            secondsCounter += 1
            lastTime = nowTime

            if secondsCounter % 5 == 0:
                print("Battery: " + str(drone.battery))

        # Deltas for controlling the camera
        tiltDelta = 0
        panDelta = 0

        if joystick.get_button(1):
            drone.pictureBoolean = True

        # A and Back to emergency land
        if joystick.get_button(0) == 1 and joystick.get_button(6) == 1:
            drone.emergency()

        # Back to land
        if joystick.get_button(6) == 1:
            print("Landing...")
            if drone.flyingState is None or drone.flyingState == 1: # if taking off then do emergency landing
                drone.emergency()
            drone.land()

        # Start to takeoff
        if joystick.get_button(7) == 1:
            if drone.flyingState is 0:
                drone.takeoff()
            # drone.takeoff()

        # --- Flying ---
        # Power values
        roll = scale(joystick.get_axis(0), MAX_SPEED)
        pitch = -scale(joystick.get_axis(1), MAX_SPEED)
        yaw = scale(joystick.get_axis(3), MAX_SPEED)
        gaz = -scale(joystick.get_axis(4), MAX_SPEED)

        if roll != 0:
            userMovement = True

        if pitch != 0:
            userMovement = True

        if yaw != 0:
            userMovement = True

        if gaz != 0:
            userMovement = True

        if joystick.get_button(3) == 1 and not drone.findSphero:
            print("Start finding sphero")
            drone.findSphero = True
            drone.moveScaler = .25

            # Upper and lower bounds for circle pixel radius
            drone.minCircleRadius = 5
            drone.maxCircleRadius = 20

        if joystick.get_button(2) and drone.findSphero:
            print("Stop finding sphero")
            drone.findSphero = False

        # --- Move camera ---

        # Triggers to tilt
        if not(joystick.get_button(0) == 1) and joystick.get_axis(2) > 0.05:
            tiltDelta = scale(joystick.get_axis(2), -10)

        if not(joystick.get_button(0) == 1) and joystick.get_axis(5) > 0.05:
            tiltDelta = scale(joystick.get_axis(5), 10)

        tilt = clip(tilt + tiltDelta, tiltMin, tiltMax)
        pan = clip(pan + panDelta, panMin, panMax)

        # Reset camera on B
        if joystick.get_button(1) == 1:
            tilt = 0
            pan = 0

        if joystick.get_button(5) == 1:
            print("Flying to altitude: 1.5")
            drone.flyToAltitude(1.25)

        drone.moveCamera(tilt, pan)

        # All movement updated here
        if userMovement:
            drone.update(cmd=movePCMDCmd(True, roll, pitch, yaw, gaz))
        elif drone.findSphero:
            roll = (drone.objectCenterX - (drone.frameWidth >> 1)) * drone.moveScaler
            pitch = ((drone.frameHeight >> 1) - drone.objectCenterY) * drone.moveScaler

            roll = clip(roll, -50, 50)
            pitch = clip(pitch, -50, 50)

            # print("Finding Sphero")
            # print(roll)
            # print(pitch)
            spheroMoveCounter += 1

            # if (drone.sinceLastSphero % 3) == 0 and drone.altitude >= 1:
            #     drone.update(cmd=movePCMDCmd(True, 0, 0, 0, -10))
            #
            # if drone.altitude < 1:
            #     drone.flyToAltitude(1.5)

            # Adjustment test
            if (spheroMoveCounter % 3) == 0:
                roll *= .4
                pitch *= .4

            drone.update(cmd=movePCMDCmd(True, roll, pitch, 0, 0))
        else:
            drone.hover()

        # Limit to 20 frames per second
        clock.tick(20)
    except:
        print("Error")
        if drone.flyingState is None or drone.flyingState == 1:
            # if taking off then do emergency landing
            drone.emergency()
        drone.land()
        done = True

# Close the window and quit.
pygame.quit()
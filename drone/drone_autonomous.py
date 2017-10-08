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
decoder = Decoder()
f = open( "./images/video.h264", "wb" )
frames = 0
lastFrames = 0

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

# frame =
# we need this to actually use this for the rectangle stuff
# if we're using that for movement.
def video_frame(frame):
    # if drone.frameWidth == 0:
    #     drone.frameWidth = numpy.size(frame, 1)
    # if drone.frameHeight == 0:
    #     drone.frameHeight = numpy.size(frame, 0)
    #
    # # Initialize variables to compare the current frame to
    # if drone.thisFrame is None:
    #     drone.lastFrame = frame
    # else:
    #     drone.lastFrame = drone.thisFrame
    # drone.thisFrame = frame
    #
    # plateLower = np.array([160, 170, 190], dtype="uint8")
    # plateUpper = np.array([225, 230, 250], dtype="uint8")
    #
    # # determine which pixels fall within the blue boundaries
    # # and then blur the binary image
    #
    # blue = cv2.inRange(frame, plateLower, plateUpper)
    # blue = cv2.GaussianBlur(blue, (3, 3), 0)
    #
    # # find contours in the image
    # a, cnts, hierarchy = cv2.findContours(blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #
    # # check to see if any contours were found
    # if len(cnts) > 0:
    #     # sort the contours and find the largest one -- we
    #     # will assume this contour correspondes to the area
    #     # of my phone
    #     cnt = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
    #     # compute the (rotated) bounding box around then
    #     # contour and then draw it
    #     rect = np.int32(cv2.boxPoints(cv2.minAreaRect(cnt)))
    #     # print("rect:",rect)
    #     # print("cnt:", cnt)
    #
    #     cv2.drawContours(frame, [rect], -1, (0, 255, 0), 2)
    #
    #     drone.objectCenterX = (rect[1][0] + rect[2][0])/2
    #     drone.objectCenterY = (rect[3][1] + rect[2][1])/2
    barcodes = decoder.decode(frame)
    if len(barcodes) > 0:
        for barcode in barcodes:
            # do something useful with results
            print('decoded', barcode.type, 'symbol', barcode.location,
                  '"%s"' % barcode.value)
            min_x = min(barcode.location[0][0], barcode.location[1][0], barcode.location[2][0], barcode.location[3][0])
            max_x = max(barcode.location[0][0], barcode.location[1][0], barcode.location[2][0], barcode.location[3][0])

            min_y = min(barcode.location[0][1], barcode.location[1][1], barcode.location[2][1], barcode.location[3][1])
            max_y = max(barcode.location[0][1], barcode.location[1][1], barcode.location[2][1], barcode.location[3][1])

            drone.objectCenterX = int( (min_x + max_x) * 0.5)
            drone.objectCenterY = int( (min_y + max_y) * 0.5)

            height, width, _ = frame.shape

            color = (0, 0 , 255)
            text = "Centered"
            if abs(width * 0.5 - drone.objectCenterX) < 50:
                color = (0, 255, 0)
            elif drone.objectCenterX - width * 0.5 > 50:
                color = (0, 0, 255)
                text = "Go Right"
            elif drone.objectCenterX - width * 0.5 < 50:
                color = (255, 0, 0)
                text = "Go Left"

            cv2.line(frame, barcode.location[0], barcode.location[1], color=color, thickness=2)
            cv2.line(frame, barcode.location[1], barcode.location[2], color=color, thickness=2)
            cv2.line(frame, barcode.location[2], barcode.location[3], color=color, thickness=2)
            cv2.line(frame, barcode.location[0], barcode.location[3], color=color, thickness=2)
            cv2.circle(frame, (drone.objectCenterX, drone.objectCenterY), 3, color=color, thickness=2)
            cv2.putText(frame, org=(width - 100, height - 100), text=text, color=color, fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1,)
            break


    cv2.imshow("Drone", frame)
    cv2.waitKey(10)
    cv2.imshow("Binary", barcodes)


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

def clip(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value

print("Connecting to drone..")
drone = Bebop()
drone.video_callbacks(video_start, video_end, video_frame)
drone.videoEnable()
print("Connected.")
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
# drone.flyToAltitude(2.25)

MAX_SPEED = 80
# drone.moveScaler = .25

tilt = 0
tilMin = -70
tiltMax = 40

pan = 0
panMin = -40
panMax = 40

secondsCounter = 0

printCounter = 0
plateFindCounter = 0

lastTime = time.time()

while not done:
    try:
        userMovement = False
#         EVENT PROCESSING STEP
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
        #   --- Flying ---
        # Power values
        roll =  scale(joystick.get_axis(0), MAX_SPEED)
        pitch = -scale(joystick.get_axis(1), MAX_SPEED)
        yaw =   scale(joystick.get_axis(3), MAX_SPEED)
        gaz =   -scale(joystick.get_axis(4), MAX_SPEED)

        if roll != 0:
            userMovement = True

        if pitch != 0:
            userMovement = True

        if yaw != 0:
            userMovement = True

        if gaz != 0:
            userMovement = True

        if joystick.get_button(0) == 1 and not drone.findPlate:
            executing_command = True
            print("Button 0 pressed")
            drone.findPlate = True
            drone.moveScaler = .25

        if joystick.get_button(1) == 1 and drone.findPlate:
            executing_command = True
            print("Button 1 pressed")
            drone.findPlate = False

        if joystick.get_button(2) == 1:
            executing_command = True
            print("Button 2 pressed")


        if joystick.get_button(6) == 1:
            executing_command = True
            print("Landing")
            drone.land()

        if joystick.get_button(7) == 1:
            executing_command = True
            print("Taking off")
            drone.takeoff()

        if joystick.get_button(8) == 1:
            executing_command = True
            print("Button 8 pressed")
            drone.emergency()

        if userMovement:
            drone.update(cmd=movePCMDCmd(True, roll, pitch, yaw, gaz))

        elif drone.findPlate:
            roll = (drone.objectCenterX - (drone.frameWidth >> 1)) * drone.moveScaler
            pitch = ((drone.frameHeight >> 1) - drone.objectCenterY) * drone.moveScaler

            roll = clip(roll, -50, 50)
            pitch = clip(pitch, -50, 50)

            plateFindCounter += 1

            if (plateFindCounter % 3) == 0:
                roll *= .4
                pitch*= .4

            drone.update(cmd=movePCMDCmd(True, roll, pitch, 0, 0))

        else:
            drone.hover()

        clock.tick(20)

    except:
        print("Error")
        if drone.flyingState is None or drone.flyingState == 1: # if taking off then do emegency landing
            drone.emergency()
        drone.land()


# Close the window and quit.
pygame.quit()
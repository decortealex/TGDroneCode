import pygame
import cv2
from commands import *
from bebop import Bebop
from bardecoder import Decoder
import logging

logging.basicConfig(level=logging.DEBUG)

decoder = Decoder()

wnd = None
def video_frame(frame):
    barcodes = decoder.decode(frame)
    if len(barcodes) > 0:
        for barcode in barcodes:
            print('decoded', barcode.type, 'symbol', barcode.location,
                  '"%s"' % barcode.value)
            min_x = min(barcode.location[0][0], barcode.location[1][0], barcode.location[2][0],
                        barcode.location[3][0])
            max_x = max(barcode.location[0][0], barcode.location[1][0], barcode.location[2][0],
                        barcode.location[3][0])

            min_y = min(barcode.location[0][1], barcode.location[1][1], barcode.location[2][1],
                        barcode.location[3][1])
            max_y = max(barcode.location[0][1], barcode.location[1][1], barcode.location[2][1],
                        barcode.location[3][1])

            center_x = int((min_x + max_x) * 0.5)
            center_y = int((min_y + max_y) * 0.5)

            height, width, _ = frame.shape

            color = (0, 0, 255)
            text = "Centered"
            if abs(width * 0.5 - center_x) < 50:
                color = (0, 255, 0)
            elif center_x - width * 0.5 > 50:
                color = (0, 0, 255)
                text = "Go Right"

            elif center_x - width * 0.5 < 50:
                color = (255, 0, 0)
                text = "Go Left"

            cv2.line(frame, barcode.location[0], barcode.location[1], color=color, thickness=2)
            cv2.line(frame, barcode.location[1], barcode.location[2], color=color, thickness=2)
            cv2.line(frame, barcode.location[2], barcode.location[3], color=color, thickness=2)
            cv2.line(frame, barcode.location[0], barcode.location[3], color=color, thickness=2)
            cv2.circle(frame, (center_x, center_y), 3, color=color, thickness=2)
            cv2.putText(frame, org=(width - 100, height - 100), text=text, color=color,
                        fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1, )

            if barcode.value == "TechGarage":
                drone.update(cmd=movePCMDCmd(True, 0, 0, 0, 0))
                drone.land()
            break

    cv2.imshow("Drone", frame)
    cv2.waitKey(10)

def video_start():
    print("Starting video...")
    cv2.namedWindow("Drone")

def video_end():
    print("Ending video...")
    cv2.destroyWindow("Drone")
    # Have to send waitKey several times on Unix to make window disappear
    for i in range(1, 5):
        cv2.waitKey(1)


print("Connecting to drone..")
drone = Bebop()
drone.video_callbacks(video_start, video_end, video_frame)
drone.videoEnable()
drone.moveCamera(-90, 0)
print("Connected.")

pygame.init()
size = [100, 100]
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Drone Teleop")

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

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


# -------- Main Program Loop -----------

MAX_SPEED = 50

while not done:

    # EVENT PROCESSING STEP
    for event in pygame.event.get():  # User did something
        if event.type == pygame.QUIT:  # If user clicked close
            done = True  # Flag that we are done so we exit this loop

    executing_command = False

    if joystick.get_button(0) == 1:
        executing_command = True
        print("Landing...")
        if drone.flyingState is None or drone.flyingState == 1: # if taking off then do emegency landing
            drone.emergency()
        drone.land()

    if joystick.get_button(3) == 1:
        executing_command = True
        print("Taking off..")
        drone.takeoff()

    if joystick.get_button(7) == 1:
        executing_command = True
        drone.update(cmd=movePCMDCmd(True, 0, MAX_SPEED, 0, 0))  # start going forward slowly

    drone.update(cmd=None)

# Close the window and quit.
pygame.quit()

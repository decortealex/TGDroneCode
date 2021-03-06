import pygame
import cv2
from commands import *
import logging
from bebop import Bebop
from bardecoder import Decoder
from bardecoder import Barcode

logging.basicConfig(level=logging.DEBUG)
decoder = Decoder()
wnd = None
def video_frame(frame):
    barcodes = decoder.decode(frame)
    if len(barcodes) > 0:
        for barcode in barcodes:
            # do something useful with results
            if barcode.value == "9" or barcode.value == "8" or barcode.value == "7":
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

                color = (0, 255, 0)
                text = "Valuable Cans"

                cv2.line(frame, barcode.location[0], barcode.location[1], color=color, thickness=2)
                cv2.line(frame, barcode.location[1], barcode.location[2], color=color, thickness=2)
                cv2.line(frame, barcode.location[2], barcode.location[3], color=color, thickness=2)
                cv2.line(frame, barcode.location[0], barcode.location[3], color=color, thickness=2)
                cv2.circle(frame, (center_x, center_y), 3, color=color, thickness=2)
                cv2.putText(frame, org=(width - 100, height - 100), text=text, color=color,
                            fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1, )
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

def scale(value, scaler):
    if abs(value) < 0.03:
        return 0
    return value * scaler

print("Connecting to drone..")
drone = Bebop()
drone.video_callbacks(video_start, video_end, video_frame)
drone.videoEnable()
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

MAX_SPEED = 70

while not done:
    try:
        # EVENT PROCESSING STEP
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop

        executing_command = False
        userMovement = False
        drone.update(cmd=moveCameraCmd(0,0))

        if joystick.get_button(0) == 1:
            executing_command = True
            # print("Button 0 pressed")
            drone.update(cmd=moveCameraCmd(-100,0))

        if joystick.get_button(1) == 1:
            executing_command = True
            drone.update(cmd=moveCameraCmd(0,100))

        if joystick.get_button(2) == 1:
            executing_command = True
            drone.update(cmd=moveCameraCmd(0,-100))

        if joystick.get_button(3) == 1:
            executing_command = True
            # print("Button 3 pressed")
            drone.update(cmd=moveCameraCmd(100,0))

        if joystick.get_button(4) == 1:
            executing_command = True
            # print("Button 4 pressed")

        if joystick.get_button(5) == 1:
            executing_command = True
            # print("Button 5 pressed")

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
            # print("Button 8 pressed")
            drone.emergency()

        if joystick.get_button(9) == 1:
            executing_command = True
            # print("Button 9 pressed")

        if joystick.get_button(10) == 1:
            executing_command = True
            # print("Button 10 pressed")

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
        if userMovement == True:
            drone.update(cmd=movePCMDCmd(True, roll, pitch, yaw, gaz))
        else:
            drone.hover()

        (hat_x, hat_y) = joystick.get_hat(0)

        # Limit to 20 frames per second
        clock.tick(20)

    #     if not executing_command:
    #         drone.update(cmd=movePCMDCmd(False, 0, 0, 0, 0))
    #         drone.update(cmd=None)
    except:
        print("Error")
        if drone.flyingState is None or drone.flyingState == 1: # if taking off then do emegency landing
            drone.emergency()
            drone.land()


# Close the window and quit.
pygame.quit()

import cv2
import logging
from bebop import Bebop
from matcher import Matcher

command = None
cnt = 0

logging.basicConfig(level=logging.DEBUG)

wnd = None
def video_frame(frame):
    global cnt, command
    cnt += 1

    cv2.imshow("Drone", frame)
    if (cnt % 10 == 0 and command is None):
        template_that_matches = matcher.match(frame)
        if template_that_matches == "techgarage-logo":
            command = "TAKEOFF"
        elif template_that_matches == "first-logo":
            command = "LAND"

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

matcher = Matcher([("fau-logo", "../opencv/templates/fau-logo.png"),
                   ("first-logo", "../opencv/templates/first-logo.jpg"),
                   ("nextera-logo", "../opencv/templates/nextera-energy-logo.jpg"),
                   ("techgarage-logo", "../opencv/templates/techgarage-logo.png")
                   ], min_keypoints_pct_match=10)


print("Connecting to drone..")
drone = Bebop()
drone.video_callbacks(video_start, video_end, video_frame)
drone.videoEnable()
print("Connected.")

for i in xrange(10000):
    if command is None:
        drone.update( );
    elif command == "TAKEOFF":
        print("Taking offf.........................")
        drone.takeoff()
        command = None
    elif command == "LAND":
        print("Landing ...........................")
        drone.land()
        command = None


print("Battery:", drone.battery)

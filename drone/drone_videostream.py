import cv2
import time
import logging

from bebop import Bebop

logging.basicConfig(level=logging.DEBUG)

wnd = None
def video_frame(frame):
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

# Setting video callback functions
drone.video_callbacks(video_start, video_end, video_frame)

# enable and play video for a bit
drone.videoEnable()
print("Connected.")
for i in xrange(300):
    drone.update();
# turn off video
drone.videoDisable()

# short delay
for i in xrange(200):
    drone.update();

#start video stream again
drone.videoEnable()
for i in xrange(300):
    drone.update();
drone.videoDisable()


print("Battery:", drone.battery)

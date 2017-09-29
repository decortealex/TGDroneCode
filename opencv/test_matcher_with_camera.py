import time
import cv2
from matcher import Matcher

matcher = Matcher([("fau-logo", "./templates/fau-logo.png"),
                   ("first-logo", "./templates/first-logo.jpg"),
                   ("nextera-logo", "./templates/nextera-energy-logo.jpg"),
                   ("techgarage-logo", "./templates/techgarage-logo.png")
                   ], min_keypoints_pct_match=15)

cam = cv2.VideoCapture(0)

cnt = 0
while True:
    (grabbed, img) = cam.read()
    print matcher.match(img)
    cv2.imshow("Pic", img)
    key = cv2.waitKey(10)
    if key == ord('q'):
        break

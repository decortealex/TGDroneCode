import time
import cv2
import imutils
from imutils.video import VideoStream


from matcher import Matcher

matcher = Matcher([("fau-logo", "./templates/fau-logo.png"),
                   ("first-logo", "./templates/first-logo.jpg"),
                   ("nextera-logo", "./templates/nextera-energy-logo.jpg"),
                   ("techgarage-logo", "./templates/techgarage-logo.png")
                   ], min_keypoints_pct_match=8)

cam = VideoStream(usePiCamera=False).start()

cnt = 0
while True:
    img = cam.read()
    cv2.imshow("Pic", img)
    print matcher.match(img)
    key = cv2.waitKey(10)
    if key == ord('q'):
       break

cam.stop()
cv2.destroyAllWindows()
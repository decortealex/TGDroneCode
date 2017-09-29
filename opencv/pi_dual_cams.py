import cv2
import datetime


left_camera = cv2.VideoCapture(1)
#left_camera.set(cv2.CAP_PROP_FRAME_WIDTH,320)
#left_camera.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

right_camera = cv2.VideoCapture(2)
#right_camera.set(cv2.CAP_PROP_FRAME_WIDTH,320)
#right_camera.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

# VideoStream(usePiCamera=False, src=0, framerate=10, resolution=(320, 240)).start()
#right_camera = VideoStream(usePiCamera=False, src=1, framerate=5).start()

cnt = 0
start = datetime.datetime.now()
fps = 0
ms = 0
while True:
    diff = datetime.datetime.now() - start

    if ms < diff.microseconds:
        cnt = cnt + 1
    else:
        if cnt != 0:
            fps = cnt
        cnt = 0
        start = datetime.datetime.now()
    ms = diff.microseconds

    g1, left_image = left_camera.read()
  #  g2, right_image = right_camera.read()
    if fps != 0:
        cv2.putText(left_image, "FPS: %d" % (fps), (5, 15), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))

    cv2.imshow("Left", left_image)
  #  cv2.imshow("Right", right_image)

    cv2.waitKey(10)
  #  if key == ord('q'):
  #     break
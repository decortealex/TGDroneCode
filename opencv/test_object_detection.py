import numpy as np
import cv2


#img1 = cv2.imread('/home/user1/opencv/samples/data/box.png',0)          # queryImage
img1 = cv2.imread('/home/user/Documents/TGDroneCode/Idol.jpg',0)          # queryImage

# Initiate SIFT detector
#sift = cv2.xfeatures2d.SURF_create(1000)
sift = cv2.xfeatures2d.SIFT_create()

# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(img1,None)
bf = cv2.BFMatcher()

cam = cv2.VideoCapture(0)
#cam.set(cv2.CAP_PROP_FRAME_WIDTH,320)
#cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)


while True:
    (grabbed, frame) = cam.read()
    img2  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kp2, des2 = sift.detectAndCompute(img2,None)

    # BFMatcher with default params
    if len(kp2) > 0:
        matches = bf.knnMatch(des1,des2, k=2)

        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                good.append([m])

        print len(kp1), len(kp2), len(good)
        # cv2.drawMatchesKnn expects list of lists as matches.
        img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,img2, flags=2)

        cv2.imshow("object detect", img3)
        cv2.waitKey(10)

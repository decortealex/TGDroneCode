# USAGE
# python cam.py --face cascades/haarcascade_frontalface_default.xml
# python cam.py --face cascades/haarcascade_frontalface_default.xml --video video/adrian_face.mov

# import the necessary packages
from pyimagesearch.facedetector import FaceDetector
from pyimagesearch import imutils
import argparse
import cv2

# construct the argument parse and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-f", "--face", required = True,
# 	help = "path to where the face cascade resides")
# ap.add_argument("-v", "--video",
# 	help = "path to the (optional) video file")
# args = vars(ap.parse_args())

# construct the face detector
fd = FaceDetector('cascades/haarcascade_frontalface_default.xml')

# if a video path was not supplied, grab the reference
# to the gray
# if not args.get("video", False):
camera = cv2.VideoCapture(0)

# otherwise, load the video
# else:
# 	camera = cv2.VideoCapture(args["video"])
# Load our overlay image: mustache.png
imgMustache = cv2.imread('images/mustache.png', -1)

# Create the mask for the mustache
orig_mask = imgMustache[:, :, 3]

# Create the inverted mask for the mustache
orig_mask_inv = cv2.bitwise_not(orig_mask)

# Convert mustache image to BGR
# and save the original image size (used later when re-sizing the image)
imgMustache = imgMustache[:, :, 0:3]
origMustacheHeight, origMustacheWidth = imgMustache.shape[:2]
# keep looping
while True:
	# grab the current frame
	(grabbed, frame) = camera.read()

	# if we are viewing a video and we did not grab a
	# frame, then we have reached the end of the video
	# if args.get("video") and not grabbed:
	# 	break

	# resize the frame and convert it to grayscale
	frame = imutils.resize(frame, width = 300)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# detect faces in the image and then clone the frame
	# so that we can draw on it
	faceRects = fd.detect(gray, scaleFactor = 1.1, minNeighbors = 5,
		minSize = (30, 30))
	frameClone = frame.copy()

	# loop over the face bounding boxes and draw them
	# for (fX, fY, fW, fH) in faceRects:
	# 	cv2.rectangle(frameClone, (fX, fY), (fX + fW, fY + fH), (0, 255, 0), 2)

	# Iterate over each face found
	for (x, y, w, h) in faceRects:
		# Un-comment the next line for debug (draw box around all faces)
		#face = cv2.rectangle(frameClone,(x,y),(x+w,y+h),(255,0,0),2)

		roi_gray = gray[y:y + h, x:x + w]
		roi_color = frameClone[y:y + h, x:x + w]

		# Detect a nose within the region bounded by each face (the ROI)
		noseCascade = cv2.CascadeClassifier('cascades/haarcascade_nose.xml')
		nose = noseCascade.detectMultiScale(roi_gray)

		for (nx, ny, nw, nh) in nose:
			# Un-comment the next line for debug (draw box around the nose)
			#cv2.rectangle(roi_color,(nx,ny),(nx+nw,ny+nh),(255,0,0),2)

			# The mustache should be three times the width of the nose
			mustacheWidth = 3 * nw
			mustacheHeight = mustacheWidth * origMustacheHeight / origMustacheWidth

			# Center the mustache on the bottom of the nose
			x1 = nx - (mustacheWidth / 4)
			x2 = nx + nw + (mustacheWidth / 4)
			y1 = ny + nh - (mustacheHeight / 2)
			y2 = ny + nh + (mustacheHeight / 2)

			# Check for clipping
			if x1 < 0:
				x1 = 0
			if y1 < 0:
				y1 = 0
			if x2 > w:
				x2 = w
			if y2 > h:
				y2 = h

			# Re-calculate the width and height of the mustache image
			mustacheWidth = x2 - x1
			mustacheHeight = y2 - y1

			# Re-size the original image and the masks to the mustache sizes
			# calcualted above
			mustache = cv2.resize(imgMustache, (mustacheWidth, mustacheHeight), interpolation=cv2.INTER_AREA)
			mask = cv2.resize(orig_mask, (mustacheWidth, mustacheHeight), interpolation=cv2.INTER_AREA)
			mask_inv = cv2.resize(orig_mask_inv, (mustacheWidth, mustacheHeight), interpolation=cv2.INTER_AREA)

			print mask.size, mustacheWidth, mustacheHeight
			# take ROI for mustache from background equal to size of mustache image
			roi = roi_color[y1:y2, x1:x2]

			# roi_bg contains the original image only where the mustache is not
			# in the region that is the size of the mustache.
			roi_bg = cv2.bitwise_and(roi, roi, mask=mask_inv)

			# roi_fg contains the image of the mustache only where the mustache is
			roi_fg = cv2.bitwise_and(mustache, mustache, mask=mask)

			# join the roi_bg and roi_fg
			dst = cv2.add(roi_bg, roi_fg)

			# place the joined image, saved to dst back over the original image
			roi_color[y1:y2, x1:x2] = dst

			break

	# show our detected faces
	cv2.imshow("Face", frameClone)

	# if the 'q' key is pressed, stop the loop
	if cv2.waitKey(1) & 0xFF == ord("q"):
		break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
import cv2

target_image = cv2.imread("./templates/sniper-target.png")
target_image_resized = None

camera = cv2.VideoCapture(1)
while True:
    (grabbed, image) = camera.read()
    if grabbed:
        if target_image_resized is None:
            target_image_resized = cv2.resize(target_image, None, fx=0.15, fy=0.15, interpolation=cv2.INTER_AREA)
            #cv2.imshow("Target", target_image_resized)
            #cv2.waitKey(0)
        face_classifier = cv2.CascadeClassifier("./cascades/haarcascade_frontalface_default.xml")
        faces = face_classifier.detectMultiScale(image)
        image_clone = image.copy()
        for (fX, fY, fW, fH) in faces:
            target = cv2.resize(target_image, dsize=(fW, fH),  interpolation=cv2.INTER_AREA)
            cv2.rectangle(image_clone, (fX, fY), (fX + fW, fY + fH), (0, 255, 0), 2)

        cv2.imshow("Image", image_clone)
        key = cv2.waitKey(10)
        if key == ord('q'):
            break
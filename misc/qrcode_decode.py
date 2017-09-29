import zbar
import PIL.Image as Image
import cv2


scanner = zbar.ImageScanner()
scanner.parse_config('enable')


cam = cv2.VideoCapture(0)

while True:
    (grabbed_frame, frame) = cam.read()

    if grabbed_frame:
        # obtain image data
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pil = Image.fromarray(gray)
        width, height = pil.size
        raw = pil.tobytes()

        # wrap image data
        image = zbar.Image(width, height, 'Y800', raw)

        # scan the image for barcodes
        scanner.scan(image)

        # extract results
        for symbol in image:
            # do something useful with results
            print 'decoded', symbol.type, 'symbol', '"%s"' % symbol.data


        cv2.imshow("Feed", frame)
        cv2.waitKey(10)
import numpy as np
import zbar
import PIL.Image as Image
import cv2

class Decoder:
    def __init__(self):
        self.scanner = zbar.ImageScanner()
        self.scanner.parse_config('enable')


    def decode(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        pil = Image.fromarray(gray)
        width, height = pil.size
        raw = pil.tobytes()

        # wrap image data
        image = zbar.Image(width, height, 'Y800', raw)

        # scan the image for barcodes
        self.scanner.scan(image)

        # extract results
        barcodes = []
        for symbol in image:
            barcode = Barcode(symbol)
            barcodes.append(barcode)
            # do something useful with results
            print('decoded', symbol.type, 'symbol', symbol.location, '"%s"' %
                  symbol.data)

        return barcodes

class Barcode:
    def __init__(self, symbol):
        self.type = symbol.type
        self.location = symbol.location
        self.value = symbol.data



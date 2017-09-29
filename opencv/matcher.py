import cv2



class Matcher:
    def __init__(self, templates, min_keypoints_pct_match = 20):

        # min. percentage of keypoints needs to match
        self.min_keypoints_pct_match = min_keypoints_pct_match

        # create SIFT object features extractor
        self.sift = cv2.xfeatures2d.SURF_create(300, 10, 2)

        # create matcher
        self.bf = cv2.BFMatcher()

        self.descriptors = {}
        for (label, filename) in templates:
            img = cv2.imread(filename, 0)
            kp, des = self.sift.detectAndCompute(img, None)
            print "Analyzed - Label = %s, Filename = %s, # of Keypoints = %s" % (label, filename, len(kp))
            self.descriptors[label] = (kp, des)


    def match(self, img):
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_keypoints, img_descriptor = self.sift.detectAndCompute(img, None)

        max_matches = 0
        detected_label = None
        detected_keypoints_ratio = 0

        for label, (template_keypoints, template_descriptor) in self.descriptors.items():
            matches = self.bf.knnMatch(img_descriptor, template_descriptor, k=2)

            # Apply ratio test
            good = []
            for m, n in matches:
                if m.distance < 0.75 * n.distance:
                    good.append([m])
            #print "Label = %s, mathes = %s, keypoints = %s ", (label, len(good), len(template_keypoints))
            match_ratio = len(good) * 100 / len(template_keypoints)
            print label, match_ratio
            if match_ratio > self.min_keypoints_pct_match and match_ratio > detected_keypoints_ratio:
                 detected_keypoints_ratio = match_ratio
                 detected_label = label

        return detected_label

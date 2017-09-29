import cv2
from matcher import Matcher


matcher = Matcher([("fau-logo",     "./templates/fau-logo.png"),
                   ("first-logo",   "./templates/first-logo.jpg"),
                   ("nextera-logo", "./templates/nextera-energy-logo.jpg"),
                   ("techgarage-logo", "./templates/techgarage-logo.png")
                  ])

# positive_match = cv2.imread("./images/mm3.jpg", 0)
# negative_match = cv2.imread("./images/mm6.jpg", 0)
#
# print "Analyzing..."
# print matcher.match(positive_match)
# print matcher.match(negative_match)

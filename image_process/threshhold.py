# import cv2 as cv
# import sys
#
# # read and store image in cv::Mat object
# img = cv.imread(cv.samples.findFile("0-2022-09-28_10_10_21.jpg"))
# if img is None:
#     sys.exit("could not read image")
#
# # display image with cv
# # cv.imshow("image", img)
# # cv.waitKey(0)

import cv2
import numpy as np

# read and scale down image
# wget https://bigsnarf.files.wordpress.com/2017/05/hammer.png #black and white
# wget https://i1.wp.com/images.hgmsites.net/hug/2011-volvo-s60_100323431_h.jpg
img = cv2.pyrDown(cv2.imread('0-2022-09-28_14_02_26.jpg', cv2.IMREAD_UNCHANGED))

# threshold image
# convert RGB to Grey
#127 255
gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# ret, threshed_img = cv2.threshold(gray_img, 230, 240, cv2.THRESH_BINARY)
threshed_img = cv2.threshold(gray_img, 230, 240, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

contours = cv2.findContours(threshed_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = contours[0] if len(contours) == 2 else contours[1]
# find contours and get the external one
# contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#image, contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE,
#                cv2.CHAIN_APPROX_SIMPLE)

# with each contour, draw boundingRect in green
# a minAreaRect in red and
# a minEnclosingCircle in blue
for c in contours:
    # get the bounding rect
    x, y, w, h = cv2.boundingRect(c)
    # draw green rect with line size 2
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # get the min area rect
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)
    # convert all coordinates floating point values to int
    box = np.int0(box)
    # draw a red 'nghien' rectangle
    #cv2.drawContours(img, [box], 0, (0, 0, 255))

    # finally, get the min enclosing circle
    (x, y), radius = cv2.minEnclosingCircle(c)
    # convert all values to int
    center = (int(x), int(y))
    radius = int(radius)
    # and draw the circle in blue
    #img = cv2.circle(img, center, radius, (255, 0, 0), 2)

print(len(contours))
#cv2.drawContours(img, contours, -1, (255, 255, 0), 1)

cv2.imshow("gray img", gray_img )
cv2.imshow("threshed img", threshed_img)
cv2.imshow("boxes", img)

#cv2.imshow("coutours", hier)

#cv2.imshow("contours", img)

while True:
    key = cv2.waitKey(1)
    if key == 27: #ESC key to break
        break

cv2.destroyAllWindows()

# Import the OpenCV library
import cv2

# Read the image from the location
img = cv2.imread('0-2022-09-28_14_02_26.jpg')

# Convert the image into graycsale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Blur the gray scale image for better edge detection
blur = cv2.GaussianBlur(gray, (5,5), 0)

# Sobel Edge Detection on x-axis
sobelx = cv2.Sobel(src=blur, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5)
# Sobel Edge Detection on y-axis
sobely = cv2.Sobel(src=blur, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5)
# Sobel Edge Detection on both axis
sobelxy = cv2.Sobel(src=blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5)

# Save the Sobel Edge Detection Images
cv2.imwrite('./images/sobelx.jpg', sobelx)
cv2.imwrite('./images/sobely.jpg', sobely)
cv2.imwrite('./images/sobelxy.jpg', sobelxy)

# Display Sobel Edge Detection Images
cv2.imshow('Sobel X Edge Detection', sobelx)
cv2.imshow('Sobel Y  Edge Detection', sobely)
cv2.imshow('Sobel X and Y Edge Detection', sobelxy)
cv2.waitKey(0)
cv2.destroyAllWindows()


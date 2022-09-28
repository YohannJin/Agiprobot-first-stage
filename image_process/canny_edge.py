# Import the OpenCV library
import cv2

# Read the original image from the location
img = cv2.imread('0-2022-09-28_10_10_21.jpg')

# Convert the image into graycsale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Blur the gray scale image for better edge detection
blur = cv2.GaussianBlur(gray, (5,5), 0)

# Canny Edge Detection
canny_edges = cv2.Canny(image=blur, threshold1=100, threshold2=200)

# Display Canny Edge Detection Image
cv2.imshow('Canny Edge Detection', canny_edges)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Save the Canny Edge Detection Image
cv2.imwrite('images/canny.jpg', canny_edges)
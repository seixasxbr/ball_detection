import numpy as np
import cv2 as cv

img = cv.imread("image.jpg")
hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

lower_range = np.array([5, 100, 20])
upper_range = np.array([25, 255, 255])

mask = cv.inRange(hsv, lower_range, upper_range)
# cv.imshow("orange", mask)
cv.imwrite("laranja2masked.jpg", mask)

cv.waitKey()
cv.destroyAllWindows()
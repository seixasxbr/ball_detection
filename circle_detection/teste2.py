import numpy as np
import cv2 as cv

img = cv.imread("laranja.jpg")
hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
#mask = cv.inRange(hsv,(10, 100, 20), (25, 255, 255) )
mask = cv.inRange(hsv,(5, 100, 20), (25, 255, 255) )
cv.imshow("orange", mask)
masked = cv.bitwise_and(img, img, mask=mask)
#masked.save("resu.jpg")

#img2 = cv.imread("resu.png")
#img2 = cv.imread(masked)
#output = masked.copy()
gray = cv.cvtColor(masked, cv.COLOR_BGR2GRAY)
gray = cv.medianBlur(gray, 5)
circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, 20,
                          param1=100, param2=70, minRadius=80, maxRadius=100)
detected_circles = np.uint16(np.around(circles))
for (x, y ,r) in detected_circles[0, :]:
    cv.circle(output, (x, y), r, (0, 0, 0), 3)
    cv.circle(output, (x, y), 2, (0, 255, 255), 3)


cv.imshow('output',output)
cv.waitKey()
cv.destroyAllWindows()
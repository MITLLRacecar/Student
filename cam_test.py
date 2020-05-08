import cv2 as cv

cam = cv.VideoCapture(2)
_,im = cam.read()
cv.imwrite('out.jpg', im)
cam.release()

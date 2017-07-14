from image_processor import ImageProcessor
import cv2

processor = ImageProcessor()

img = cv2.imread('images/GearTarget13.jpg')
img = cv2.resize(img, (320, 240))
out = processor.process_frame(img, 0)

cv2.imshow('Frame', out)

cv2.waitKey(0)
cv2.destroyAllWindows()

exit(0)

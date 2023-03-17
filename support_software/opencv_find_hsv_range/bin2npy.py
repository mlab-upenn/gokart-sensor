
import cv2
import numpy as np

img = cv2.imread('/home/felix/Desktop/Evaluation set Cone Detection.v3i.darknet/train/frame0001_jpg.rf.7825a547122dfc6005b2d6cb92a5a4c1.jpg')

ORANGE_MIN = np.array([0, 50, 50],np.uint8)
ORANGE_MAX = np.array([25, 255, 255],np.uint8)

hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
window_name = 'image'

frame_threshed = cv2.inRange(hsv_img, ORANGE_MIN, ORANGE_MAX)
cv2.imshow(window_name, frame_threshed)
# cv2.imwrite('output2.jpg', frame_threshed)

cv2.waitKey(0)
cv2.destroyAllWindows()
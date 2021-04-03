import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('../data/images/squares.png',0)
print("#####")
print(img.shape)

#img = cv2.resize(img, (1080, 1920)); 
#print("#####")
#print(img.shape)

#cv2.resizeWindow("full window", 1024, 576);
cv2.namedWindow("full window", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty('full window', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

cv2.imshow("full window", img)
cv2.waitKey(0);

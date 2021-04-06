import cv2
import numpy as np
from matplotlib import pyplot as plt

'''
* Project the squares patter onto full screen and having no title bar.
* For the projector image itself, use a screenshot image when displaying this full screen image.
* Dims of screenshot will be the same as the full screen image being sent to projector 
    * (Assuming resolution being sent to the projector is of the default full screen of laptop).

* Ensure python 3.7+ is being used... ie. conda activate if needed
* Python 2.7 will not yield a full screen image but some really large image whose actual resolution
will be matched with the screen resolution in a 1:1 ratio.
'''

img = cv2.imread('../data/images/squares.png',0)
print("---")
print(img.shape)

#img = cv2.resize(img, (1080, 1920)); # But this squashes the squares
#print("---")
#print(img.shape)
#cv2.resizeWindow("full window", 1024, 576); # Window with no title bar and of some fixed window size

cv2.namedWindow("full window", cv2.WND_PROP_FULLSCREEN) # Create a window and of type/size full screen
cv2.setWindowProperty('full window', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN) # Officially set properties of window to be fullscreen

cv2.imshow("full window", img)
cv2.waitKey(0);

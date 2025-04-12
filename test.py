import cv2
import numpy as np

while True:
    frame = np.random.rand(500, 500)

    cv2.imshow('frame', frame)
    cv2.waitKey(1)
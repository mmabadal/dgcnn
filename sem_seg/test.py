import os
import cv2
import numpy as np
from tqdm import tqdm
from natsort import natsorted


disp_path = "/home/olivia/Desktop/test_polygon/1615300348362891008.jpeg"
disp = cv2.imread(disp_path, 2)

print(np.unique(disp))

print(disp.shape)

print(disp[0,0])




 
# disp = cv2.resize(disp, (1024, 1024), interpolation = cv2.INTER_AREA)
   
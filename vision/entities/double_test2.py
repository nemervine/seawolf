# double_test.py

from __future__ import division
import math

import cv
import cv2
import svr
import numpy as np

from base import VisionEntity
import libvision

class DoubleTestEntity(VisionEntity):
    def init(self):
        self.number = 0
        
    
    def process_frame(self, frame):
        if self.debug:
            ff = np.asarray(frame[:,:])
            
            out_image = cv.fromarray(out_image)
            out_image = cv.GetImage(out_image)
            
            svr.debug("Lines", out_image)
            

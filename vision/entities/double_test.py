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
        
            #frame1 = cv.CreateImage(cv.GetSize(frame), 8, 3)
            #frame3 = cv.CreateImage(cv.GetSize(frame), 8, 1)
            #frame2 = cv.CreateImage(cv.GetSize(frame), 8, 1)
            #frame4 = cv.CreateImage(cv.GetSize(frame), 8, 3)
            #cv.SubS(frame, (255, 255, 0), frame1)
            #cv.Sub(frame, frame1, frame2)
            #cv.Smooth(frame, frame1, cv.CV_GAUSSIAN, 51, 51)
            
            
            ff = np.asarray(frame[:,:])
            #frame1 = cv2.GaussianBlur(ff, (31,31), 3.0)
            gray = cv2.cvtColor(ff, cv.CV_BGR2GRAY)
            frame1 = cv2.Sobel(gray, -1, 3, 3, ksize=11)
            frame1 = cv2.GaussianBlur(frame1, (3, 3), 3.0)
            
            lines = cv2.HoughLinesP(frame1, 1, math.pi/180, 30, minLineLength=30)
            
            color = cv2.cvtColor(frame1, cv.CV_GRAY2BGR)
            
            height, width, depth = color.shape
            out_image = np.zeros((height,width,3), np.uint8)
            out_image2 = np.zeros((height, width, 3), np.uint8)
            print type(lines)
            if lines is not None:
                for l in lines:
                    print l
                    a = l[0][0]
                    b = l[0][1]
                    c = l[0][2]
                    d = l[0][3]
                    cv2.line(out_image2, (a, b), (c, d), (255, 0, 0))
                    out_image = cv2.add(out_image, out_image2)
            
            
            #cv.CvtColor(frame, frame3, cv.CV_RGB2GRAY)
            #frame3 = cv2.cvtColor(frame1, cv.CV_BGR2GRAY)
            
            #cv.Canny(frame3, frame2, 70, 70)
            #frame2 = cv2.Canny(frame3, 70, 100)
            
            #cv.CvtColor(frame2, frame4, cv.CV_GRAY2RGB)
            #frame4 = cv2.cvtColor(frame2, cv.CV_GRAY2BGR)
            
            #cv.Add(frame1, frame4, frame4)
            #frame1 = cv2.add(frame1, frame4)
            
            
            frame4 = cv.fromarray(color)
            img_frame4 = cv.GetImage(frame4)
            
            out_image = cv.fromarray(out_image)
            out_image = cv.GetImage(out_image)
            
            svr.debug("Original", frame)
            svr.debug("Test", img_frame4)
            svr.debug("Lines", out_image)
            

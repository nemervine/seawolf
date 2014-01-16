# paths.py

from __future__ import division
from __future__ import print_function
import math
import numpy as np
import cv
import cv2
#from cv_conversion_tools import * --> HAS BEEN INSERTED INTO THIS FILE.
from line_reducer import LineReducer
import svr
from base import VisionEntity


class PathsEntity(VisionEntity):
    '''A class that searches the screen for the presence of one or more orange
    boards which are designed to give directions to the robot.
    
    This is the method the class uses:
    1.  Preprocess the image -- Since we know the color of the board, and the
        color it is when it is in the shadow of the robot, we can threashold
        this color and be fairly confident we will be left with just the boards
        and some small amount of noise.
    2.  Now we will take the gradient of the image.  We will be left with a map
        of edges which we can further analyze.
    3.  Next, we use the Hough Probabilistic trasform to identify candidates
        for the boards.  We can use the width of the boards in sample videos to
        determine what minimum length to use.
    4.  Now that we have a set of lines describing the board/boards, all we
        must do is group them together and return the common orientations.
    *As an extra, we may also return the centers of the boards.
    '''
    
    
    threshold_block_size = 3                #For hue_threshold
    threshold_path_hue_upper = 0x10 & 0xFF
    threshold_path_hue_lower = -0x1A & 0xFF
    sqrt_3 = math.sqrt(3)
    
    
    def cv_to_cv2(self, frame):
        '''Convert a cv image into cv2 format.
        
        Keyword Arguments:
        frame -- a cv image
        Returns a numpy array that can be used by cv2.
        '''
        cv2_image = np.asarray(frame[:,:])
        return cv2_image
        
        
    def cv2_to_cv(self, frame):
        '''Convert a cv2 image into cv format.
        
        Keyword Arguments:
        frame -- a cv2 numpy array representing an image.
        Returns a cv image.
        '''
        container = cv.fromarray(frame)
        cv_image = cv.GetImage(container)
        return cv_image
    
        
    def hue_threshold(self, frame):
        '''Take frame and threshold it between a range of hues.
        
        Keyword Arguments:
        frame -- cv2 numpy image
        lower_hue -- a lower value for the hue (0..255)
        upper_hue -- an upper value for the hue (0..255)
        Returns single-channel cv2 numpy image with values for each pixel:
            - 255 if hue of corresponding pixel in input image is in range
            - 0 otherwise
        '''
        size = cv.GetSize(frame)
	    
        hsvframe = cv.CreateImage(size, 8, 3)
        cv.CvtColor(frame, hsvframe, cv.CV_BGR2HSV)
        
        hueframe = cv.CreateImage(size, 8, 1)
        cv.SetImageCOI(hsvframe, 1) # (0, 1, 2, 3) = (all, h, s, v)
        cv.Copy(hsvframe, hueframe)
        
        below_upper = cv.CreateImage(size, 8, 1)
        cv.AdaptiveThreshold(
            src             = hueframe,
            dst             = below_upper,
            maxValue        = 255,
            adaptive_method = cv2.ADAPTIVE_THRESH_MEAN_C,
            thresholdType   = cv2.THRESH_BINARY_INV,
            blockSize       = self.threshold_block_size,
            param1          = self.threshold_path_hue_upper) # param1 name is derp on cv's part.
            
        above_lower = cv.CreateImage(size, 8, 1)
        cv.AdaptiveThreshold(
            src             = hueframe,
            dst             = above_lower,
            maxValue        = 255,
            adaptive_method = cv2.ADAPTIVE_THRESH_MEAN_C,
            thresholdType   = cv2.THRESH_BINARY,
            blockSize       = self.threshold_block_size,
            param1          = self.threshold_path_hue_lower) # param1 name is derp on cv's part.
        
        final_threshold = hueframe #re-use this. Not needed anymore, and it's pass-by-reference.
        if self.threshold_path_hue_lower > self.threshold_path_hue_upper:
            cv.And(
                src1    = below_upper,
                src2    = above_lower,
                dst     = final_threshold)
        else:
            cv.Or(
                src1    = below_upper,
                src2    = above_lower,
                dst     = final_threshold)
        return final_threshold
        #kono ko~do wa totemo utsukushii naa
    
    
    def preprocess(self, frame):
        '''Preprocess an image by threasholding it to desired ranges and then
        converting it to 1-channel binary format.  The image is also converted
        to cv2 compatible format.
        
        Note RGB2HSV is intentional to prevent hue wrap on our desired range.
        The input image is in BGR format, not RGB format.
        
        Keyword Arguments:
        frame -- cv image to be preprocessed.
        Returns cv2 numpy array representing grayscaled thresholded image.
        '''
        prepared = self.hue_threshold(frame)
        prepared_cv2 = self.cv_to_cv2(prepared)
        print ('paratum.')
        
        return prepared_cv2
        
        
    def get_gradient(self, frame):
        '''Find the gradient of the image.
        
        Keyword Arguments:
        frame -- binary image
        Returnes image representing gradient.
        '''
        mild_blur = cv2.GaussianBlur(frame, (3, 3), 3.0)
        gradient = cv2.Laplacian(mild_blur, ddepth=8)
        return gradient
        
        
    def hough_p(self, frame):
        '''Take a binary image which consists of edges and use the hough
        probabilistic transform to find lines that describe those edges.  Since
        this class is focused on finding orange guidance boards, we will
        specifically be looking for the lines that run along the long side of
        the board.  This means we will use a minimum size requirement for the
        lines to avoid accepting lines on the width side of the board.  It also
        reduces chances of labeling noise.
        
        Keyword Arguments:
        frame -- binary image of edges
        Returns list of lines in rectangular form of lines that were found.
        '''        
        hough = cv2.HoughLinesP(
            frame,
            rho         = 30,
            theta       = math.pi / 90,
            threshold   = 40)
        #TODO: Evaluate safety of line reducer for the long, parallel lines.
        print ("hough be this:", type(hough))
        hough_tuple = tuple(map(tuple, hough))
        line_reducer = LineReducer(hough_tuple, error_ratio=1/6)
        simplified_hough = line_reducer.calculate_lines()
        return simplified_hough
        
        
    def max_angle_range(self, group, angle):
        '''Determine what the total angle range will be if angle is added to a
        group of angles.
        
        First find the minimum angle.  Next, ensure that all relative angles
        are less than 180 degrees (as compared with minimum).  Finally, return
        the maximum angle difference found between the min angle and all others.
        
        Keyword Arguments:
        group -- list of angles in radians.
        angle -- an angle to consider adding to group in radians.
        Returns maximum angle in radians.
        '''
        group_local = list(group)
        group_local.append(angle)
        
        theta_min = min(group_local)
        for a in group:
            while (a - theta_min) >= math.pi:
                a -= math.pi
                
        max_range = 0
        for a in group:
            diff = min(a, math.pi - a)
            max_range = max(max_range, diff)
        return max_range
        
        
        
    def group_lines(self, lines, degrees=True):
        '''Takes a list of lines found from image analysis, and determines how
        many true path-markers were present.  Further, it returns a list that
        has the angle of those markers in degrees (radians).
        
        This grouping algorithm has a flaw with the grouping... but if the lines
        in are good nothing bad should happen.
        
        Keyword Arguments:
        lines -- list of lines describing edges that were found
        degrees -- Should results be in degrees or radians? (default = degrees)
        Returns list of angles describing path markers found.
        '''
        acceptable_range = math.pi / 90
        line_count = len(lines)
        
        angles = []
        for rho, theta in lines:
            angles.append(theta)
            
        line_untested = [True for x in xrange(line_count)]
        group = []
        
        for i in xrange(line_count):
            if line_untested[i]:
                group_set = []
                group_set.append(angles[i])
                for j in xrange(i + 1, line_count):
                    if line_untested[i + j]:
                        angle_range = self.max_angle_range(group_set, angles[j])
                        if angle_range <= acceptable_range:
                            group_set.append(angles[j])
                            line_untested[j] = False
                group.append(np.mean(group_set))
                
        return group
                    
        
    def make_debug(self, frame, lines):
        '''Create some feedback for people to look at for debugging.  Input is
        really what every we happen to need or want to test.
        
        Keyword Arguments:
        frame -- original image
        '''
        pass
        
        
        
    def init(self):
        '''Sets up variable used in this class.'''
        pass
        
        
    def process_frame(self, frame):
        '''Process the frame and search for path markers.
        
        Keyword Arguments:
        frame -- image in cv format to process
        '''
        binary_img = self.preprocess(frame)
        grad = self.get_gradient(binary_img)
        hough_p = self.hough_p(grad)
        groups = self.group_lines(hough_p)
        self.output = groups
        self.debug(binary_img)
        return        
        
        
    def __repr__(self):
        '''Returns quick summary of this class'''
        return "<PathsEntity Generic>"
        

# cv_conversion_tools
'''As set of tools to allow interoperability of cv and cv2 modules.

Most of the problems involved in trying to use both cv and cv2 modules has to
do with the image format used by each.  These function will allow you to easily
convert between the two forms without wasting time and space in other files.
'''

import numpy
import cv


def cv_to_cv2(frame):
    '''Convert a cv image into cv2 format.
    
    Keyword Arguments:
    frame -- a cv image
    Returns a numpy array that can be used by cv2.
    '''
    cv2_image = numpy.asarray(frame[:,:])
    return cv2_image
    
    
def cv2_to_cv(frame):
    '''Convert a cv2 image into cv format.
    
    Keyword Arguments:
    frame -- a cv2 numpy array representing an image.
    Returns a cv image.
    '''
    container = cv.fromarray(frame)
    cv_image = cv.GetImage(container)
    return cv_image

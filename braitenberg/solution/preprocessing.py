import numpy as np
import cv2

#lower_hsv = np.array([0, 89, 59])
#upper_hsv = np.array([21, 200, 255])

lower_hsv = np.array([15, 200, 120])
upper_hsv = np.array([30, 255, 255])

def preprocess(image_rgb: np.ndarray) -> np.ndarray:
    """ Returns a 2D array """
    hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    # masked = cv2.bitwise_and(hsv, hsv, mask=mask)
    shape = image_rgb.shape[0], image_rgb.shape[1]
    ones=np.ones(shape=shape,dtype='float32')/255
    output=ones*mask
    #z=np.zeros(shape=shape,dtype='uint8')
    #output = z + cv2.bitwise_or(ones,ones,mask=mask)
    #if np.sum(output 
    return output
    #return mask

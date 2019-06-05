from scipy.ndimage import map_coordinates as interp2
import numpy as np
import cv2

# UndistortImage - undistort an image using a lookup table
# OUTPUTS:
#   undistorted: image after undistortion

def UndistortImage(image,LUT):
    reshaped_lut = LUT[:, 1::-1].T.reshape((2, image.shape[0], image.shape[1]))
    undistorted = np.rollaxis(np.array([interp2(image[:, :, channel], reshaped_lut, order=1)
                                for channel in range(0, image.shape[2])]), 0, 3)

    
    return undistorted.astype(image.dtype)
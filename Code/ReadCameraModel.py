import numpy as np

def ReadCameraModel(models_dir):

# ReadCameraModel - load camera intrisics and undistortion LUT from disk

# OUTPUTS:
#   fx: horizontal focal length in pixels
#   fy: vertical focal length in pixels
#   cx: horizontal principal point in pixels
#   cy: vertical principal point in pixels
#   G_camera_image: transform that maps from image coordinates to the base
#     frame of the camera. For monocular cameras, this is simply a rotation.
#     For stereo camera, this is a rotation and a translation to the left-most
#     lense.
#   LUT: undistortion lookup table. For an image of size w x h, LUT will be an
#     array of size [w x h, 2], with a (u,v) pair for each pixel. Maps pixels
#     in the undistorted image to pixels in the distorted image

    intrinsics_path = models_dir + "/stereo_narrow_left.txt"
    lut_path = models_dir + "/stereo_narrow_left_distortion_lut.bin"
    intrinsics = np.loadtxt(intrinsics_path)
    fx = intrinsics[0,0]
    fy = intrinsics[0,1]
    cx = intrinsics[0,2]
    cy = intrinsics[0,3]
    G_camera_image = intrinsics[1:5,0:4]
    lut = np.fromfile(lut_path, np.double)
    lut = lut.reshape([2, lut.size//2])
    LUT = lut.transpose()

    return fx, fy, cx, cy, G_camera_image, LUT
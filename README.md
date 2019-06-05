# Visual-Odometry

## **PROJECT DESCRIPTION**

The aim of this project is to implement the different steps to estimate the 3D motion of the camera and provide as output a plot of the trajectory of a car driving around the city. As the car moves around the city, we track the change in position of the camera with respective to the initial point.

Please refer to [Project Report](https://github.com/adheeshc/Visual-Odometry/blob/master/Report/FINAL%20REPORT.pdf) for further description

### Preparing the Input

<p align="center">
  <img src="/Images/pre.png" alt="pre">
</p>

The dataset used is the Oxford Dataset courtesy of Oxford’s Robotics Institute which if downloaded directly requires further pre-processing

- The input images are in Bayer format which have to be converted to RGB scale
- The image has to be undistorted

However, to speed up the processing, I have already done the same and saved them in a folder FRAMES which can be taken direclty from the folder Datasets. I have also converted to grayscale as it is easier to process in one channel. 

<p align="center">
  <img src="/Images/30.jpg" alt="post">
</p>

### Fundamental Matrix Estimation

<p align="center">
  <img src="/Images/sift.png" alt="SIFT">
</p>

- SIFT algorithm is used to detect keypoints
- Point correspondences are found between successive frames using the 8-point algorithm
- Normalizing all the points around the mean of the points and enclose them at a distance of √2 from the new center location
- The best Fundamental matrix is found using the RANSAC algorithm 

### Camera Pose Estimation

- Essential m atrix is calculated from the Fundamental matrix accounting for the Camera Calibration Parameters.
- The Essential matix is decomposed into 4 possible Translations and Rotations pairs

### Triangulation Check

<p align="center">
  <img src="/Images/my_code.png" alt="my_code">
</p>

The correct T and R pair is found from depth positivity. I choose the R and T which gives the largest amount of positive depth values.

The values are saved in a csv file updated2.csv

### Built-in Check

<p align="center">
  <img src="/Images/compare.png" alt="Built_in">
</p>

Finally, the results are compared to against the rotation/translation parameters recovered using the cv2.findEssentialMat and cv2.recoverPose from opencv.The final trajectory for both methods are plotted compared.

The values are saved in a csv file points.csv

### Final Output

<p align="center">
  <img src="/Images/video.gif" alt="video">
</p>


## **DEPENDANCIES**

- Python 3
- OpenCV
- Numpy
- Glob
- Matplotlib
- Copy (built-in)

## **FILE DESCRIPTION**

- Code Folder/[FINAL CODE.py](https://github.com/adheeshc/Visual-Odometry/blob/master/Code/FINAL%20CODE.py) - The final code without for Visual Odometry
- Code Folder/[Built_in.py](https://github.com/adheeshc/Visual-Odometry/blob/master/Code/Built_in.py) - The code made completely using Built-in functions
- Code Folder/[ReadCameraModel.py](https://github.com/adheeshc/Visual-Odometry/blob/master/Code/ReadCameraModel.py) - Loads camera intrisics and undistortion LUT from disk
- Code Folder/[UndistortImage.py](https://github.com/adheeshc/Visual-Odometry/blob/master/Code/UndistortImage.py) - Undistort an image using a lookup table
- Code Folder/[VIDEO.py](https://github.com/adheeshc/Visual-Odometry/blob/master/Code/VIDEO.py) - Used to display the 2 final plots - my code vs built-in

- Dataset folder - Contains link to dataset. Should have 3 folders - Frames, SIFT images and model

- Images folder - Contains images for github use (can be ignored)

- Output folder - Contains output videos and 2 output csv files
  - points_final.csv - This is the output points from the Built_in.py
  - updated2_final.csv - This is the output points from the FINAL_CODE.py

- References folder - Contains supplementary documents that aid in understanding

- Report folder - Contains [Project Report](https://github.com/adheeshc/Visual-Odometry/blob/master/Report/FINAL%20REPORT.pdf)

## **RUN INSTRUCTIONS**

- Make sure all dependancies are met
- Ensure the location of the input video files are correct in the code you're running
- Comment/Uncomment as reqd

- RUN Final_CODE.py for my code of Visual Odometry to generate the a new first csv file
- RUN Built_in.py for code made using completely Built-in functions to generate a new second csv file
- RUN VIDEO.py to use original csv files to display output

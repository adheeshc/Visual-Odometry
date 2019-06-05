import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import copy

myData1=np.genfromtxt('points_final.csv',delimiter = ',')
x_new1=myData1[:,1]
z_new1=myData1[:,3]
myData2=np.genfromtxt('updated2_final.csv',delimiter=',')
x_new2=myData2[:,0]
z_new2=myData2[:,1]

fig = plt.figure()
gs = plt.GridSpec(2,3)

for i in range(0,myData1.shape[0]):
	print("FRAME %d"%i)
	j=i+30
	img=cv2.imread("FRAMES/%d.jpg"%j)
	img2=cv2.imread("SIFT/%d.jpg"%j)
	plt.plot(x_new1[i],-z_new1[i],'o',color='blue')
	plt.plot(x_new2[i],z_new2[i],'o',color='red')
	img=cv2.resize(img, (0,0), fx=0.3, fy=0.3)
	img2=cv2.resize(img2, (0,0), fx=0.6, fy=0.6)
	cv2.imshow("Original_Image",img)
	cv2.moveWindow("Original_Image",800,100)
	cv2.imshow("SIFT_Image",img2)
	cv2.moveWindow("SIFT_Image",800,450)
	plt.pause(0.01)
	cv2.waitKey(1)
	
## UNCOMMENT HERE TO VIEW OUTPUT DIRECTLY WIHTOUT ACCESSING INDIVIDUAL FRAMES. Comment For loop above.

# plt.plot(x_new1,-z_new1,'o',color='blue')
# plt.plot(x_new2,z_new2,'o',color='red')
# plt.legend(['Built-In','Our Code'])
# plt.show()
import cv2
import numpy as np
import glob
from ReadCameraModel import ReadCameraModel
from UndistortImage import UndistortImage
import matplotlib.pyplot as plt
import os

np.set_printoptions(suppress=True)

def randomMatchingPoints(matchpts1,matchpts2):
    rand_index = np.random.randint(len(matchpts1), size=8)
    
    X1 = np.array([matchpts1[rand_index[0]],matchpts1[rand_index[1]],matchpts1[rand_index[2]],matchpts1[rand_index[3]],matchpts1[rand_index[4]],matchpts1[rand_index[5]],matchpts1[rand_index[6]],matchpts1[rand_index[7]]]) 
    X2 = np.array([matchpts2[rand_index[0]],matchpts2[rand_index[1]],matchpts2[rand_index[2]],matchpts2[rand_index[3]],matchpts2[rand_index[4]],matchpts2[rand_index[5]],matchpts2[rand_index[6]],matchpts2[rand_index[7]]]) 
    
    return X1, X2


def NormalizedFMatrix(points1,points2):

    dist1 = np.sqrt((points1[:,0]- np.mean(points1[:,0]))**2 + (points1[:,1]- np.mean(points1[:,1]))**2)
    dist2 = np.sqrt((points2[:,0]- np.mean(points2[:,0]))**2 + (points2[:,1]- np.mean(points2[:,1]))**2)

    m_dist1 = np.mean(dist1)
    m_dist2 = np.mean(dist2)
    
    scale1 = np.sqrt(2)/m_dist1
    scale2 = np.sqrt(2)/m_dist2
    
    t1 = np.array([[scale1, 0, -scale1*np.mean(points1[:,0])],[0, scale1, -scale1*np.mean(points1[:,1])],[0, 0, 1]])
    t2 = np.array([[scale2, 0, -scale2*np.mean(points2[:,0])],[0, scale2, -scale2*np.mean(points2[:,1])],[0, 0, 1]])

    
    U_x = (points1[:,0] - np.mean(points1[:,0]))*scale1
    U_y = (points1[:,1] - np.mean(points1[:,1]))*scale1
    V_x = (points2[:,0] - np.mean(points2[:,0]))*scale2
    V_y = (points2[:,1] - np.mean(points2[:,1]))*scale2
    
    A = np.zeros((len(U_x),9))
    
    for i in range(len(U_x)):
        
        A[i] = np.array([U_x[i]*V_x[i], U_y[i]*V_x[i], V_x[i], U_x[i]*V_y[i], U_y[i]*V_y[i], V_y[i], U_x[i], U_y[i], 1])
       
    U,S,V = np.linalg.svd(A)
    V = V.T
    F = V[:,-1].reshape(3,3)
    
    Uf, Sf, Vf = np.linalg.svd(F)
    SF = np.diag(Sf)
    SF[2,2] = 0
    
    F = Uf @ SF @ Vf
    F = t2.T @ F @ t1 
    F = F/F[2,2]
    
    return F
    

def EstimateEssentialMatrix(K,F):
    E = K.T @ F @ K
    Ue, Se, Ve = np.linalg.svd(E)
    SE = np.diag(Se)   
    SE[0][0] = 1
    SE[1][1] = 1
    SE[2][2] = 0
    
    E = Ue @ SE @ Ve
    E = E/np.linalg.norm(E)
    return E



def ExtractCameraPose(E):
    W = np.array(([0,-1,0],[1,0,0],[0,0,1]))
    U,S,V = np.linalg.svd(E)
    C1 = U[:,2]
    C2 = -U[:,2]
    C3 = U[:,2]
    C4 = -U[:,2]
    R1 = U @ W @ V
    R2 = U @ W @ V
    R3 = U @ W.T @ V
    R4 = U @ W.T @ V
    
    if np.linalg.det(R1)<0:
        R1=-1*R1
        C1=-1*C1
    if np.linalg.det(R2)<0:    
        R2=-1*R2
        C2=-1*C2
    if np.linalg.det(R3)<0:
        R3=-1*R3
        C3=-1*C3
    if np.linalg.det(R4)<0:
        R4=-1*R4
        C4=-1*C4

    return C1,C2,C3,C4,R1,R2,R3,R4


def New_LinearTriangulation(P1,P2,pts1,pts2):
    A = np.zeros((4,3))
    b = np.zeros((4,1))

    x = np.zeros((3,len(pts1)))

    pt1 = np.array(-np.eye(2,3))
    pt2 = np.array(-np.eye(2,3))

    for i in range(len(pts1)):
        pt1[:,2] = pts1[i,:]
        pt2[:,2] = pts2[i,:]

        A[0:2,:] = pt1.dot(P1[0:3,0:3])
        A[2:4,:] = pt2.dot(P2[0:3,0:3])

        b[0:2,:] = pt1.dot(P1[0:3,3:4])
        b[2:4,:] = pt2.dot(P2[0:3,3:4])

        cv2.solve(A,b,x[:,i:i+1],cv2.DECOMP_SVD)

    return x

def findZ(R1,t,pts1,pts2,K):
    Co = [[0],[0],[0]]
    Ro = np.eye(3,3)
    P1 = np.eye(3,4)
    P2 = np.hstack((R1,t))
    X1_p = np.array(New_LinearTriangulation(P1,P2,np.array(pts1),np.array(pts2)))
    X1 = np.vstack((X1_p,np.ones((1,len(pts1)))))

    X1= X1.reshape(-1,4)

    X1 = np.divide(X1,np.array([X1[:,3],X1[:,3],X1[:,3],X1[:,3]]).T)

    X1 = np.sum(P2@X1.T>0)

    return X1



fx, fy, cx, cy, G_camera_image, LUT=ReadCameraModel("Oxford_dataset/model")
K = np.zeros((3,3))
K[0][0] = fx
K[1][1] = fy
K[2][2] = 1
K[0][2] = cx
K[1][2] = cy

N = 3872
camera_centre=np.zeros((4,1))
camera_centre[3]=1
new_centre=[]
count_final=0
H_new=np.identity(4)
C_origin=np.zeros((3,1))
R_origin=np.identity(3)
final_points=[]

for i in range(30,N):
    print(i)
    img_current_frame = cv2.imread('FRAMES/%d.jpg'%i)
    img_next_frame = cv2.imread('FRAMES/%d.jpg'%(i+1))
    
    img_current_frame= cv2.rectangle(img_current_frame,(np.float32(50),np.float32(np.shape(img_current_frame)[0])),(np.float32(1250),np.float32(800)),(0,0,0),-1)
    img_next_frame= cv2.rectangle(img_next_frame,(np.float32(50),np.float32(np.shape(img_next_frame)[0])),(np.float32(1250),np.float32(800)),(0,0,0),-1)

    sift = cv2.xfeatures2d.SIFT_create()

    kp1, des1 = sift.detectAndCompute(img_current_frame,None)
    kp2, des2 = sift.detectAndCompute(img_next_frame,None)
    

    bf=cv2.BFMatcher()
    matches=bf.match(des1,des2)

    U = []
    V = []
    for m in matches:        
        pts_1 = kp1[m.queryIdx]
        x1,y1=pts_1.pt
        pts_2 = kp2[m.trainIdx]
        x2,y2=pts_2.pt
        U.append((x1,y1))
        V.append((x2,y2))
    U=np.array(U)
    V=np.array(V)

    Inliers_UN = []
    max_inliers = 0
    M = len(U)

    for i in range(500):
        X1,X2 = randomMatchingPoints(U,V)
        F_r = NormalizedFMatrix(X1,X2)
        Inliers_U = []
        Inliers_V = []
        Inliers = 0
        for j in range(len(U)):
            U1 = np.array([U[j][0],U[j][1],1]).reshape(1,-1)
            V1 = np.array([V[j][0],V[j][1],1]).reshape(1,-1)

            epiline1 = F_r @ U1.T
            epiline2 = F_r.T @ V1.T
            error_bottom = epiline1[0]**2 + epiline1[1]**2 + epiline2[0]**2 + epiline2[1]**2
            
            error = ((V1 @ F_r @ U1.T)**2)/error_bottom
            
            if error[0,0]<.008:
                Inliers+=1
                Inliers_U.append([U[j][0],U[j][1]])
                Inliers_V.append([V[j][0],V[j][1]])
                          
        if max_inliers < Inliers:
            max_inliers = Inliers
            Inliers_UN = Inliers_U
            Inliers_VN = Inliers_V
            F = F_r

    E=EstimateEssentialMatrix(K,F)
    C1,C2,C3,C4,R1,R2,R3,R4=ExtractCameraPose(E)
    C1=(C1).reshape(-1,1)
    C2=(C2).reshape(-1,1)
    C3=(C3).reshape(-1,1)
    C4=(C4).reshape(-1,1)
    
    T = [C1,C2,C3,C4]
    R = [R1,R2,R3,R4]
    
    number_good1=findZ(R1,C1,U,V,K)
    number_good2=findZ(R2,C2,U,V,K)
    number_good3=findZ(R3,C3,U,V,K)
    number_good4=findZ(R4,C4,U,V,K)
    
    count_numbers = [number_good1, number_good2,number_good3,number_good4]
    index = np.argmax(count_numbers)
    
    R_new = R[index]
    T_new = T[index]

    H_final= np.hstack((R_new,T_new))
    H_final=np.vstack((H_final,[0,0,0,1]))

    x_old=H_new[0][3]
    z_old=H_new[2][3]
    H_new=H_new@H_final
    x_test=H_new[0][3]
    z_test=H_new[2][3]

    cv2.imshow('11',img_current_frame)
    plt.plot([-x_old,-x_test],[z_old,z_test],'o')
    final_points.append([-x_old,-x_test,z_old,z_test])
    plt.pause(0.01)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
np.savetxt("updated2.csv", final_points, delimiter=",")
cv2.waitKey(1)
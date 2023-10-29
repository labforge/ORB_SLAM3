import time
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
img = cv.imread('simple.jpeg',0)

#np.set_printoptions(threshold=np.inf)

start = time.time()
orb = cv.ORB_create()
kp1 = orb.detect(img,None)
for i in kp1:
	#print(i.pt,end=' ')
	print(i.pt)
kp1, des1 = orb.compute(img, kp1)
print(type(kp1))
print(type(kp1[0]))
print(type(des1))
print(type(des1[0]))
end = time.time()
img21 = cv.drawKeypoints(img, kp1, None, color=(0,255,0), flags=0)
print("ORB")
print("Time Taken:" + str(end - start))
print("Number of key points detected by default:" + str(len(kp1)))
print("Sample - key point 0: " + str(kp1[0]))
print("Number of descriptors:" + str(len(des1)))
print("Descriptors:" + str(des1))
for i in range(len(des1)):
    print (i, end = " ")
    print (des1[i])
print("Descriptor size:" + str(len(des1[0])))
print("Sample - Decriptor 0:" + str(des1[0]))

start = time.time()
akaze = cv.AKAZE_create()
kp2 = akaze.detect(img,None)
print(type(kp2))
print(type(kp2[0]))

print("\n------------")
for i in kp1:
	print(i.class_id,end=' ')
print("\n------------")
for i in kp2:
	print(i.class_id,end=' ')
print("\n------------")

#Replace class id for orb and feep same orb detected key points into AKAZE decriptor generator
#for i in kp1:
	#i.class_id = kp2[0].class_id
#	i.class_id =0 

#kp2, des2 = akaze.compute(img, kp1)
kp2, des2 = akaze.compute(img, kp2)
end = time.time()
img22 = cv.drawKeypoints(img, kp2, None, color=(0,255,0), flags=0)
print("AKAZE")
print("Time Taken:" + str(end - start))
print("Number of key points detected by default:" + str(len(kp2)))
print("Sample - key point 0: " + str(kp2[0]))
print("Number of descriptors:" + str(len(des2)))
print("Descriptors:" + str(des2))
for i in range(len(des2)):
    print (i, end = " ")
    print (des2[i])
print("Descriptor size:" + str(len(des2[0])))
print("Sample - Decriptor 0:" + str(des2[0]))


fig = plt.figure(figsize=(30, 10))
fig.add_subplot(1, 2,1)
plt.imshow(img21)
plt.axis('off')
plt.title("ORB")
fig.add_subplot(1, 2,2)
plt.imshow(img22)
plt.axis('off')
plt.title("AKAZE")
plt.show()

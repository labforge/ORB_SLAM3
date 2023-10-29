import cv2
import argparse
import numpy as np
from numpy.linalg import inv
#rot = -0.0007439316834764603, -0.0008682345518261254, 0.0036869066931661913
#trans = -0.14386635075261975, -6.75640508940185e-05, -0.0028363773910315285

R_vec = np.array([ -0.0007439316834764603, -0.0008682345518261254, 0.0036869066931661913 ]).reshape(3, 1)
print(str(R_vec)+"\n")

T_vec = np.array([ -0.14386635075261975, -6.75640508940185e-05, -0.0028363773910315285 ]).reshape(3, 1)
print(str(T_vec)+"\n")


R_mat = cv2.Rodrigues(R_vec)[0].reshape(3,3)
print(str(R_mat)+"\n")

RT_mat = np.hstack((R_mat, T_vec)).reshape(3,4)
print(str(R_mat)+"\n")

RT_mat = np.vstack((RT_mat, [0,0,0,1])).reshape(4,4)
print(str(RT_mat)+"\n")

inv_mat = inv(RT_mat)
print(str(inv_mat)+"\n")

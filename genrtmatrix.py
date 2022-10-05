import cv2
import argparse
import numpy as np
#rot = -0.0007439316834764603, -0.0008682345518261254, 0.0036869066931661913
#trans = -0.14386635075261975, -6.75640508940185e-05, -0.0028363773910315285

R_vec = np.array([ -0.0007439316834764603, -0.0008682345518261254, 0.0036869066931661913 ]).reshape(3, 1)
T_vec = np.array([ -0.14386635075261975, -6.75640508940185e-05, -0.0028363773910315285 ]).reshape(3, 1)
R_mat = cv2.Rodrigues(R_vec)[0].reshape(3,3)
RT_mat = np.hstack((R_mat, T_vec)).reshape(3,4)
RT_mat = np.vstack((RT_mat, [0,0,0,1])).reshape(4,4)
print(RT_mat)

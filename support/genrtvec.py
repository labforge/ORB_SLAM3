import cv2
import argparse
import numpy as np
from numpy.linalg import inv

R_inv_mat = np.array([
	 [0.9999994457734953, 0.0007916877528168117, 0.0006940340102242531],
         [-0.0008233639921576076, 0.9988994619156757, 0.04689549078870055],
         [-0.0006561436136444361, -0.04689603624058988, 0.9988995601463054],
        ])

#R_inv_mat = CreateMat(3, 3, np.float32)
print(str(R_inv_mat)+"\n")

R_mat = inv(R_inv_mat)
print(str(R_mat)+"\n")

R_vec,_ = cv2.Rodrigues(R_mat)
print(str(R_vec)+"\n")

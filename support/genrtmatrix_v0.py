import argparse

# read strandsXXXXX_YYYYY_AAAAA_mBB.txt
# The rotation (Euler angles) and the position of the head. 
# R_x, R_y, R_z, X, Y, Z
# R_vec = (R_x, R_y, R_z), reshape->(3, 1)
# T_vec = (X, Y, Z), reshape->(3, 1)
def gen_RT_matrix(path):
    with open(path, 'r') as f:
        lines = f.readlines()
        lines = lines[0].split(' ')
        R_vec = np.array([float(lines[3]),float(lines[5]), float(lines[4])]).reshape(3, 1)
        T_vec = np.array([float(lines[0]),float(lines[1]), float(lines[2])]).reshape(3, 1)
        R_vec = np.array(R_vec).reshape(3,1)
        T_vec = np.array(T_vec).reshape(3,1)
        R_mat = cv2.Rodrigues(R_vec)[0].reshape(3,3)
        RT_mat = np.hstack((R_mat, T_vec)).reshape(3,4)
        RT_mat = np.vstack((RT_mat, [0,0,0,1])).reshape(4,4)
        return inv(RT_mat)

def main():
    parser = argparse.ArgumentParser(description='Gen RT Matrix')
    parser.add_argument('--path', type = str, default=r'rtinput.txt')
    args = parser.parse_args()
    print(args.path)
    RT_mat=gen_RT_matrix(args.path)
    print(RT_mat)


if __name__ == '__main__':
    main()

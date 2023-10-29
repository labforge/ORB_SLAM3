import csv
import numpy as np
import cv2 as cv
import sys
'''
with open('data.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        print(row[0])
'''
keypointsLeft = cv.FileStorage('keypointsLeft.yml', cv.FileStorage_READ)
keypointsLeft_r = cv.FileStorage('keypointsLeft_r.yml', cv.FileStorage_WRITE)
node = keypointsLeft.getNode('img1')
for i in range(node.size()):
    print (node.at(i).string())
keypointsLeft.release()
keypointsLeft_r.release()

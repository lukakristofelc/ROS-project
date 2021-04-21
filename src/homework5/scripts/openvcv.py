import json
import numpy as np
from matplotlib import pyplot as plt
import cv2
from PIL import Image
from scipy.spatial import distance

ros_map = json.load(open("map.json","r"))
map_valus={-1:0,0:255,100:0}
ros_map = map(lambda a : map_valus[a],ros_map)
ros_map = np.array(list(ros_map))
ros_map = np.resize(ros_map,(512,544)).astype("uint8")

#cv2.imwrite("original.jpg",ros_map)
#cv2.imshow("Original",ros_map)

img = ros_map
size = np.size(img)
skel = np.zeros(img.shape, np.uint8)

element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
element2 = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))

while True:
    open = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
    temp = cv2.subtract(img, open)
    eroded = cv2.erode(img, element2)
    skel = cv2.bitwise_or(skel,temp)
    img = eroded.copy()
    if cv2.countNonZero(img)==0:
        break

#element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
#skel = cv2.dilate(skel,element)
#element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
#skel = cv2.erode(skel,element)

#cv2.imwrite("original.jpg",img)
#cv2.imshow("skeleton",skel)

elements = [[[1,0,1],[0,1,0],[0,1,0]],[[0,1,0],[0,1,1],[1,0,0]],[[0,0,1],[1,1,0],[0,0,1]],[[1,0,0],[0,1,1],[0,1,0]],[[0,1,0],[0,1,0],[1,0,1]],[[0,0,1],[1,1,0],[0,1,0]],[[1,0,0],[0,1,1],[1,0,0]],[[0,1,0],[1,1,0],[0,0,1]],[[1,0,0],[0,1,0],[1,0,1]],[[1,0,1],[0,1,0],[1,0,0]],[[1,0,1],[0,1,0],[0,0,1]],[[0,0,1],[0,1,0],[1,0,1]]]
elements = np.array(elements)

intersections = np.zeros(img.shape, np.uint8)
for element in elements:
    temp = cv2.morphologyEx(skel,cv2.MORPH_HITMISS,element)
    intersections = cv2.bitwise_or(intersections,temp)

#cv2.imshow("Test",intersections)
#cv2.waitKey(0)

img = ros_map
intersection_coords = np.transpose(np.nonzero(intersections))

centres = []
all_points = []

for intersection_coord in intersection_coords:
    if len(centres)!=0:
        d = distance.cdist([intersection_coord],centres)
    if len(centres)==0 or np.min(d)>10:
        all_points.append([])
        all_points[-1].append(intersection_coord)
        centres.append(intersection_coord)
    else:
        i = np.argmin(d)
        all_points[i].append(intersection_coord)
        centres[i] = np.mean(all_points[i],axis=0)

centres = np.array(centres).astype("int")

for centre in centres:
    cv2.circle(img,tuple(np.flip(centre)),3,(0,0,255))

cv2.imwrite("intersections.jpg",ros_map)
cv2.imshow("Test",img)
cv2.waitKey(0)
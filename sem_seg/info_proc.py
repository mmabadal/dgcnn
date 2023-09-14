import os
import copy
import ros_numpy
import numpy as np
from PIL import Image
from dgcnn.msg import info_bbs
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point32, Polygon


def set_margin(points, point1, point2, margin):

    for i, point in enumerate(points):
        if i<2:
            if point[0] >= point1[0]:
                point[0] += margin
                point[1] -= margin
            else:
                point[0] -= margin
                point[1] += margin
            points[i] = point

    for i, point in enumerate(points):
        if i>1:
            if point[0] >= point2[0]:
                point[0] += margin
                point[1] -= margin
            else:
                point[0] -= margin
                point[1] += margin
            points[i] = point

    return points


def points_to_img(points_list, id, c_info, path):

    decimation = c_info.binning_x

    P = c_info.P 

    height = c_info.height/decimation
    width = c_info.width/decimation
    
    P_array = np.array([[P[0]/decimation, P[1]/decimation, P[2]/decimation, P[3]/decimation],
                       [P[4]/decimation, P[5]/decimation, P[6]/decimation, P[7]/decimation],
                       [P[8],            P[9],            P[10],           P[11]]])

     # 1988    0     971         x        u  =  1988*x + 971*z 
     #   0    1988   714   *     y    =   v  =  1988*y + 714*z 
     #   0     0      1          z        w  =  z 

    points_list2 = list()

    keyframes = os.listdir(path)
    for keyframe in keyframes:
        if "left" in keyframe:
            if id in keyframe:
                key = Image.open(os.path.join(path, keyframe))
                break
    k = -1

    for points in points_list:

        points2 = list()

        k = k+1
        if k == 4:
            k = 0
        colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),(0,255, 255),(255,255,255),(128,0,0),(0,128,0),(0,0,128),(128,128,0),(128,0,128)]

        for point in points:

            xyz = np.array([[point[0]],
                           [point[1]],
                           [point[2]],
                           [1]])

            uvw = np.matmul(P_array, xyz)

            u = uvw[0][0]
            v = uvw[1][0]
            w = uvw[2][0]

            xdisp = int(u/w) # = (1988*x + 971*z) / z
            ydisp = int(v/w) # = (1988*y + 714*z) / z

            xdisp = int(np.clip(xdisp, 1, width-1))
            ydisp = int(np.clip(ydisp, 1, height-1))

            key.putpixel((xdisp, ydisp), colors[k])

            xydisp = np.array((ydisp, xdisp))
            points2.append(xydisp)

        points_list2.append(points2)

    key.save("/home/bomiquel/Desktop/" + str(id) + "_colour.png")

    return points_list2

def get_bb(info, margin, id, c_info, path):

    infobbs = info_bbs()
    p1 = Point32()
    p2 = Point32()
    p3 = Point32()
    p4 = Point32()
    polygon = Polygon()

    info_pipes_list = info[0]
    info_valves_list = info[2]

    points_list = list()

    for pipe_info in info_pipes_list:

        chain = pipe_info[0]
        elbow_list = pipe_info[1]
        vector_list = pipe_info[2]

        vector = vector_list[0][0:3]

        point1 = chain[0][0:3]
        point2 = point1 + vector

        center = point1 + vector/2

        vector_orth = np.array([-vector[1], vector[0], 0])
        vector_orth = vector_orth/np.linalg.norm(vector_orth)
        vector_orth = 0.06 * vector_orth

        point3 = point1 + vector_orth/2
        point4 = point1 - vector_orth/2
        point5 = point2 + vector_orth/2
        point6 = point2 - vector_orth/2

        points1 = [point1, point1, point2, point2]
        points_list.append(points1)
        points2 = [point3, point4, point5, point6]
        points_list.append(points2)
        points3 = set_margin(points2, point1, point2, margin)
        points_list.append(points3)

        print("after pipe")
        print(points_list)

        for i, elbow in enumerate(elbow_list):

            point1 = elbow[0:2]
            vector = vector_list[i+1][0:3]

            point2 = point1 + vector

            center = point1 + vector/2

            vector_orth = np.array([-vector[1], vector[0],0])
            vector_orth = vector_orth/np.linalg.norm(vector_orth)
            vector_orth = 0.06 * vector_orth

            point3 = point1 + vector_orth/2
            point4 = point1 - vector_orth/2
            point5 = point2 + vector_orth/2
            point6 = point2 - vector_orth/2

            points1 = [point1, point1, point2, point2]
            points_list.append(points1)
            points2 = [point3, point4, point5, point6]
            points_list.append(points2)
            points3 = set_margin(points2, point1, point2, margin)
            points_list.append(points3)

            print("after elbow")
            print(points_list)


    for valve_info in info_valves_list:

        center = valve_info[0][0:3]
        vector = valve_info[1][0:3]
        
        point1 = center - (vector/2)
        point2 = center + (vector/2)

        vector_orth = np.array([-vector[1], vector[0],0])
        vector_orth = vector_orth/np.linalg.norm(vector_orth)
        vector_orth = 0.07 * vector_orth

        point3 = point1 + vector_orth/2
        point4 = point1 - vector_orth/2
        point5 = point2 + vector_orth/2
        point6 = point2 - vector_orth/2

        points1 = [point1, point1, point2, point2]
        points_list.append(points1)
        points2 = [point3, point4, point5, point6]
        points_list.append(points2)
        points3 = set_margin(points2, point1, point2, margin)
        points_list.append(points3)

        print("after valve")
        print(points_list)

    print("final 3d")
    print(points_list)
    
    points_list_2 = points_to_img(points_list, id, c_info, path)


    print("final 2d")
    print(points_list_2)

    for i, points in enumerate(points_list_2):

        p1.x = points[0][0]
        p1.y = points[0][1]
        p1.z = 0
        p2.x = points[1][0]
        p2.y = points[1][1]
        p2.z = 0
        p3.x = points[2][0]
        p3.y = points[2][1]
        p3.z = 0
        p4.x = points[3][0]
        p4.y = points[3][1]
        p4.z = 0

        polygon.points.append(p1)
        polygon.points.append(p2)
        polygon.points.append(p3)
        polygon.points.append(p4)

        polygon2 = copy.deepcopy(polygon)
        infobbs.bbs.append(polygon2)
        
        polygon.points.clear()

    return infobbs
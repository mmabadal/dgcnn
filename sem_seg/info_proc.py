import copy
import numpy as np
from dgcnn.msg import info_bb
from dgcnn.msg import info_bbs
from stereo_msgs.msg import DisparityImage




def set_margin(points, center, margin):

    for i, point in enumerate(points):

        if point[0] >= center[0]:
            point[0] += margin
        else:
            point[0] -= margin

        if point[1] >= center[1]:
            point[1] += margin
        else:
            point[1] -= margin

        points[i] = point

    return points


 
def get_bb(info, margin, dispatity):

    info_bb = info_bb()
    info_bbs = info_bbs()

    # TODO delete this
    info = np.load("/home/bomiquel/SLAM_ws/src/dgcnn/test_polygon/out/1604421321894689_info_ref.npy", allow_pickle = True)

    info_pipes_list = info[0]
    info_connexions_list = info[1]
    info_valves_list = info[2]
    instances_ref_pipe_list = info[3]

    points_list = list()

    margin = 0.05

    print(info_pipes_list)

    for pipe_info in info_pipes_list:

        chain = pipe_info[0]
        elbow_list = pipe_info[1]
        vector_list = pipe_info[2]

        vector = vector_list[0][0:2]

        vector_orth = np.array([-vector[1], vector[0]])
        vector_orth = vector_orth/np.linalg.norm(vector_orth)
        vector_orth = 0.05 * vector_orth

        point1 = chain[0][0:2]

        point2 = point1 + vector

        center = point1 + vector/2

        point3 = point1 + vector_orth/2
        point4 = point1 - vector_orth/2
        point5 = point2 + vector_orth/2
        point6 = point2 - vector_orth/2

        points = [point3, point4, point5, point6]
        points = set_margin(points, center, margin)
        points_list.append(points)


        for i, elbow in enumerate(elbow_list):

            vector = vector_list[i+1][0:2]
            vector_orth = np.array([-vector[1], vector[0]])
            vector_orth = vector_orth/np.linalg.norm(vector_orth)
            vector_orth = 0.05 * vector_orth

            point1 = elbow[0:2]

            point2 = point1 + vector

            center = point1 + vector/2

            point3 = point1 + vector_orth/2
            point4 = point1 - vector_orth/2
            point5 = point2 + vector_orth/2
            point6 = point2 - vector_orth/2

            points = [point3, point4, point5, point6]
            points = set_margin(points, center, margin)
            points_list.append(points)

    for valve_info in info_valves_list:

        center = valve_info[0][0:2]
        vector = valve_info[1][0:2]
        vector_orth = np.array([-vector[1], vector[0]])
        vector_orth = vector_orth/np.linalg.norm(vector_orth)
        vector_orth = 0.09 * vector_orth

        point1 = center-(vector/2)
        point2 = center+(vector/2)

        point3 = point1 + vector_orth/2
        point4 = point1 - vector_orth/2
        point5 = point2 + vector_orth/2
        point6 = point2 - vector_orth/2

        points = [point3, point4, point5, point6]
        points = set_margin(points, center, margin)
        points_list.append(points)

    
    # TODO project to XY through disp
    for i, points in enumerate(points_list):
        for j, p in enumerate(points):
            p = p * 40
            p = p.astype(int)
            p = np.absolute(p)
            points_list[i][j] = p

    for i, points in enumerate(points_list):
        info_bb.x1 = points[0][0]
        info_bb.y1 = points[0][1]
        info_bb.x2 = points[1][0]
        info_bb.y2 = points[1][1]
        info_bb.x3 = points[2][0]
        info_bb.y3 = points[2][1]
        info_bb.x4 = points[3][0]
        info_bb.y4 = points[3][1]
        info_bb2 = copy.deepcopy(info_bb)
        info_bbs.bbs.append(info_bb2)

    print(points_list)
    return info_bbs





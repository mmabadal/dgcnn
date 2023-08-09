import copy
import ros_numpy
import numpy as np
from dgcnn.msg import info_bbs
from stereo_msgs.msg import DisparityImage
from geometry_msgs.msg import Point32, Polygon


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


def points_to_img(points_list, pointcloud, disparity):

    print("-----------------------------------------")
    print("-----------------------------------------")
    print(points_list)
    print("-----------------------------------------")
   
    pc_xmin, pc_ymin, *_ = pointcloud.min(axis=0)
    pc_xmax, pc_ymax, *_ = pointcloud.max(axis=0)
    pc_xrange = pc_xmax - pc_xmin
    pc_yrange = pc_ymax - pc_ymin

    print(pc_xmin)
    print(pc_xmax)
    print(pc_xrange)
    print(pc_ymin)
    print(pc_ymax)
    print(pc_yrange)

    print("-----------------------------------------")
    disp = ros_numpy.numpify(disparity.image)
    disp_pos = np.where(disp>15)
    disp_pos_np = np.vstack(disp_pos).T

    disp_ymin, disp_xmin = disp_pos_np.min(axis=0)
    disp_ymax, disp_xmax = disp_pos_np.max(axis=0)
    disp_xrange = disp_xmax - disp_xmin
    disp_yrange = disp_ymax - disp_ymin

    print(disp_xmin)
    print(disp_xmax)
    print(disp_xrange)
    print(disp_ymin)
    print(disp_ymax)
    print(disp_yrange)
    print("-----------------------------------------")

    points_list2 = list()

    for points in points_list:

        points2 = list()

        for point in points:

            xpc = point[0]
            ypc = point[1]

            ratio_xpc = (xpc-pc_xmin)/pc_xrange
            xdisp = int(disp_xmin + (ratio_xpc*disp_xrange))
            
            ratio_ypc = (ypc-pc_ymin)/pc_yrange
            ydisp = int(disp_ymin + (ratio_ypc*disp_yrange))

            xydisp = np.array((xdisp,ydisp))
            points2.append(xydisp)
        points_list2.append(points2)

    print(xpc)
    print(ratio_xpc)
    print(xdisp)
    
    print(ypc)
    print(ratio_ypc)
    print(ydisp)

    print("-----------------------------------------")
    print("-----------------------------------------")

    return points_list2

 

def get_bb(info, margin, pointcloud, disparity):

    infobbs = info_bbs()
    p1 = Point32()
    p2 = Point32()
    p3 = Point32()
    p4 = Point32()
    polygon = Polygon()

    # TODO delete this
    # info = np.load("/home/bomiquel/SLAM_ws/src/dgcnn/test_polygon/out/1604421321894689_info_ref.npy", allow_pickle = True)

    info_pipes_list = info[0]
    info_connexions_list = info[1]
    info_valves_list = info[2]
    instances_ref_pipe_list = info[3]

    points_list = list()

    margin = 0.05

    print(f"LEN INFO PIPES LIST: {len(info_pipes_list)}")
    print(f"LEN INFO VALVES LIST: {len(info_valves_list)}")

    for pipe_info in info_pipes_list:

        chain = pipe_info[0]
        elbow_list = pipe_info[1]
        vector_list = pipe_info[2]

        vector = vector_list[0][0:2]

        vector_orth = np.array([-vector[1], vector[0]])
        vector_orth = vector_orth/np.linalg.norm(vector_orth)
        vector_orth = 0.05 * vector_orth

        point1 = chain[0][0:2]

        # point2 = point1 + vector

        point2 = chain[-1][0:2]

        center = point1 + vector/2

        point3 = point1 # + vector_orth/2
        point4 = point1 # - vector_orth/2
        point5 = point2 # + vector_orth/2
        point6 = point2 # - vector_orth/2

        points = [point3, point4, point5, point6]
        # points = set_margin(points, center, margin)
        points_list.append(points)


        for i, elbow in enumerate(elbow_list):

            vector = vector_list[i+1][0:2]
            vector_orth = np.array([-vector[1], vector[0]])
            vector_orth = vector_orth/np.linalg.norm(vector_orth)
            vector_orth = 0.05 * vector_orth

            point1 = elbow[0:2]

            point2 = point1 + vector

            center = point1 + vector/2

            point3 = point1 # + vector_orth/2
            point4 = point1 # - vector_orth/2
            point5 = point2 # + vector_orth/2
            point6 = point2 # - vector_orth/2

            points = [point3, point4, point5, point6]
            # points = set_margin(points, center, margin)
            points_list.append(points)

    for valve_info in info_valves_list:

        center = valve_info[0][0:2]
        vector = valve_info[1][0:2]
        vector_orth = np.array([-vector[1], vector[0]])
        vector_orth = vector_orth/np.linalg.norm(vector_orth)
        vector_orth = 0.09 * vector_orth

        point1 = center-(vector/2)
        point2 = center+(vector/2)

        point3 = center # point1 + vector_orth/2
        point4 = center # point1 - vector_orth/2
        point5 = center # point2 + vector_orth/2
        point6 = center # point2 - vector_orth/2

        points = [point3, point4, point5, point6]
        # points = set_margin(points, center, margin)
        points_list.append(points)

    points_list_2 = points_to_img(points_list, pointcloud, disparity)
    print(f"LEN POINTS LIST 2: {len(points_list_2)}")

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
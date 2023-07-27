import os
import numpy as np
import copy
 
if __name__=='__main__':

    # info = np.load("a/b/c")

    # info_pipes_list = info[0]
    # info_connexions_list = info[1]
    # info_valves_list = info[2]
    # instances_ref_pipe_list = info[3]
    
    # info3 = [info_pipes_list2, info_connexions_list2, info_valves_list2, instances_ref_pipe_list]
    # info_pipes_list = info3[0]
    # info_valves_list = info3[2]

    info_pipes_list = list()
    info_pipes_list.append([np.array([[1,2,3], [4,5,6], [4,5,6], [4,5,6], [4,5,6]]),[np.array([[1,2,3], [4,5,6]])], [np.array([[1,2,3], [4,5,6]])]])

    info_valves_list = list()
    info_valves_list.append([np.array([0.2,0.2,0.2]),np.array([[1,2,3], [4,5,6]]),1,np.array([[1,2,3], [4,5,6]]), np.array([0.5])])

    points_list = list()

    margin = 0.05

    for pipe_info in info_pipes_list:

        chain = pipe_info[0]
        elbow_list = pipe_info[1]
        vector_list = pipe_info[2]

        vector = np.array([[0,0], [0,0]])
        vector[0] = vector_list[0][0][0:2]
        vector[1] = vector_list[0][1][0:2]

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
        points = margin(points, center, margin)
        points_list.append(points)

        for i, elbow in enumerate(elbow_list):

            vector = np.array([[0,0], [0,0]])
            vector[0] = vector_list[i+1][0][0:2]
            vector[1] = vector_list[i+1][1][0:2]
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
            points = margin(points, center, margin)
            points_list.append(points)

    for valve_info in info_valves_list:

        center = valve_info[0][0:2]
        vector = np.array([[0,0], [0,0]])
        vector[0] = valve_info[1][0][0:2]
        vector[1] = valve_info[1][1][0:2]
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
        points = margin(points, center, margin)
        points_list.append(points)

    print(points_list)


def margin(points, center, margin):

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


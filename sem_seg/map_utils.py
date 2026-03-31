import os
import re
import sys
import time
import copy
import math
import argparse
import get_info
import numpy as np
import scipy as scp
import open3d as o3d
import get_instances
import conversion_utils
from natsort import natsorted
import matplotlib.pyplot as plt
from scipy.spatial import distance
from mpl_toolkits.mplot3d import Axes3D


def check_near(arr1, arr2, dist):
    for point1 in arr1:
        for point2 in arr2:
            d = get_instances.get_distance(point1, point2, 2)
            if d < dist:
                return True
    return False

def find_pipe_groups(pipes, threshold=0.10, check_near_func=None):

    merge_lists = []
    n = len(pipes)

    for i in range(n):
        merge_list_current = [i]
        for j in range(n):
            if i != j:
                if check_near_func(pipes[i][0], pipes[j][0], threshold):
                    merge_list_current.append(j)

        merged = False
        for merge_list in merge_lists:
            if any(pipe in merge_list for pipe in merge_list_current):
                merge_list.extend(p for p in merge_list_current if p not in merge_list)
                merged = True
                break

        if not merged:
            merge_lists.append(merge_list_current)

    # Merge overlapping groups
    i = 0
    while i < len(merge_lists):
        j = i + 1
        while j < len(merge_lists):
            if any(p in merge_lists[i] for p in merge_lists[j]):
                merge_lists[i].extend(p for p in merge_lists[j] if p not in merge_lists[i])
                del merge_lists[j]
            else:
                j += 1
        i += 1

    # Sort groups for clarity
    merge_lists = [sorted(merge_list) for merge_list in merge_lists]

    return merge_lists


def get_info_map(info_world):

    info_pipes_map_list = list()
    info_connexions_map_list = list()
    info_valves_map_list = list()
    pipe_inst_map_list = list()

    info_pipes_world_list = info_world[0]
    info_connnexions_world_list = info_world[1]
    info_valves_world_list = info_world[2]
    pipe_inst_world_list = info_world[3]

    pipe_merge_list_all = find_pipe_groups(info_pipes_world_list, threshold=0.10, check_near_func=check_near)

    grouped_indices = set(i for merge_list in pipe_merge_list_all for i in merge_list)
    all_indices = set(range(len(info_pipes_world_list)))
    isolated = sorted(all_indices - grouped_indices)

    for i in isolated:
        new_pipe = info_pipes_world_list[i]
        new_pipe.append(1)                                          # count 1
        info_pipes_map_list.append(new_pipe)

    k_pipe = 0

    for pipe_merge_list in pipe_merge_list_all:
        inst_idx_list = list()   
        for pipe in pipe_merge_list:
            inst_idx_list.append(info_pipes_world_list[pipe][3])

        merge_inst_idx_list = sorted({item for sublist in inst_idx_list for item in sublist})

        inst_list = list()
        for inst_idx in merge_inst_idx_list:
            inst = pipe_inst_world_list[inst_idx]
            inst_list.append(inst)
            
        new_inst = np.vstack(inst_list)
        new_inst = np.hstack((new_inst,new_inst))  # add fake colors

        info_pipe_map = get_info.get_info(new_inst, models=0, method="skeleton", close = 12) # get pipe instance info list( list( list(chain1, start1, end1, elbow_list1, vector_chain_list1), ...), list(connexions_points)) 
        
        for j, pipe_info in enumerate(info_pipe_map[0]):                         # add corresponding inst i to all chains              
            pipe_info.append(merge_inst_idx_list)
            info_pipes_map_list.append(pipe_info)                                # stack pipes info

        for j, connexion_info in enumerate(info_pipe_map[1]):                    # stack conenexions info
            connexion_info[1] = [x+k_pipe for x in connexion_info[1]]
            info_connexions_map_list.append(connexion_info)

        k_pipe += len(info_pipe_map[0])                                          # update actual pipe idx

    # for i, info_connexion_world in enumerate(info_connnexions_world_list):
    #     merged = False

    #     for j, info_connexion_map in enumerate(info_connexions_map_list):
    #         dist = get_instances.get_distance(info_connexion_world[0], info_connexion_map[0], 2) 
    #         if dist < 0.15:  # las valvulas tienen una longitud de 0.18
    #             info_connexions_map_list[j][0] = (info_connexions_map_list[j][0] + info_connexion_world[0])/2
    #             info_connexions_map_list[j][2] = info_connexions_map_list[j][2]+1         # count +1
    #             merged = True
    #             break

    #     if merged == False:
    #         count_c = 1     # count 1
    #         info_connexion_world.append(count_c)
    #         info_connexions_map_list.append(info_connexion_world)

    # for i, info_connexion_map in enumerate(info_connexions_map_list):        # for each connexion

    #     near_pipes_list = list()
    #     for j, info_pipe_map in enumerate(info_pipes_map_list):                     # get near pipes
    #         c_p = info_connexions_map_list[i][0]                                   # central point
    #         d_to_start = get_instances.get_distance(c_p, info_pipe_map[0][0], 3)    # get distance from conn central point to pipe start
    #         d_to_end = get_instances.get_distance(c_p, info_pipe_map[0][-1], 3)     # get distance from conn central point to pipe end
    #         if d_to_start <= 0.05 or d_to_end <= 0.05:                              # if distance < thr             //PARAM
    #             near_pipes_list.append(j)                                           # append pipe as near
    #             break  

    #     info_connexions_map_list[i][1] = near_pipes_list                           # replace near pipes to conn info [central_point, near_pipes]

    for i, info_valve_world in enumerate(info_valves_world_list):
        merged = False

        for j, info_valve_map in enumerate(info_valves_map_list):
            print(info_valves_map_list)
            print("----")
            dist = get_instances.get_distance(info_valve_world[0], info_valve_map[0], 2) 
            if dist < 0.15: # las valvulas tienen una longitud de 0.18
                info_valves_map_list[j][0] = (info_valves_map_list[j][0] + info_valve_world[0])/2
                info_valves_map_list[j][4].append(info_valve_world[2])    # add valve type to valve type list
                info_valves_map_list[j][5] = info_valves_map_list[j][5]+1 # count +1
                two = sum(i <= 1 for i in info_valves_map_list[j][4])     # get number of times valve is identified as two way
                three = sum(i >= 2 for i in info_valves_map_list[j][4])   # get number of times valve is identified as three way
                new_type = 0
                if two<three:  # new type = type with most votes
                    new_type = 2
                info_valves_map_list[j][2] = new_type

                merged = True
                print(info_valves_map_list)

                break
        
        if merged == False:
            count_v = 1     # count 1
            info_valve_world.append(count_v)
            info_valves_map_list.append(info_valve_world)

    for i, info_valve_map in enumerate(info_valves_map_list):        # for each valve

        near_pipes_list = list()
        for j, info_pipe_map in enumerate(info_pipes_map_list):                     # get near pipes
            c_p = info_valves_map_list[i][0]                                        # central point
            d_to_start = get_instances.get_distance(c_p, info_pipe_map[0][0], 3)    # get distance from valve central point to pipe start
            d_to_end = get_instances.get_distance(c_p, info_pipe_map[0][-1], 3)     # get distance from valve central point to pipe end
            if d_to_start <= 0.25 or d_to_end <= 0.25:                              # if distance < thr             //PARAM
                near_pipes_list.append(j)                                           # append pipe as near
                break  

        info_valves_map_list[i][3] = near_pipes_list                                # replace near pipes to valve info [central_point, vector, max_id, near_pipes]

    info_pipes_map_list_elongated = elongate_pipes(info_pipes_map_list, info_connexions_map_list, info_valves_map_list, 0.10, 0.10)

    info_map = [info_pipes_map_list_elongated, info_connexions_map_list, info_valves_map_list, pipe_inst_map_list]

    return info_map


def elongate_pipes(info_pipes_list, info_connexions_list, info_valves_list, elongation, thr_near):

    for info_pipe in info_pipes_list:
        start = info_pipe[0][0]
        end = info_pipe[0][-1]

        if not info_pipe[1]:   # no elbow
            vector = info_pipe[2][0]
            vector_length= np.linalg.norm(vector)
            vector_unit = vector / vector_length
            vector_elong = vector_unit * elongation

            if start not near_conn(start,info_connexions_list,thr_near):
                newstart = start-vector_elong
                info_pipe[0]= np.concatenate([newstart,info_pipe[0]])   # new startpoint
                info_pipe[2][0] = info_pipe[2][0]+vector_elong          # elongate vector

            if end not near_conn(end,info_connexions_list,thr_near):
                newend = end + vector_elong
                info_pipe[0]= np.concatenate([info_pipe[0],newend])     # new endpoint
                info_pipe[2][0] = info_pipe[2][0]+vector_elong          #elongate vector

    return info_pipes_list
        

def near_conn(point, info_connexions_list, thr_near):
    for info_connexion in info_connexions_list:
        if np.linalg.norm(point - info_connexion[0]) < thr_near:
            return True
    return False


def clean_map(info_map, count_thr):

    info_pipes_map_list = info_map[0]
    info_connexions_map_list = info_map[1]
    info_valves_map_list = info_map[2]
    info_inst_pipe_map_list = info_map[3]

    pipe_del_list = list()
    for i, info_pipe_map in enumerate(info_pipes_map_list):        # for each pipe
        if info_pipe_map[4] <= count_thr:
            pipe_del_list.append(i)
    for i in sorted(pipe_del_list, reverse=True):  # delete pipes
        del info_pipes_map_list[i]  

    connexion_del_list = list()
    for i, info_connexion_map in enumerate(info_connexions_map_list):        # for each connexion
        if info_connexion_map[2] <= count_thr:
            connexion_del_list.append(i)
    for i in sorted(connexion_del_list, reverse=True):  # delete connexions
        del info_connexions_map_list[i]  

    valve_del_list = list()
    for i, info_valve_map in enumerate(info_valves_map_list):        # for each valve
        if info_valve_map[5] <= count_thr:
            valve_del_list.append(i)
    for i in sorted(valve_del_list, reverse=True):  # delete valves
        del info_valves_map_list[i]  

    # TODO CAN DELETE PIPES SHORT

    info_map = [info_pipes_map_list, info_connexions_map_list, info_valves_map_list, info_inst_pipe_map_list]

    return info_map


if __name__ == "__main__":


    parser = argparse.ArgumentParser()
    parser.add_argument('--path_in', help='path in info.')
    parser.add_argument('--path_out', help='path out info.')
    parsed_args = parser.parse_args(sys.argv[1:])

    path_in = parsed_args.path_in
    path_out = parsed_args.path_out

    count = 0
    count_target = 10
    count_thr = 1

    T_time = 0

    info_slam = [[], [], [], []]  # [pipes, connexions, valves, inst_pipes]

    inst_idx = 0

    for file in natsorted(os.listdir(path_in)):

        if "_info_map.npy" in file:

            print("\n\n")
            print("working on: " + file)

            file_name, _ = os.path.splitext(file)
            count += 1

            file_path = os.path.join(path_in, file)
            info_array_world = np.load(file_path)

            info_pipes_slam_list, info_connexions_slam_list, info_valves_slam_list, info_inst_pipe_slam_list = conversion_utils.array_to_info(info_array_world)

            for i in range(len(info_pipes_slam_list)):                            # update pipe idx
                old_idx = info_pipes_slam_list[i][3]
                new_idx = [x + inst_idx for x in old_idx]
                info_pipes_slam_list[i][3] = new_idx

            inst_idx+=len(info_inst_pipe_slam_list)

            for i in range(len(info_valves_slam_list)):                            # create a list of valve types, so when valver are merged, the final 
                info_valves_slam_list[i].append([info_valves_slam_list[i][2]])    # type is the most common one in this list

            info_slam[0].extend(info_pipes_slam_list)
            info_slam[1].extend(info_connexions_slam_list)
            info_slam[2].extend(info_valves_slam_list)
            info_slam[3].extend(info_inst_pipe_slam_list)

    info_map = get_info_map(info_slam)

    path_out_map = os.path.join(path_out, file_name+"_map.ply")
    conversion_utils.info_to_ply(info_map, path_out_map)

    path_out_map = os.path.join(path_out, file_name+"_map.npy")
    array_slam_map = conversion_utils.info_to_array(info_map)
    np.save(path_out_map, array_slam_map)  

    if count == count_target:
        count = 0
        info_map = clean_map(info_map, count_thr)

        path_out_map_clean = os.path.join(path_out, file_name+"_map_clean.ply")
        conversion_utils.info_to_ply(info_map, path_out_map_clean)





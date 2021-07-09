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

def check_near(arr1, arr2):

    near = False
    start = arr1[0]
    end = arr1[-1]
    
    for i in arr2:
        d_start = get_instances.get_distance(i, start, 3)
        d_end = get_instances.get_distance(i, end, 3)

        if d_start < 0.05 or d_end < 0.05:
            near = True
            break

    return near

def get_info_map(info_map, info_world):

    info_pipes_map_list = info_map[0]
    info_connexions_map_list = info_map[1]
    info_valves_map_list = info_map[2]
    pipe_inst_map_list = info_map[3]

    info_pipes_world_list = info_world[0]
    info_connexions_world_list = info_world[1]
    info_valves_world_list = info_world[2]
    pipe_inst_world_list = info_world[3]


    merge_inst_map_list = list()

    for i, pipe_world in enumerate(info_pipes_world_list):
        merge_list = list()
        for j, pipe_map in enumerate(info_pipes_map_list):
            near = check_near(pipe_map[0], pipe_world[0])
            if near == True:
                merge_list = merge_list + pipe_map[3]  

        merge_inst_map_list.append(merge_list)              # lista de listas de merges

    print(merge_inst_map_list)

    new_insts = list()

    for i, merge_list in enumerate(merge_inst_map_list):
        
        if len(merge_list)>0:
            merge_list = list(set(merge_list))

            # delete pipes map belonging to insts map that are going to be merged and recalculated
            delete_pipe_list = list()
            for j, pipe_map in enumerate(info_pipes_map_list):
                delete = any(inst in merge_list for inst in pipe_map[3])
                if delete == True:
                    delete_pipe_list.append(j)
            for j in sorted(delete_pipe_list, reverse=True): 
                del info_pipes_map_list[j] 

            # delete connexions map belonging to pipes map that belong to instances map that are going to be recalculated
            delete_connexion_list = list()
            for j, connexion_map in enumerate(info_connexions_map_list):
                delete = any(pipe in delete_pipe_list for pipe in connexion_map[1])
                if delete == True:
                    delete_connexion_list.append(j)
            for j in sorted(delete_connexion_list, reverse=True): 
                del info_connexions_map_list[j] 

            new_inst = pipe_inst_world_list[i]
            for j in merge_list:
                new_inst = np.vstack((new_inst, pipe_inst_map_list[j]))
                new_insts.append(new_inst)

                pipe_inst_map_list[j] = new_inst   # TODO para juntar con inst world siguientes, despues se borraran subconjuntos, lo mismo en new_insts

            pipe_inst_map_list.append(new_inst)     # TODO despues se borraran subconjuntos,

    # delete lists that are repeated or a set of another list, due to stacking the concatenated inst into its component inst positions
    inst_del_list = list()
    for i, inst1 in enumerate(pipe_inst_map_list):
        if i not in inst_del_list:
            for j , inst2 in enumerate(pipe_inst_map_list):
                if j not in inst_del_list:
                    if i != j:
                        inst_test = np.vstack((inst1,inst2))             # stack inst
                        inst_test_u = np.unique(inst_test, axis=0)        
                        if inst1.shape[0] == inst_test_u.shape[0]:        # if second inst does not add new links, is same inst or a set
                            inst_del_list.append(j)                        # mark inst to be deleted

    for i in sorted(inst_del_list, reverse=True):      # delete marked inst
        del pipe_inst_map_list[i]

    # delete lists that are repeated or a set of another list, due to stacking the concatenated inst into its component inst positions
    inst_del_list = list()
    for i, inst1 in enumerate(new_insts):
        if i not in inst_del_list:
            for j , inst2 in enumerate(new_insts):
                if j not in inst_del_list:
                    if i != j:
                        inst_test = np.vstack((inst1,inst2))             # stack inst
                        inst_test_u = np.unique(inst_test, axis=0)        
                        if inst1.shape[0] == inst_test_u.shape[0]:        # if second inst does not add new links, is same inst or a set
                            inst_del_list.append(j)                       # mark inst to be deleted

    for i in sorted(inst_del_list, reverse=True):      # delete marked inst
        del new_insts[i]


    new_info_pipes_map_list = list()
    new_info_connexions_map_list = list()
    k_pipe = 0

    for i, inst_map in enumerate(new_insts): # for each pipe instance
        
        # transform instance to o3d pointcloud
        inst_map_o3d = o3d.geometry.PointCloud()
        inst_map_o3d.points = o3d.utility.Vector3dVector(inst_map[:,0:3])

        info_pipe_map = get_info.get_info(inst_map_o3d, models=0, method="skeleton") # get pipe instance info list( list( list(chain1, start1, end1, elbow_list1, vector_chain_list1), ...), list(connexions_points)) 
        
        for j, pipe_info in enumerate(info_pipe_map[0]):                         # stack pipes info
            inst_list = list()
            inst_list.append(i)
            pipe_info.append(inst_list)
            new_info_pipes_map_list.append(pipe_info)

        for j, connexion_info in enumerate(info_pipe_map[1]):                    # stack conenexions info
            connexion_info[1] = [x+k_pipe for x in connexion_info[1]]
            new_info_connexions_map_list.append(connexion_info)

        k_pipe += len(info_pipe_map[0])                                          # update actual pipe idx


    new_info_pipes_map_list_copy = copy.deepcopy(new_info_pipes_map_list) 
    new_info_connexions_map_list_copy = copy.deepcopy(new_info_connexions_map_list)
    new_info_pipes_map_list, new_info_connexions_map_list = get_info.unify_chains(new_info_pipes_map_list_copy, new_info_connexions_map_list_copy)



    info_pipes_map_list = info_pipes_map_list + new_info_pipes_map_list
    info_connexions_map_list = info_connexions_map_list + new_info_connexions_map_list


    for i, pipe_world in enumerate(info_pipes_world_list): # SE HACE DESPUES APRA EVITAR QUE PIPE WORLD SE CHECK NEAR VS ANTERIORES PIPE WORLS ADDED TO info_pipes_map_list

        if len(merge_inst_map_list[i]) == 0:


            for j, info_connexion_world in enumerate(info_connexions_world_list):
                if i in info_connexion_world[1]:
                    if info_connexion_world not in info_connexions_map_list:
                        info_connexions_map_list.append(info_connexion_world)  # TODO actializar near pipes - AL FINAL CALCULAR NUEVOS NEAR DE CONEXIONES Y VALVES

            insts_map = len(pipe_inst_map_list)

            for j in pipe_world[3]:
                pipe_inst_map_list.append(pipe_inst_world_list[j])

            for j, inst_idx in enumerate(pipe_world[3]):
                pipe_world[3][j] = inst_idx + insts_map
            info_pipes_map_list.append(pipe_world) 

    
    info_map = [info_pipes_map_list, info_connexions_map_list, info_valves_map_list, pipe_inst_map_list]

    return info_map



def clean_map(info_map, count_thr):

    info_pipes_map_list = info_map[0]
    info_connexions_map_list = info_map[1]
    info_valves_map_list = info_map[2]
    pipe_inst_map_list = info_map[3]


    # for instance i
        # if count i < count_thr
            # mark instance i to delete
    # delete instances
    # ESTO SE PUEDE HACER POR DENSIDAD???


    # get_info instances pipe
    # RECALCULATE BELONGING INST OF PIPES (old ones and newly gotten)
    # RECALCULATE NEAR PIPES OF CONNEXIONS (old ones and newly gotten)

    #for valve
        #if count < count_thr
            #delete valve
    # if list deleted instances len > 0
        # RECALCULATE NEAR PIPES OF VALVES (old ones and newly gotten)

    info_map = [info_pipes_map_list, info_connexions_map_list, info_valves_map_list, pipe_inst_map_list]

    return info_map


    ############# OPCION 2 #############
    # for pipe p
        # if all belonging instances are on list of instances deleted
            # mark pipe to delete
    # delete pipes

    # RECALCULATE BELONGING INST OF PIPES (old ones and newly gotten)

    #for connexion c
        # delete near pipes that are in pipes deleted
    # refine connexions?
    ####################################



if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--path_in', help='path in info.')
    parser.add_argument('--path_out', help='path out info.')
    parsed_args = parser.parse_args(sys.argv[1:])

    path_in = parsed_args.path_in
    path_out = parsed_args.path_out

    info_pipes_list_map = list()
    info_connexions_list_map = list()
    info_valves_list_map = list()
    instances_ref_pipe_list_map = list()
    info_map = [info_pipes_list_map, info_connexions_list_map, info_valves_list_map, instances_ref_pipe_list_map]

    count = 0
    count_target = 5
    count_thr = 1

    for file in natsorted(os.listdir(path_in)):

        print("working on: " + file)

        file_name, _ = os.path.splitext(file)
        count += 1

        file_path = os.path.join(path_in, file)
        info_array_world = np.load(file_path)

        info_pipes_world_list, info_connexions_world_list, info_valves_world_list, info_inst_pipe_world_list = conversion_utils.array_to_info(info_array_world)
        info_world = [info_pipes_world_list, info_connexions_world_list, info_valves_world_list, info_inst_pipe_world_list]

        info_map = get_info_map(info_map, info_world)

        path_out_map = os.path.join(path_out, file_name+"_map.ply")
        conversion_utils.info_to_ply(info_map, path_out_map)

        # if count == count_target:
        #     count = 0
        #     info_map = clean_map(info_map, count_thr)

        #     path_out_map_clean = os.path.join(path_out, file_name+"_map_clean.ply")
        #     conversion_utils.info_to_ply(info_map, path_out_map_clean)

        




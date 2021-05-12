import os
import re
import sys
import time
import copy
import math
import argparse
import numpy as np
import scipy as scp
import open3d as o3d
import conversion_utils
from natsort import natsorted
import matplotlib.pyplot as plt
from scipy.spatial import distance
from mpl_toolkits.mplot3d import Axes3D



def get_info_map(info_map, info_world):

    info_pipes_list_map = info_map[0]
    info_connexions_list_map = info_map[1]
    info_valves_list_map = info_map[2]
    pipe_inst_list_map = info_map[3]

    info_pipes_list_world = info_world[0]
    info_connexions_list_world = info_world[1]
    info_valves_list_world = info_world[2]
    pipe_inst_list_world = info_world[3]


    # for pipe in world, check if skeleton is near a skeleton in pipe map

        # if it is
            # anotate all its instances to be merged, world and map
        # if it is not
            # add its pipe, connexions (if not already in, to not put it three times if T completely new) and instances to world

    # if len(list_instances_merge)>0
        # set of instances to be merged

        # delete pipes belonging to these instances 
        # delete connexions belonging to these pipes

        # merge instances
        # apply     128-0.1     64-0.05     32-0.025

        # get new pipes and connexions info

        # sort pipe inst belonging to number
        # sort connexion  near pipe number

    # sort count for insts just additions, just deletions, additions and deletions

    # for valve world check it its near valve map
        # if it is
            # merge
            # sort valve near pipe number
            # sort count for inst
        # if it is not
            # new valve
            # count = 0





    
    info_map = [info_pipes_list_map, info_connexions_list_map, info_valves_list_map, instances_ref_pipe_list_map]

    return info_map



def clean_map(info_map, count_thr):

    info_pipes_list_map = info_map[0]
    info_connexions_list_map = info_map[1]
    info_valves_list_map = info_map[2]
    pipe_inst_list_map = info_map[3]


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

    info_map = [info_pipes_list_map, info_connexions_list_map, info_valves_list_map, instances_ref_pipe_list_map]

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

    for file in listdir(path_in):

        file_name, _ = os.path.splitext(file)
        count += 1

        file_path = os.path.join(path_in, file)
        info_array_world = np.load(file_path)

        info_pipes_list, info_connexions_list, info_valves_list, info_inst_pipe_list = conversion_utils.array_to_info(info_array_world)
        info_world = [info_pipes_list, info_connexions_list, info_valves_list, info_inst_pipe_list]

        info_map = map_utils.get_info_map(info_map, info_world)

        path_out_map = os.path.join(path_out, file_name+"_map.ply")
        conversion_utils.info_to_ply(info_map, path_out_map)

        if count == count_target:
            count = 0
            info_map = map_utils.clean_map(info_map, count_thr)

            path_out_map_clean = os.path.join(path_out, file_name+"_map_clean.ply")
            conversion_utils.info_to_ply(info_map, path_out_map_clean)

        




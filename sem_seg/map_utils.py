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



    return info_map



def clean_map(info_map, count_thr):


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
    count_thr = 2

    for file in listdir(path_in):

        count += 1

        file_path = os.path.join(path_in, file)
        info_array = np.load(file_path)

        info_pipes_list, info_valves_list, info_connexions_list, info_inst_pipe_list = conversion_utils.array_to_info(info_array_world)
        info_world = [info_pipes_list, info_valves_list, info_connexions_list, info_inst_pipe_list]

        
        info_map = map_utils.get_info_map(info_map, info_world)
        # TODO PASAR A PLY Y GUARDAR PLY CON NOMBRE QUE HA ENTRADO PERO MAP

        if count == count_target:
            count = 0
            info_map = map_utils.clean_map(info_map, count_thr)
            # TODO PASAR A PLY Y GUARDAR PLY CON NOMBRE QUE HA ENTRADO PERO MAP CLEAN

        




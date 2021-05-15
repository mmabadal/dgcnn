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


def get_instances_o3d_pipes(data, dim, rad, min_points):

    data_copy = data.copy()
    if dim == 2:
        data_copy[...,2] = 0

    data_o3d = o3d.geometry.PointCloud()
    data_o3d.points = o3d.utility.Vector3dVector(data_copy[:,0:3])

    labels = np.array(data_o3d.cluster_dbscan(eps=rad, min_points=10, print_progress=False))
    data = data[labels != -1]

    instances = list()
    for i in set(labels):
        inst = data[np.where(labels == float(i))]
        if inst.shape[0]>min_points:
            instances.append(inst)

    if len(instances)== 0:
        print("NO INSTANCES FOUND")
    return instances


def get_instances_pipes(data, dim, rad, min_points):        # //PARAM

    n_inst = 1
    stolen_list = list()

    # get instances 
    instances = list()
    while data.size > 0:                                        # while there are  points
        actual_idx = [0]                                              # init idx
        actual_inst = data[actual_idx]                          # init actual inst
        inst = actual_inst                                            # init inst
        data = np.delete(data, actual_idx, axis=0)        # delete  points
        while actual_idx:                                             # while idx exists
            actual_idx = grow_pipes(data, actual_inst, rad, dim)      # idx grow
            actual_inst = data[actual_idx]                      # get new actual inst
            inst = np.vstack([inst, actual_inst])                     # append to inst
            data = np.delete(data, actual_idx, axis=0)    # delete  points
        if inst.shape[0] > min_points:                                # save instance if  bigger than min
            instances.append(inst)
    return instances


def grow_pipes(data, points, min_dist, dim):          # //PARAM

    new_idx = list()
    for n, p in enumerate(points):              # for each point to grow from

        #progress(n, len(points), status='growing')

        p1 = p[0:3]
        for j, point2 in enumerate(data):       # for each point of data to grow over
            p2 = data[j,0:3]

            d = get_instances.get_distance(p1,p2,dim)     # get distance
            if d < min_dist:                # if dist < thr 
                new_idx.append(j)           # add idx

    new_idx = list(set(new_idx))

    return new_idx


def data2blocks(data, num_sample, block_size=0.1, stride=0.1):
    """ Prepare block training data.
    Args:
        data: N x 6 numpy array, 012 are XYZ in meters, 345 are RGB in [0,1]
            assumes the data is shifted (min point is origin) and aligned
            (aligned with XYZ axis)
        num_point: int, how many points to sample in each block
        block_size: float, physical size of the block in meters
        stride: float, stride for block sweeping
    Returns:
        block_datas: K x num_point x 6 np array of XYZRGB, RGB is in [0,1]
    """
    assert(stride<=block_size)

    limit = np.amax(data, 0)[0:3]

    # Get the corner location for our sampling blocks    
    xbeg_list = []
    ybeg_list = []

    num_block_x = int(np.ceil((limit[0] - block_size) / stride)) + 1
    num_block_y = int(np.ceil((limit[1] - block_size) / stride)) + 1

    for i in range(num_block_x):
        for j in range(num_block_y):
            xbeg_list.append(i*stride)
            ybeg_list.append(j*stride)

    # Collect blocks
    block_data_list = []
    block_label_list = []
    idx = 0
    for idx in range(len(xbeg_list)): 

        xbeg = xbeg_list[idx]
        ybeg = ybeg_list[idx]
        xcond = (data[:,0]<=xbeg+block_size) & (data[:,0]>=xbeg)
        ycond = (data[:,1]<=ybeg+block_size) & (data[:,1]>=ybeg)
        cond = xcond & ycond
        if np.sum(cond) < 0: #  lessthan
            print("DISCARTED BLOCK")
            continue

        block_data = data[cond, :]

        # randomly subsample data
        block_data_sampled, sample_indices = sample_data(block_data, num_sample)

        if block_data_sampled.size == 0:
            continue
        block_data_list.append(np.expand_dims(block_data_sampled, 0))

    if len(block_data_list) > 0:
        data_batch = np.concatenate(block_data_list, 0)
        return data_batch
    else:
        return np.array([]), np.array([])


def sample_data(data, num_sample):
    """ data is in N x ...
        we want to keep num_samplexC of them.
        if N > num_sample, we will randomly keep num_sample of them.
        if N < num_sample, we will randomly duplicate samples.
    """
    empty = list()
    N = data.shape[0]
    if (N == 0):
        return data, empty
    elif (N == num_sample):
        return data, range(N)
    elif (N > num_sample):
        sample = np.random.choice(N, num_sample)
        return data[sample, ...], sample
    else:
        sample = np.random.choice(N, num_sample-N)
        dup_data = data[sample, ...]
        return np.concatenate([data, dup_data], 0), list(range(N))+list(sample)


def get_info_map_valve(info_valves_map_list, info_valves_world_list):

    for i, info_valve_world in enumerate(info_valves_world_list):
        merged = False

        for j, info_valve_map in enumerate(info_valves_map_list):
            dist = get_instances.get_distance(info_valve_world[0], info_valve_map[0], 2) 
            if dist < 0.15: # las valvulas tienen una longitud de 0.18
                info_valves_map_list[j][0] = (info_valves_map_list[j][0] + info_valve_world[0])/2
                info_valves_map_list[j][4] = info_valves_map_list[j][4]+1 # count 1
                merged = True
                break
        
        if merged == False:
            count_v = 1
            info_valve_world.append(count_v)
            info_valves_map_list.append(info_valve_world)

    return info_valves_map_list


def clean_map(info_map, count_thr):

    info_pipes_map_list = info_map[0]
    info_connexions_map_list = info_map[1]
    info_valves_map_list = info_map[2]
    pipe_inst_map_list = info_map[3]

    valve_del_list = list()

    for i, valve_info_map in enumerate(info_valves_map_list):        # for each valve

        if valve_info_map[4] <= count_thr:
            valve_del_list.append(i)

    for i in sorted(valve_del_list, reverse=True):  # delete chains
        del info_valves_map_list[i]  

    info_map = [info_pipes_map_list, info_connexions_map_list, info_valves_map_list, pipe_inst_map_list]

    return info_map


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--path_in', help='path in info.')
    parser.add_argument('--path_out', help='path out info.')
    parsed_args = parser.parse_args(sys.argv[1:])

    path_in = parsed_args.path_in
    path_out = parsed_args.path_out

    info_pipes_map_list = list()
    info_connexions_map_list = list()
    info_valves_map_list = list()
    instances_ref_pipe_map_list = list()
    info_map = [info_pipes_map_list, info_connexions_map_list, info_valves_map_list, instances_ref_pipe_map_list]

    count = 0
    count_target = 5
    count_thr = 1

    pred_pipe_map_list = list()

    for file in natsorted(os.listdir(path_in)):

        print("working on: " + file)

        file_name, _ = os.path.splitext(file)
        count += 1

        file_path = os.path.join(path_in, file)
        info_array_world = np.load(file_path)

        info_pipes_world_list, info_connexions_world_list, info_valves_world_list, info_inst_pipe_world_list = conversion_utils.array_to_info(info_array_world)
        info_world = [info_pipes_world_list, info_connexions_world_list, info_valves_world_list, info_inst_pipe_world_list]
        
        pred_pipe_map_list = pred_pipe_map_list + info_inst_pipe_world_list

        info_valves_map_list = get_info_map_valve(info_valves_map_list, info_valves_world_list)


        if count == count_target:
            count = 0
            info_map = clean_map(info_map, count_thr)

            path_out_map_clean = os.path.join(path_out, file_name+"_map_clean.ply")
            conversion_utils.info_to_ply(info_map, path_out_map_clean)

    
    pred_pipe_map = np.vstack(pred_pipe_map_list)

    inst_o3d = o3d.geometry.PointCloud()
    inst_o3d.points = o3d.utility.Vector3dVector(pred_pipe_map[:,0:3])
    get_info.print_o3d(inst_o3d)

    print(pred_pipe_map.shape)

    xyz_min = np.amin(pred_pipe_map, axis=0)[0:3]   # get pointcloud mins
    pred_pipe_map[:, 0:3] -= xyz_min                # move pointcloud to origin


    pred_pipe_map_sub = data2blocks(pred_pipe_map, 128, block_size=0.1, stride=0.1) # subsample PC

    pred_pipe_map_sub = pred_pipe_map_sub.reshape((pred_pipe_map_sub.shape[0]*pred_pipe_map_sub.shape[1]), pred_pipe_map_sub.shape[2])

    pred_pipe_map_sub = np.unique(pred_pipe_map_sub, axis=0)  # delete duplicates

    pred_pipe_map_sub[:, 0:3] += xyz_min                # move pointcloud to original position

    rad_p = 0.04               # max distance for pipe growing                             //PARAM
    dim_p = 3                  # compute 2D (2) or 3D (3) distance for pipe growing        //PARAM
    min_p_p = 60               # minimum number of points to consider a blob as a pipe     //PARAM

    #pipe_inst_map_list  = get_instances_pipes(pred_pipe_map_sub, dim_p, rad_p, min_p_p)
    pipe_inst_map_list  = get_instances_o3d_pipes(pred_pipe_map_sub, dim_p, rad_p, min_p_p)

    info_pipes_map_list = list()
    info_connexions_map_list = list()
    k_pipe = 0

    for i, inst_map in enumerate(pipe_inst_map_list): # for each pipe instance
        
        # transform instance to o3d pointcloud
        inst_map_o3d = o3d.geometry.PointCloud()
        inst_map_o3d.points = o3d.utility.Vector3dVector(inst_map[:,0:3])

        info_pipe_map = get_info.get_info(inst_map_o3d, models=0, method="skeleton") # get pipe instance info list( list( list(chain1, start1, end1, elbow_list1, vector_chain_list1), ...), list(connexions_points)) 
        
        for j, pipe_info in enumerate(info_pipe_map[0]):                         # stack pipes info
            inst_list = list()
            inst_list.append(i)
            pipe_info.append(inst_list)
            info_pipes_map_list.append(pipe_info)

        for j, connexion_info in enumerate(info_pipe_map[1]):                    # stack conenexions info
            connexion_info[1] = [x+k_pipe for x in connexion_info[1]]
            info_connexions_map_list.append(connexion_info)

        k_pipe += len(info_pipe_map[0])                                          # update actual pipe idx

    info_pipes_map_list_copy = copy.deepcopy(info_pipes_map_list) 
    info_connexions_map_list_copy = copy.deepcopy(info_connexions_map_list)
    info_pipes_map_list, info_connexions_map_list = get_info.unify_chains(info_pipes_map_list_copy, info_connexions_map_list_copy) 

    
    for i, valve_info_map in enumerate(info_valves_map_list):        # for each valve

        near_pipes_list = list()
        near_type_list = list()
        for j, info_pipe_map in enumerate(info_pipes_map_list):                         # get near pipes
            c_p = info_valves_map_list[i][0]                                        # central point
            d_to_start = get_instances.get_distance(c_p, info_pipe_map[0][0], 3)      # get distance from valve central point to pipe start
            d_to_end = get_instances.get_distance(c_p, info_pipe_map[0][-1], 3)       # get distance from valve central point to pipe end
            if d_to_start <= 0.25:                                                   # if distance < thr             //PARAM
                near_pipes_list.append(j)                                           # append pipe as near
                near_type_list.append(0)                                            # append start is near
                break  
            if d_to_end <= 0.25:                                                     # if distance < thr             //PARAM
                near_pipes_list.append(j)                                           # append pipe as near
                near_type_list.append(-1)                                           # append end is near
                break 

        info_valves_map_list[i][3] = near_pipes_list                                    # replace near pipes to valve info [central_point, vector, max_id, near_pipes]

    info_map = [info_pipes_map_list, info_connexions_map_list, info_valves_map_list, pipe_inst_map_list]

    path_out_map = os.path.join(path_out, file_name+"_map_final.ply")
    conversion_utils.info_to_ply(info_map, path_out_map)
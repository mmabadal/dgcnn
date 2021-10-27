import os
import re
import sys
import time
import copy
import argparse
import get_info
import numpy as np
from model import *
import open3d as o3d
import indoor3d_util
import get_instances
import conversion_utils
from natsort import natsorted

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(BASE_DIR)

"python inference_online_all.py --path_data a/b/c --path_cls meta/class_or.txt --model_path RUNS/test_indoor --points_sub 128 --points_proj 512 --test_name test_name"

parser = argparse.ArgumentParser()
parser.add_argument('--path_data', help='folder with train test data')
parser.add_argument('--path_cls', help='path to classes txt.')
parser.add_argument('--dump_path', required=True, help='dump_path')
parser.add_argument('--points_sub', type=int, default=128, help='Point number sub [default: 4096]')
parser.add_argument('--targets_path', help='path in valve models.')

parsed_args = parser.parse_args()

path_data = parsed_args.path_data
path_cls = parsed_args.path_cls
dump_path = parsed_args.dump_path
points_sub = parsed_args.points_sub
targets_path = parsed_args.targets_path
if not os.path.exists(dump_path): os.mkdir(dump_path)

classes, labels, label2color = indoor3d_util.get_info_classes(path_cls)
num_classes = len(classes)

batch_size = 1
gpu_index = 0
block_sub = 0.1
stride_sub = 0.1


if __name__=='__main__':

    # get valve matching targets
    targets_list = list()
    for file_name in natsorted(os.listdir(targets_path)):
        target_path = os.path.join(targets_path, file_name)
        target = get_info.read_ply(target_path, "model")
        xyz_central = np.mean(target, axis=0)[0:3]
        target[:, 0:3] -= xyz_central  
        target_o3d = o3d.geometry.PointCloud()
        target_o3d.points = o3d.utility.Vector3dVector(target[:,0:3])
        target_o3d.colors = o3d.utility.Vector3dVector(target[:,3:6])
        target_o3d.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=15))
        target_o3d.orient_normals_to_align_with_direction(orientation_reference=([0, 0, 1]))
        targets_list.append(target_o3d)

    # LOOP
    #while 1:

    for root, dirs, files in os.walk(path_data):        # for each folder

        for file in enumerate(sorted(files)):           # for each file           

            if re.search("\.(txt)$", file[1]):          # if its a txt       
                print("working on: " + str(file[1]))
                
                filepath = os.path.join(root, file[1])
                pred_sub = np.loadtxt(filepath)  # read from txt (not on xyz origin)

                # downsample prediction to 128 if its not already on 128
                if points_sub >= 128:                                             # //PARAM
                    down = 128/points_sub                                         # //PARAM
                    n_idx_pred_sub_down = int(pred_sub.shape[0] * down)  
                    idx_pred_sub_down = np.random.choice(pred_sub.shape[0], n_idx_pred_sub_down, replace=False)
                    pred_sub = pred_sub[idx_pred_sub_down, 0:7] 

                col_inst = {
                0: [255, 255, 0],
                1: [255, 0, 255],
                2: [0, 255, 255],
                3: [0, 128, 0],
                4: [0, 0, 128],
                5: [128, 0, 0],
                6: [0, 255, 0],
                7: [0, 0, 255],
                8: [255, 0, 0],
                9: [0, 100, 0],
                10: [0, 0, 100],
                11: [100, 0, 0],
                12: [100, 0, 255],
                13: [0, 255, 100],
                13: [255, 100, 0]
                }

                # init get_instances parameters
                rad_p = 0.04               # max distance for pipe growing                             //PARAM
                rad_v = 0.04               # max distance for valve growing                            //PARAM
                dim_p = 3                  # compute 2D (2) or 3D (3) distance for pipe growing        //PARAM
                dim_v = 2                  # compute 2D (2) or 3D (3) distance for valve growing       //PARAM
                min_p_p = 50               # minimum number of points to consider a blob as a pipe     //PARAM
                min_p_v = 30 # 40 80 140   # minimum number of points to consider a blob as a valve    //PARAM

                pred_sub = np.c_[ pred_sub, np.zeros(pred_sub.shape[0]) ]
                for i in range(pred_sub.shape[0]):
                    if int(pred_sub[i,4]) == 255:
                        pred_sub[i,6] = 0.0
                    elif int(pred_sub[i,5]) == 255:
                        pred_sub[i,6] = 1.0
                    else:
                        pred_sub[i,6] = 2.0
            
                pred_sub_pipe = pred_sub[pred_sub[:,6] == [labels["pipe"]]]       # get points predicted as pipe
                pred_sub_valve = pred_sub[pred_sub[:,6] == [labels["valve"]]]     # get points predicted as valve

                # get valve instances
                instances_ref_valve_list, pred_sub_pipe_ref, stolen_list  = get_instances.get_instances(pred_sub_valve, dim_v, rad_v, min_p_v, ref=True, ref_data = pred_sub_pipe, ref_rad = 0.1)
                #instances_ref_valve_list, pred_sub_pipe_ref, stolen_list  = get_instances.get_instances_o3d(pred_sub_valve, dim_v, rad_v, min_p_v, ref=True, ref_data = pred_sub_pipe, ref_rad = 0.1)

                # get valve information
                info_valves_list = list()
                for i, inst in enumerate(instances_ref_valve_list): # for each valve instance
                    # transform instance to o3d pointcloud
                    xyz_central = np.mean(inst, axis=0)[0:3]    # get isntance center
                    inst[:, 0:3] -= xyz_central                 # move center to origin
                    inst_o3d = o3d.geometry.PointCloud()
                    inst_o3d.points = o3d.utility.Vector3dVector(inst[:,0:3])
                    inst_o3d.colors = o3d.utility.Vector3dVector(inst[:,3:6])
                    inst_o3d.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=15)) # compute normal
                    inst_o3d.orient_normals_to_align_with_direction(orientation_reference=([0, 0, 1]))                   # align normals
                    inst[:, 0:3] += xyz_central                                                                          # recover original position

                    info_valve = get_info.get_info(inst_o3d, targets_list, method="matching")        # get valve instance info list([fitness1, rotation1],[fitness2, rotation2], ...)  len = len(targets_list)
                    max_info =  max(info_valve)                                                      # the max() function compares the first element of each info_list element, which is fitness)
                    max_idx = info_valve.index(max_info)                                             # idx of best valve match
                    
                    rad = math.radians(max_info[1])
                    vector = np.array([math.cos(rad), math.sin(rad), 0])                             # get valve unit vector at zero
                    vector = vector*0.18                                                             # resize vector to valve size //PARAM
                    info_valves_list.append([xyz_central, vector, max_idx, inst[:,0:3], max_info])   # append valve instance info


                # based on valve fitness, delete it and return stolen points to pipe prediction
                descart_valves_list = [i for i, x in enumerate(info_valves_list) if x[4][0] < 0.4]     # if max fitnes < thr  //PARAM
                for i in descart_valves_list:
                    print("Valve descarted")
                    descarted_points = np.vstack(instances_ref_valve_list[i])                           # notate points to discard
                    if len(stolen_list)>0:                                                                  # if there were stolen points
                        stolen_idx = list(np.vstack(stolen_list[i])[:,0].astype(int))                       # get stolen idx
                        stolen_cls = np.vstack(stolen_list[i])[:,1].astype(int)                             # get stolen class
                        stolen_cls = stolen_cls.reshape(stolen_cls.shape[0],1)                              # reshape stolen class
                        stolen_points = descarted_points[stolen_idx, :-2]                               # recover stolen points
                        stolen_points = np.concatenate((stolen_points,stolen_cls),axis=1)               # concatenate stolen points and stolen class
                        pred_sub_pipe_ref = np.concatenate((pred_sub_pipe_ref,stolen_points),axis=0)    # add points and class pipe prediction points
                
                for index in sorted(descart_valves_list, reverse=True):                                 # delete discarted valve info                                                                             
                    del info_valves_list[index]
                    del instances_ref_valve_list[index]     # for print only  

                # get pipe instances
                instances_ref_pipe_list, _, _  = get_instances.get_instances(pred_sub_pipe_ref, dim_p, rad_p, min_p_p)
                #instances_ref_pipe_list, _, _  = get_instances.get_instances_o3d(pred_sub_pipe_ref, dim_p, rad_p, min_p_p)

                info_pipes_list = list()
                info_connexions_list = list()
                k_pipe = 0

                for i, inst in enumerate(instances_ref_pipe_list): # for each pipe instance
                    # transform instance to o3d pointcloud
                    inst_o3d = o3d.geometry.PointCloud()
                    inst_o3d.points = o3d.utility.Vector3dVector(inst[:,0:3])
                    inst_o3d.colors = o3d.utility.Vector3dVector(inst[:,3:6]/255)

                    info_pipe = get_info.get_info(inst_o3d, models=0, method="skeleton") # get pipe instance info list( list( list(chain1, start1, end1, elbow_list1, vector_chain_list1), ...), list(connexions_points)) 
                    
                    for j, pipe_info in enumerate(info_pipe[0]):                         # stack pipes info
                        inst_list = list()
                        inst_list.append(i)
                        pipe_info.append(inst_list)
                        info_pipes_list.append(pipe_info)

                    for j, connexion_info in enumerate(info_pipe[1]):                    # stack conenexions info
                        connexion_info[1] = [x+k_pipe for x in connexion_info[1]]
                        info_connexions_list.append(connexion_info)

                    k_pipe += len(info_pipe[0])                                          # update actual pipe idx

                info_pipes_list_copy = copy.deepcopy(info_pipes_list) 
                info_connexions_list_copy = copy.deepcopy(info_connexions_list)
                info_pipes_list2, info_connexions_list2 = get_info.unify_chains(info_pipes_list_copy, info_connexions_list_copy)

                info_valves_list_copy = copy.deepcopy(info_valves_list)
                info_valves_list2 = get_info.refine_valves(info_valves_list_copy, info_pipes_list2) 

                info1 = [info_pipes_list, info_connexions_list, info_valves_list, instances_ref_pipe_list]
                #info2 = [info_pipes_list2, info_connexions_list2, info_valves_list, instances_ref_pipe_list] 
                info3 = [info_pipes_list2, info_connexions_list2, info_valves_list2, instances_ref_pipe_list]

                empty1 = False
                if len(info_pipes_list)==0 and len(info_connexions_list)==0 and len(info_valves_list)==0 and len(instances_ref_pipe_list)==0:
                    empty1 = True
                
                empty3 = False
                if len(info_pipes_list2)==0 and len(info_connexions_list2)==0 and len(info_valves_list2)==0 and len(instances_ref_pipe_list)==0:
                    empty3 = True


                if empty1==False:
                    path_out1 = os.path.join(dump_path, os.path.basename(filepath)[:-4]+'_info.ply')
                    conversion_utils.info_to_ply(info1, path_out1)
                    
                #path_out2 = os.path.join(dump_path, os.path.basename(filepath)[:-4]+'_info2.ply')
                #conversion_utils.info_to_ply(info2, path_out2)

                if empty3==False:
                    path_out3 = os.path.join(dump_path, os.path.basename(filepath)[:-4]+'_info_ref.ply')
                    conversion_utils.info_to_ply(info3, path_out3)

                    info3_array = conversion_utils.info_to_array(info3)
                    path_out3_array = os.path.join(dump_path, os.path.basename(filepath)[:-4]+'_info_ref_array.npy')
                    np.save(path_out3_array, info3_array)

                    fout_inst = open(os.path.join(dump_path, os.path.basename(filepath)[:-4]+'_info_ref.txt'), 'w')
                    fout_inst.write('    x         y         z   type  info class inst    \n')
                    for i in range(info3_array.shape[0]):
                        if info3_array[i,6]!= 0 and info3_array[i,6]!= 4 and info3_array[i,6]!= 6 and info3_array[i,6]!= 7:
                            fout_inst.write('%f %f %f  %d     %d     %d     %d\n' % (info3_array[i,0], info3_array[i,1], info3_array[i,2], info3_array[i,6], info3_array[i,7], info3_array[i,8], info3_array[i,9]))
                # print info

                print(" ")
                print("INFO VALVES:")
                for valve in info_valves_list:
                    valve.pop(-2)
                    print(valve)
                print(" ")

                print("INFO VALVES2:")
                for valve in info_valves_list2:
                    valve.pop(-2)
                    print(valve)
                print(" ")

                print("INFO PIPES:")
                for pipe1 in info_pipes_list:
                    pipe1.pop(0)
                    print(pipe1)
                print(" ")

                print("INFO PIPES2")
                for pipe2 in info_pipes_list2:
                    pipe2.pop(0)
                    print(pipe2)
                print(" ")

                print("INFO CONNEXIONS:")
                for connexion in info_connexions_list:
                    print(connexion)
                print(" ")

                print("INFO CONNEXIONS2:")
                for connexion in info_connexions_list2:
                    print(connexion)
                print(" ")

                # PRINTS

                i = len(instances_ref_valve_list)

                if len(instances_ref_valve_list)>0:
                    instances_ref_valve = np.vstack(instances_ref_valve_list)
                if len(instances_ref_pipe_list)>0:
                    instances_ref_pipe = np.vstack(instances_ref_pipe_list)
                    instances_ref_pipe[:,7] = instances_ref_pipe[:,7]+i

                if len(instances_ref_valve_list)>0 and len(instances_ref_pipe_list)>0:
                    instances_ref = np.concatenate((instances_ref_valve, instances_ref_pipe), axis=0)
                elif len(instances_ref_valve_list)==0 and len(instances_ref_pipe_list)>0:
                    instances_ref = instances_ref_pipe
                elif len(instances_ref_valve_list)>0 and len(instances_ref_pipe_list)==0:
                    instances_ref = instances_ref_valve
                else:
                    instances_ref = None

                fout_sub = open(os.path.join(dump_path, os.path.basename(filepath)[:-4]+'_pred_sub.obj'), 'w')
                fout_sub_col = open(os.path.join(dump_path, os.path.basename(filepath)[:-4]+'_pred_sub_col.obj'), 'w')
                for i in range(pred_sub.shape[0]):
                    fout_sub.write('v %f %f %f %d %d %d %d\n' % (pred_sub[i,0], pred_sub[i,1], pred_sub[i,2], pred_sub[i,3], pred_sub[i,4], pred_sub[i,5], pred_sub[i,6]))
                for i in range(pred_sub.shape[0]):
                    color = label2color[pred_sub[i,6]]
                    fout_sub_col.write('v %f %f %f %d %d %d %d\n' % (pred_sub[i,0], pred_sub[i,1], pred_sub[i,2], color[0], color[1], color[2], pred_sub[i,6]))

                if instances_ref is not None: # if instances were found
                    fout_inst = open(os.path.join(dump_path, os.path.basename(filepath)[:-4]+'_pred_inst_ref.obj'), 'w')
                    fout_inst_col = open(os.path.join(dump_path, os.path.basename(filepath)[:-4]+'_pred_inst_ref_col.obj'), 'w')
                    for i in range(instances_ref.shape[0]):
                        fout_inst.write('v %f %f %f %d %d %d %d %d\n' % (instances_ref[i,0], instances_ref[i,1], instances_ref[i,2], instances_ref[i,3], instances_ref[i,4], instances_ref[i,5], instances_ref[i,6], instances_ref[i,7]))
                    for i in range(instances_ref.shape[0]):
                        color = col_inst[instances_ref[i,7]]
                        fout_inst_col.write('v %f %f %f %d %d %d %d %d\n' % (instances_ref[i,0], instances_ref[i,1], instances_ref[i,2], color[0], color[1], color[2], instances_ref[i,6], instances_ref[i,7]))

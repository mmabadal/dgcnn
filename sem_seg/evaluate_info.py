import os
import re
import sys
import math
import argparse
import numpy as np
from natsort import natsorted


def get_distance(p1, p2, dim):
    if dim == 2:
        d = math.sqrt(((p2[0]-p1[0])**2)+((p2[1]-p1[1])**2))
    if dim == 3:
        d = math.sqrt(((p2[0]-p1[0])**2)+((p2[1]-p1[1])**2)+((p2[2]-p1[2])**2))
    return d


def angle_between_vectors(v1, v2):

    v1_u = v1/np.linalg.norm(v1)
    v2_u = v2/np.linalg.norm(v2)
    angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    return np.degrees(angle)


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--path_data', help='path to the data.')
    parsed_args = parser.parse_args(sys.argv[1:])
    path_data = parsed_args.path_data

    elbow_point_list = list()

    pipe_vector_size_list = list()
    pipe_vector_direction_list = list()

    conn_central_point_list = list()

    valve_central_point_list = list()
    valve_vector_direction_list = list()
    valve_type_list = list()

    gt3_iea2 = 0
    gt2_iea3 = 0

    path_info_gt = os.path.join(path_data, "gt_ply", "info_juntos_replace")
    path_info_iea = os.path.join(path_data, "out2_con_info", "info_juntos_replace")

    files_gt = natsorted(os.listdir(path_info_gt))

    for file_gt in files_gt:

        name_base, _ = file_gt.split('_')
        print("working on " + str(name_base))

        file_iea = name_base + "_info_ref.txt"

        filepath_gt = os.path.join(path_info_gt, file_gt)
        filepath_iea = os.path.join(path_info_iea, file_iea)

        replace = False
        if replace == True:
            f = open(filepath_gt,'r')
            filedata = f.read()
            f.close()
            newdata = filedata.replace(";"," ")
            f = open(filepath_gt,'w')
            f.write(newdata)
            f.close()
            continue

        v_size = 25 # TODO
        c1 = v_size / 0.171695423555784
        c2 = v_size / 0.170245588127857
        c3 = v_size / 0.19430948521624
        c4 = v_size / 0.200959192586455

        name_base_int = int(name_base[0:5])
        conversion = c1  
        if name_base_int > 15753:
            conversion = c2 
        if name_base_int > 16037:
            conversion = c3 
        if name_base_int > 16069:
            conversion = c3

        data_gt = np.genfromtxt(filepath_gt)
        data_iea = np.genfromtxt(filepath_iea)

        data_gt = np.delete(data_gt, 0, 0)
        data_iea = np.delete(data_iea, 0, 0)

        while data_gt.shape[0] > 0:

            if data_gt[0,3] == 1:
                print("elbow")
                pos_gt = data_gt[0,0:3]
                pos_iea = data_iea[0,0:3]
                distance = get_distance(pos_gt, pos_iea, 3) * conversion
                elbow_point_list.append(distance)
                data_gt = np.delete(data_gt, 0, 0)
                data_iea = np.delete(data_iea, 0, 0)

            elif data_gt[0,3] == 2:
                print("pipe")

                vector_gt = data_gt[1,0:3] - data_gt[0,0:3]                    
                vector_iea = data_iea[1,0:3] - data_iea[0,0:3]

                angle_diff = angle_between_vectors(vector_gt, vector_iea)
                angle_diff180 = angle_diff + 180
                angle_diff360 = angle_diff + 360
                angle_diff180_2 = angle_diff - 180
                angle_diff360_2 = angle_diff - 360
                angle_diff = min([abs(angle_diff), abs(angle_diff180), abs(angle_diff360), abs(angle_diff180_2), abs(angle_diff360_2)])

                pipe_vector_direction_list.append(angle_diff)

                module_gt = math.sqrt((vector_gt[0]**2)+(vector_gt[1]**2)+(vector_gt[2]**2))
                module_iea = math.sqrt((vector_iea[0]**2)+(vector_iea[1]**2)+(vector_iea[2]**2))
                module_diff = abs(module_gt-module_iea) * conversion
                pipe_vector_size_list.append(module_diff)

                data_gt = np.delete(data_gt, [0,1], 0)
                data_iea = np.delete(data_iea, [0,1], 0)

            elif data_gt[0,3] == 3:

                if data_gt[0,5] == 1:

                    if data_gt[3,4] != 9:
                        print("valve")

                        pos_gt = data_gt[0,0:3]
                        pos_iea = data_iea[0,0:3]
                        distance = get_distance(pos_gt, pos_iea, 3) * conversion
                        valve_central_point_list.append(distance)

                        max_id_gt  = data_gt[3,4]
                        max_id_iea  = data_iea[3,4]

                        if max_id_gt != 8:

                            if max_id_gt == 0 or max_id_gt == 1:
                                gt_id = 2
                            if max_id_gt == 2 or max_id_gt == 3 or max_id_gt == 4:
                                gt_id = 3
                            if max_id_iea == 0 or max_id_iea == 1:
                                iea_id = 2
                            if max_id_iea == 2 or max_id_iea == 3 or max_id_iea == 4:
                                iea_id = 3

                            if gt_id == iea_id:
                                same = True
                            elif gt_id == 2 and iea_id == 3:
                                gt2_iea3 = gt2_iea3 +1
                                same = False
                            elif gt_id == 3 and iea_id == 2:
                                gt3_iea2 = gt3_iea2 +1
                                same = False
                            
                            valve_type_list.append(same)

                        vector_gt = data_gt[2,0:3] - data_gt[1,0:3]                    
                        vector_iea = data_iea[2,0:3] - data_iea[1,0:3]
                        angle_diff = angle_between_vectors(vector_gt, vector_iea)

                        if gt_id == 2 and iea_id == 2:
                            angle_diff180 = angle_diff + 180
                            angle_diff360 = angle_diff + 360
                            angle_diff180_2 = angle_diff - 180
                            angle_diff360_2 = angle_diff - 360
                            angle_diff = min([abs(angle_diff), abs(angle_diff180), abs(angle_diff360), abs(angle_diff180_2), abs(angle_diff360_2)])

                        else:
                            angle_diff90 = angle_diff + 90
                            angle_diff180 = angle_diff + 180
                            angle_diff270 = angle_diff + 270
                            angle_diff360 = angle_diff + 360
                            angle_diff90_2 = angle_diff - 90
                            angle_diff180_2 = angle_diff - 180
                            angle_diff270_2 = angle_diff - 270
                            angle_diff360_2 = angle_diff - 360
                            angle_diff = min([abs(angle_diff), abs(angle_diff90), abs(angle_diff180),abs(angle_diff270), abs(angle_diff360), abs(angle_diff90_2), abs(angle_diff180_2), abs(angle_diff270_2), abs(angle_diff360_2)])

                        valve_vector_direction_list.append(angle_diff)

                    data_gt = np.delete(data_gt, [0,1,2,3], 0)
                    data_iea = np.delete(data_iea, [0,1,2,3], 0)

                elif data_gt[0,5] == 2:
                    print("connection")
                    pos_gt = data_gt[0,0:3]
                    pos_iea = data_iea[0,0:3]
                    distance = get_distance(pos_gt, pos_iea, 3) * conversion
                    conn_central_point_list.append(distance)
                    data_gt = np.delete(data_gt, 0, 0)
                    data_iea = np.delete(data_iea, 0, 0)

    print("   ")
    elbow_point_avg = sum(elbow_point_list) / len(elbow_point_list)
    print("elbow central point: " + str(elbow_point_avg))

    print("   ")
    pipe_vector_direction_avg = sum(pipe_vector_direction_list) / len(pipe_vector_direction_list)
    print("pipe direction angle: " + str(pipe_vector_direction_avg) + "º")
    pipe_vector_size_avg = sum(pipe_vector_size_list) / len(pipe_vector_size_list)
    print("pipe module: " + str(pipe_vector_size_avg))

    print("   ")
    valve_central_point_avg = sum(valve_central_point_list) / len(valve_central_point_list)
    print("valve central point: " + str(valve_central_point_avg))
    valve_vector_direction_avg = sum(valve_vector_direction_list) / len(valve_vector_direction_list)
    print("valve direction angle: " + str(valve_vector_direction_avg) + "º")
    true_count = sum(valve_type_list)
    false_count = len(valve_type_list)-true_count
    percentaje = (true_count/len(valve_type_list))*100
    print("same: " + str(true_count))
    print("not same: " + str(false_count))
    print("gt2_iea3: " + str(gt2_iea3))
    print("gt3_iea2: " + str(gt3_iea2))
    
    print("percentaje: " + str(percentaje))

    print("   ")
    conn_central_point_avg = sum(conn_central_point_list) / len(conn_central_point_list)
    print("connection central point: " + str(conn_central_point_avg)) # TODO REVISAR, MUY BAJO... :D


if __name__ == "__main__":
    main()
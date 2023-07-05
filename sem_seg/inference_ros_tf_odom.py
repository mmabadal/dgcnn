import os
import sys
import time
import copy
import rospy
import ctypes
import struct
import get_info
import map_utils
import numpy as np
from model import *
import open3d as o3d
import indoor3d_util
import get_instances
import conversion_utils
from natsort import natsorted
from scipy.spatial.transform import Rotation as Rot

from std_msgs.msg import Int32
import message_filters
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class Pointcloud_Seg:

    def __init__(self, name):
        self.name = name
        
        # T times
        now = rospy.Time.now()
        tzero = now-now

        self.n_pc = 0
        self.T_read = tzero
        self.T_blocks = tzero
        self.T_inferference = tzero

        self.T_instaces_valve = tzero
        self.T_instaces_pipe = tzero

        self.T_info_valve = tzero
        self.T_info_pipe = tzero

        self.T_ref_valve = tzero
        self.T_ref_pipe = tzero

        self.T_publish =  tzero
        self.T_total = tzero

        # Params inference
        self.fps = 1.0                # target fps        //PARAM
        self.period = 1.0/self.fps    # target period     //PARAM
        self.batch_size = 1         #                     //PARAM
        self.points_sub = 128       #   128               //PARAM
        self.block_sub = 0.1        #   0.1               //PARAM
        self.stride_sub = 0.1       #   0.1               //PARAM
        self.gpu_index = 0          #                     //PARAM
        self.desired_points = int(6000/(128/self.points_sub))  # n of points to wich the received pc will be downsampled    //PARAM

        # get valve matching targets
        self.targets_path = "../valve_targets"      # //PARAM
        self.targets_list = list()
        for file_name in natsorted(os.listdir(self.targets_path)):
            target_path = os.path.join(self.targets_path, file_name)
            target = get_info.read_ply(target_path, "model")
            xyz_central = np.mean(target, axis=0)[0:3]
            target[:, 0:3] -= xyz_central  
            target[:, 2] *= -1                                                  # flip Z axis
            target_o3d = o3d.geometry.PointCloud()
            target_o3d.points = o3d.utility.Vector3dVector(target[:,0:3])
            target_o3d.colors = o3d.utility.Vector3dVector(target[:,3:6])
            target_o3d.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=15))  # //PARAM
            target_o3d.orient_normals_to_align_with_direction(orientation_reference=([0, 0, 1]))                    # //PARAM
            self.targets_list.append(target_o3d)

        # Params get_instance
        self.col_inst = {
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
        self.rad_p = 0.04               # max distance for pipe growing                             //PARAM
        self.rad_v = 0.04               # max distance for valve growing                            //PARAM
        self.dim_p = 3                  # compute 2D (2) or 3D (3) distance for pipe growing        //PARAM
        self.dim_v = 2                  # compute 2D (2) or 3D (3) distance for valve growing       //PARAM
        self.min_p_p = 60               # minimum number of points to consider a blob as a pipe     //PARAM
        self.min_p_v = 30 # 40 80 140   # minimum number of points to consider a blob as a valve    //PARAM

        self.train_path = "RUNS/4_128_11_c9" # path to train
        self.model_path = os.path.join(self.train_path, "model.ckpt")         # path to model         //PARAM
        self.path_cls =  os.path.join(self.train_path, "cls.txt")             # path to clases info   //PARAM
        self.classes, self.labels, self.label2color = indoor3d_util.get_info_classes(self.path_cls) # get classes info

        self.loop = 0

        self.out = True
        self.print = True
        self.time = True
        self.path_in = "/home/bomiquel/Documents/SRV/recerca/slam/pipes/slam_and_pipes/"
        self.path_out = os.path.join(self.path_in, "pipes")
        self.path_graph= os.path.join(self.path_in, "graph_vertices.txt")

        if not os.path.exists(self.path_out):
            os.makedirs(self.path_out)

        self.init = False
        self.new_pc = False

        # set subscribers
        pc_sub = message_filters.Subscriber('/turbot/slamon/points2', PointCloud2)               # //PARAM
        odom_sub = message_filters.Subscriber('/turbot/slamon/graph_robot_odometry', Odometry)   # //PARAM
        #ts_pc_odom = message_filters.TimeSynchronizer([pc_sub, odom_sub], 10)
        ts_pc_odom = message_filters.ApproximateTimeSynchronizer([pc_sub, odom_sub], queue_size=10, slop=0.001)
        ts_pc_odom.registerCallback(self.cb_pc)

        #pc_sub.registerCallback(self.cb_pc)

        loop_sub = message_filters.Subscriber('/turbot/slamon/loop_closings_num', Int32)               # //PARAM
        loop_sub.registerCallback(self.cb_loop)

        # Set class image publishers
        self.pub_pc_base = rospy.Publisher("/turbot/slamon/points2_base", PointCloud2, queue_size=4)
        self.pub_pc_seg = rospy.Publisher("/turbot/slamon/points2_seg", PointCloud2, queue_size=4)
        self.pub_pc_inst = rospy.Publisher("/turbot/slamon/points2_inst", PointCloud2, queue_size=4)
        self.pub_pc_info = rospy.Publisher("/turbot/slamon/points2_info", PointCloud2, queue_size=4)
        self.pub_pc_info_world = rospy.Publisher("/turbot/slamon/points2_info_world", PointCloud2, queue_size=4)

        # Set segmentation timer
        rospy.Timer(rospy.Duration(self.period), self.run)

    def cb_pc(self, pc, odom):
        self.pc = pc
        self.odom = odom
        self.new_pc = True

    def cb_loop(self, loop):
        print("loop is: " + str(self.loop))
        if loop.data != self.loop:
            self.loop = loop.data
            self.update_positions(path_pc, self.path_graph)


    def set_model(self):
        with tfw.device('/gpu:'+str(self.gpu_index)):
            pointclouds_pl, labels_pl = placeholder_inputs(self.batch_size, self.points_sub)
            is_training_pl = tfw.placeholder(tfw.bool, shape=())

            pred = get_model(pointclouds_pl, is_training_pl)
            loss = get_loss(pred, labels_pl)
            pred_softmax = tfw.nn.softmax(pred)

            saver = tfw.train.Saver() # Add ops to save and restore all the variables.
            
        # Create a session
        config = tfw.ConfigProto()
        config.gpu_options.allow_growth = True
        config.allow_soft_placement = True
        config.log_device_placement = True
        self.sess = tfw.Session(config=config)

        # Restore variables from disk.
        saver.restore(self.sess, self.model_path)

        self.ops = {'pointclouds_pl': pointclouds_pl,
            'labels_pl': labels_pl,
            'is_training_pl': is_training_pl,
            'pred': pred,
            'pred_softmax': pred_softmax,
            'loss': loss}

    def run(self,_):
        rospy.loginfo('[%s]: Running', self.name)	
        t0 = rospy.Time.now()

        # New pc available
        if not self.new_pc:
            rospy.loginfo('[%s]: No new pointcloud', self.name)	
            return
        self.new_pc = False

        # Retrieve image
        try:
            pc = self.pc
            header = self.pc.header
            if not self.init:
                rospy.loginfo('[%s]: Start pc segmentation', self.name)	
        except:
            rospy.logwarn('[%s]: There is no input pc to run the segmentation', self.name)
            return

        # Set model
        if not self.init:
            self.set_model()
            self.init = True
            return

        pc_np = self.pc2array(pc)
        if pc_np.shape[0] < 2000:               # return if points < thr   //PARAM
            rospy.loginfo('[%s]: Not enough input points', self.name)
            return

        left2worldned = self.get_transform()


        pc_np[:, 2] *= -1  # flip Z axis        # //PARAM
        #pc_np[:, 1] *= -1  # flip Y axis       # //PARAM

        xyz_min = np.amin(pc_np, axis=0)[0:3]   # get pointcloud mins
        pc_np[:, 0:3] -= xyz_min                # move pointcloud to origin
        xyz_max = np.amax(pc_np, axis=0)[0:3]   # get pointcloud maxs

        t1 = rospy.Time.now()

        # divide data into blocks of size "block" with an stride "stride", in each block select random "points_sub" points
        data_sub, label_sub = indoor3d_util.room2blocks_plus_normalized_parsed(pc_np,  xyz_max, self.points_sub, block_size=self.block_sub, stride=self.stride_sub, random_sample=False, sample_num=None, sample_aug=1) # subsample PC for evaluation

        if data_sub.size == 0:      # return if room2blocks_plus_normalized_parsed has deleted all blocks
            rospy.loginfo('[%s]: No data after block-stride', self.name)
            return

        t2 = rospy.Time.now()

        with tfw.Graph().as_default():
            pred_sub = self.evaluate(data_sub, label_sub, xyz_max)  # evaluate PC

        if pred_sub.size == 0:      # return if no prediction
            rospy.loginfo('[%s]: No prediction', self.name)
            return

        pred_sub = np.unique(pred_sub, axis=0)  # delete duplicates from room2blocks (if points in block < points_sub, it duplicates them)

        pred_sub[:, 0:3] += xyz_min             # recover original position
        pred_sub[:, 2] *= -1                    # unflip Z axis

        t3 = rospy.Time.now()

        pc_np_base = pred_sub.copy()            # for print only
        pc_np_base = np.delete(pc_np_base,6,1)  # delete class prediction

        # downsample prediction to 128 if its not already on 128
        if self.points_sub >= 128:                                             # //PARAM
            down = 128/self.points_sub                                         # //PARAM
            n_idx_pred_sub_down = int(pred_sub.shape[0] * down)  
            idx_pred_sub_down = np.random.choice(pred_sub.shape[0], n_idx_pred_sub_down, replace=False)
            pred_sub = pred_sub[idx_pred_sub_down, 0:7] 
        
        pred_sub_pipe = pred_sub[pred_sub[:,6] == [self.labels["pipe"]]]       # get points predicted as pipe
        pred_sub_valve = pred_sub[pred_sub[:,6] == [self.labels["valve"]]]     # get points predicted as valve

        # get valve instances
        instances_ref_valve_list, pred_sub_pipe_ref, stolen_list  = get_instances.get_instances(pred_sub_valve, self.dim_v, self.rad_v, self.min_p_v, ref=True, ref_data = pred_sub_pipe, ref_rad = 0.1)    # //PARAM
        #instances_ref_valve_list, pred_sub_pipe_ref, stolen_list  = get_instances.get_instances_o3d(pred_sub_valve, self.dim_v, self.rad_v, self.min_p_v, ref=True, ref_data = pred_sub_pipe, ref_rad = 0.1)

        # project valve isntances, removed because produced errors on valve matching due to the points gathered @ the floor
        #instances_ref_valve_list = project_inst.project_inst(instances_ref_valve_list, pc_proj) # pc_np_base 

        t4 = rospy.Time.now()

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

            info_valve = get_info.get_info(inst_o3d, self.targets_list, method="matching")   # get valve instance info list([fitness1, rotation1],[fitness2, rotation2], ...)  len = len(targets_list)
            max_info =  max(info_valve)                                                      # the max() function compares the first element of each info_list element, which is fitness)
            max_idx = info_valve.index(max_info)                                             # idx of best valve match
            
            rad = math.radians(max_info[1])
            vector = np.array([math.cos(rad), math.sin(rad), 0])                             # get valve unit vector at zero
            vector = vector*0.18                                                             # resize vector to valve size //PARAM
            info_valves_list.append([xyz_central, vector, max_idx, inst[:,0:3], max_info])   # append valve instance info

        info_valves_list.append([np.array([0.2,0.2,0.2]),np.array([0.1,0.1,0])],1,np.array([[1,2,3], [4,5,6]]),0.5) # BORRAAAAARR, ESTO ES PARA TEST!!!!!!!

            # print best valve matching
            #trans = np.eye(4) 
            #trans[:3,:3] = inst_o3d.get_rotation_matrix_from_xyz((0,0, -rad))
            #get_info.draw_registration_result(inst_o3d, self.targets_list[max_idx], trans)

        # based on valve fitness, delete it and return stolen points to pipe prediction
        descart_valves_list = [i for i, x in enumerate(info_valves_list) if x[4][0] < 0.4]     # if max fitnes < thr  //PARAM
        for i in descart_valves_list:
            print("Valve descarted")
            descarted_points = np.vstack(instances_ref_valve_list[i])                           # notate points to discard
            if len(stolen_list[i])>0:                                                                  # if there were stolen points
                stolen_idx = list(np.vstack(stolen_list[i])[:,0].astype(int))                       # get stolen idx
                stolen_cls = np.vstack(stolen_list[i])[:,1].astype(int)                             # get stolen class
                stolen_cls = stolen_cls.reshape(stolen_cls.shape[0],1)                              # reshape stolen class
                stolen_points = descarted_points[stolen_idx, :-2]                               # recover stolen points
                stolen_points = np.concatenate((stolen_points,stolen_cls),axis=1)               # concatenate stolen points and stolen class
                pred_sub_pipe_ref = np.concatenate((pred_sub_pipe_ref,stolen_points),axis=0)    # add points and class pipe prediction points
        
        for index in sorted(descart_valves_list, reverse=True):                                 # delete discarted valve info                                                                             
            del info_valves_list[index]
            del instances_ref_valve_list[index]     # for print only  


        
        t5 = rospy.Time.now()

        # get pipe instances
        instances_ref_pipe_list, _, _  = get_instances.get_instances(pred_sub_pipe_ref, self.dim_p, self.rad_p, self.min_p_p)
        #instances_ref_pipe_list, _, _  = get_instances.get_instances_o3d(pred_sub_pipe_ref, self.dim_p, self.rad_p, self.min_p_p)

        t6 = rospy.Time.now()

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


        t7 = rospy.Time.now()

        info_pipes_list_copy = copy.deepcopy(info_pipes_list) 
        info_connexions_list_copy = copy.deepcopy(info_connexions_list)
        info_pipes_list2, info_connexions_list2 = get_info.unify_chains(info_pipes_list_copy, info_connexions_list_copy)  

        t8 = rospy.Time.now()

        info_valves_list_copy = copy.deepcopy(info_valves_list)
        info_valves_list2 = get_info.refine_valves(info_valves_list_copy, info_pipes_list2) 

        t9 = rospy.Time.now()

        #info1 = [info_pipes_list, info_connexions_list, info_valves_list, instances_ref_pipe_list]
        #info2 = [info_pipes_list2, info_connexions_list2, info_valves_list, instances_ref_pipe_list] 
        info3 = [info_pipes_list2, info_connexions_list2, info_valves_list2, instances_ref_pipe_list]

        if len(info_pipes_list2)>0 or len(info_valves_list2)>0:
            info_array = conversion_utils.info_to_array(info3)
            pc_info = self.array2pc_info(header, info_array)
            self.pub_pc_info.publish(pc_info)

            info_array_world = info_array.copy()
            for i in range(info_array.shape[0]):
                xyz = np.array([[info_array[i,0]],
                                [info_array[i,1]],
                                [info_array[i,2]],
                                [1]])
                xyz_trans_rot = np.matmul(left2worldned, xyz)
                info_array_world[i,0:3] = [xyz_trans_rot[0], xyz_trans_rot[1], xyz_trans_rot[2]]

            header.frame_id = "world_ned"
            pc_info_world = self.array2pc_info(header, info_array_world)
            self.pub_pc_info_world.publish(pc_info_world)

            if self.out == True:   

                path_out_info_ply = os.path.join(self.path_out, str(header.stamp) + "_info.ply")
                path_out_info_npy = os.path.join(self.path_out, str(header.stamp) + "_info.npy")

                np.save(path_out_info_npy, info_array)

                conversion_utils.info_to_ply(info3, path_out_info_ply)
              
                path_out_world_info = os.path.join(self.path_out, str(header.stamp)+"_info_world.ply")
                info_pipes_world_list, info_connexions_world_list, info_valves_world_list, info_inst_pipe_world_list = conversion_utils.array_to_info(info_array_world)
                info_world = [info_pipes_world_list, info_connexions_world_list, info_valves_world_list, info_inst_pipe_world_list]
                conversion_utils.info_to_ply(info_world, path_out_world_info)

                pred_sub_world = pred_sub.copy()
                for i in range(pred_sub.shape[0]):
                    xyz = np.array([[pred_sub[i,0]],
                                    [pred_sub[i,1]],
                                    [pred_sub[i,2]],
                                    [1]])
                    xyz_trans_rot = np.matmul(left2worldned, xyz)
                    pred_sub_world[i,0:3] = [xyz_trans_rot[0], xyz_trans_rot[1], xyz_trans_rot[2]]

                path_out_base = os.path.join(self.path_out, str(header.stamp)+"_base.obj")
                path_out_pred = os.path.join(self.path_out, str(header.stamp)+"_pred.obj")
                fout_base = open(path_out_base, 'w')
                fout_pred = open(path_out_pred, 'w')
                for i in range(pred_sub.shape[0]):
                    fout_base.write('v %f %f %f %d %d %d\n' % (pred_sub[i,0], pred_sub[i,1], pred_sub[i,2], pred_sub[i,3], pred_sub[i,4], pred_sub[i,5]))
                for i in range(pred_sub.shape[0]):
                    color = self.label2color[pred_sub[i,6]]
                    fout_pred.write('v %f %f %f %d %d %d\n' % (pred_sub[i,0], pred_sub[i,1], pred_sub[i,2], color[0], color[1], color[2]))
                

                path_out_world_base = os.path.join(self.path_out, str(header.stamp)+"_base_world.obj")
                path_out_world_pred = os.path.join(self.path_out, str(header.stamp)+"_pred_world.obj")
                fout_base = open(path_out_world_base, 'w')
                fout_pred = open(path_out_world_pred, 'w')
                for i in range(pred_sub_world.shape[0]):
                    fout_base.write('v %f %f %f %d %d %d\n' % (pred_sub_world[i,0], pred_sub_world[i,1], pred_sub_world[i,2], pred_sub_world[i,3], pred_sub_world[i,4], pred_sub_world[i,5]))
                for i in range(pred_sub_world.shape[0]):
                    color = self.label2color[pred_sub_world[i,6]]
                    fout_pred.write('v %f %f %f %d %d %d\n' % (pred_sub_world[i,0], pred_sub_world[i,1], pred_sub_world[i,2], color[0], color[1], color[2]))
                


            header.frame_id = "turbot/stereo_down/left_optical"

        t10 = rospy.Time.now()

        # publishers

        n_v = len(instances_ref_valve_list)

        if len(instances_ref_valve_list)>0:
            instances_ref_proj_valve = np.vstack(instances_ref_valve_list)
        if len(instances_ref_pipe_list)>0:
            instances_ref_pipe = np.vstack(instances_ref_pipe_list)
            instances_ref_pipe[:,7] = instances_ref_pipe[:,7]+n_v

        if len(instances_ref_valve_list)>0 and len(instances_ref_pipe_list)>0:
            instances_ref = np.concatenate((instances_ref_proj_valve, instances_ref_pipe), axis=0)
        elif len(instances_ref_valve_list)==0 and len(instances_ref_pipe_list)>0:
            instances_ref = instances_ref_pipe
        elif len(instances_ref_valve_list)>0 and len(instances_ref_pipe_list)==0:
            instances_ref = instances_ref_proj_valve
        else:
            instances_ref = None

        if instances_ref is None: # if instances were not found
            rospy.loginfo('[%s]: No instances found', self.name)	
            return

        if len(info_pipes_list2)>0 or len(info_valves_list2)>0:  # print here because instrances_ref is needed
            if self.out == True:
                instances_ref_world = instances_ref.copy()
                for i in range(instances_ref.shape[0]):
                    xyz = np.array([[instances_ref[i,0]],
                                    [instances_ref[i,1]],
                                    [instances_ref[i,2]],
                                    [1]])
                    xyz_trans_rot = np.matmul(left2worldned, xyz)
                    instances_ref_world[i,0:3] = [xyz_trans_rot[0], xyz_trans_rot[1], xyz_trans_rot[2]]

                path_out_inst = os.path.join(self.path_out, str(header.stamp)+"_inst.obj")
                fout_pred = open(path_out_inst, 'w')
                for i in range(instances_ref.shape[0]):
                    color = self.col_inst[instances_ref[i,7]]
                    fout_pred.write('v %f %f %f %d %d %d\n' % (instances_ref[i,0], instances_ref[i,1], instances_ref[i,2], color[0], color[1], color[2]))

                path_out_world_inst = os.path.join(self.path_out, str(header.stamp)+"_inst_world.obj")
                fout_pred = open(path_out_world_inst, 'w')
                for i in range(instances_ref_world.shape[0]):
                    color = self.col_inst[instances_ref_world[i,7]]
                    fout_pred.write('v %f %f %f %d %d %d\n' % (instances_ref_world[i,0], instances_ref_world[i,1], instances_ref_world[i,2], color[0], color[1], color[2]))

        for i in range(pred_sub.shape[0]):
            color = self.label2color[pred_sub[i,6]]
            pred_sub[i,3] = color[0]
            pred_sub[i,4] = color[1]
            pred_sub[i,5] = color[2]

        for i in range(instances_ref.shape[0]):
            color = self.col_inst[instances_ref[i,7]]
            instances_ref[i,3] = color[0]
            instances_ref[i,4] = color[1]
            instances_ref[i,5] = color[2]

        pc_base = self.array2pc(header, pc_np_base)
        pc_seg = self.array2pc(header, pred_sub)
        pc_inst = self.array2pc(header, instances_ref)
        self.pub_pc_base.publish(pc_base)
        self.pub_pc_seg.publish(pc_seg)
        self.pub_pc_inst.publish(pc_inst)


        if self.print == True: # print info
            print(" ")
            print("INFO VALVES:")
            for valve in info_valves_list:
                valve.pop(-2)
                print(valve)
            print(" ")
            print("INFO VALVES REF:")
            for valve in info_valves_list2:
                valve.pop(-2)
                print(valve)
            print(" ")
            print("INFO PIPES:")
            for pipe1 in info_pipes_list:
                pipe1.pop(0)
                print(pipe1)
            print(" ")
            print("INFO PIPES REF:")
            for pipe2 in info_pipes_list2:
                pipe2.pop(0)
                print(pipe2)
            print(" ")
            print("INFO CONNEXIONS:")
            for connexion in info_connexions_list:
                print(connexion)
            print(" ")
            print("INFO CONNEXIONS REF:")
            for connexion in info_connexions_list2:
                print(connexion)
            print(" ")

        if self.time == True:             # print time
            time_read = t1-t0
            time_blocks = t2-t1
            time_inferference = t3-t2
            time_instaces_valve = t4-t3
            time_instaces_pipe = t6-t5
            time_instaces = time_instaces_valve + time_instaces_pipe
            time_info_valve = t5-t4
            time_info_pipe = t7-t6
            time_info = time_info_valve + time_info_pipe
            time_ref_valve = t9-t8
            time_ref_pipe = t8-t7
            time_ref = time_ref_valve + time_ref_pipe
            time_publish = t10-t9
            time_total = t10-t0

            rospy.loginfo('[%s]: INFO TIMES:', self.name)	
            print("")
            rospy.loginfo('[%s]: Pc processing took %.2f seconds. Split into:', self.name, time_total.secs + time_total.nsecs*1e-9)
            rospy.loginfo('[%s]: Reading -------- %.2f seconds (%i%%)', self.name, time_read.secs + time_read.nsecs*1e-9, (time_read/time_total)*100)
            rospy.loginfo('[%s]: Blocks --------- %.2f seconds (%i%%)', self.name, time_blocks.secs + time_blocks.nsecs*1e-9, (time_blocks/time_total)*100)
            rospy.loginfo('[%s]: Inference ------ %.2f seconds (%i%%)', self.name, time_inferference.secs + time_inferference.nsecs*1e-9, (time_inferference/time_total)*100)
            rospy.loginfo('[%s]: Instances ------ %.2f seconds (%i%%)', self.name, time_instaces.secs + time_instaces.nsecs*1e-9, (time_instaces/time_total)*100)
            rospy.loginfo('[%s]:  - Valve - %.2f seconds (%i%%)', self.name, time_instaces_valve.secs + time_instaces_valve.nsecs*1e-9, (time_instaces_valve/time_total)*100)
            rospy.loginfo('[%s]:  - Pipe -- %.2f seconds (%i%%)', self.name, time_instaces_pipe.secs + time_instaces_pipe.nsecs*1e-9, (time_instaces_pipe/time_total)*100)
            rospy.loginfo('[%s]: Info ----------- %.2f seconds (%i%%)', self.name, time_info.secs + time_info.nsecs*1e-9, (time_info/time_total)*100)
            rospy.loginfo('[%s]:  - Valve - %.2f seconds (%i%%)', self.name, time_info_valve.secs + time_info_valve.nsecs*1e-9, (time_info_valve/time_total)*100)
            rospy.loginfo('[%s]:  - Pipe -- %.2f seconds (%i%%)', self.name, time_info_pipe.secs + time_info_pipe.nsecs*1e-9, (time_info_pipe/time_total)*100)
            rospy.loginfo('[%s]: Refine --------- %.2f seconds (%i%%)', self.name, time_ref.secs + time_ref.nsecs*1e-9, (time_ref/time_total)*100)
            rospy.loginfo('[%s]:  - Valve - %.2f seconds (%i%%)', self.name, time_ref_valve.secs + time_ref_valve.nsecs*1e-9, (time_ref_valve/time_total)*100)
            rospy.loginfo('[%s]:  - Pipe -- %.2f seconds (%i%%)', self.name, time_ref_pipe.secs + time_ref_pipe.nsecs*1e-9, (time_ref_pipe/time_total)*100)
            rospy.loginfo('[%s]: Publish -------- %.2f seconds (%i%%)', self.name, time_publish.secs + time_publish.nsecs*1e-9, (time_publish/time_total)*100)


    def pc2array(self, ros_pc):
        gen = pc2.read_points(ros_pc, skip_nans=True)   # ROS pointcloud into generator
        pc_np = np.array(list(gen))                     # generator to list to numpy

        if pc_np.size > 0:                              # if there are points

            if self.desired_points != 0:                # downsample pointcloud to desired points
                if pc_np.shape[0] > self.desired_points:
                    idx_sub = np.random.choice(pc_np.shape[0], self.desired_points, replace=False)
                    pc_np = pc_np[idx_sub, :]

            rgb_list = list()

            for rgb in pc_np[...,3]:
                # cast float32 to int so that bitwise operations are possible
                s = struct.pack('>f' ,rgb)
                i = struct.unpack('>l',s)[0]
                # get back the float value by the inverse operations
                pack = ctypes.c_uint32(i).value
                r = (pack & 0x00FF0000)>> 16
                g = (pack & 0x0000FF00)>> 8
                b = (pack & 0x000000FF)
                rgb_np = np.array([r,g,b])
                rgb_list.append(rgb_np)

            rgb = np.vstack(rgb_list)
            pc_np = np.delete(pc_np, 3, 1) 
            pc_np = np.concatenate((pc_np, rgb), axis=1)

        return pc_np


    def array2pc(self, header, array):

        fields =   [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('rgba', 12, PointField.UINT32, 1)]
        
        points = list()

        for i, p in enumerate(array):
            r = int(p[3])
            g = int(p[4])
            b = int(p[5])
            a = 255
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

            p_rgb = [p[0], p[1], p[2], rgb]
            points.append(p_rgb)

        pc = pc2.create_cloud(header, fields, points)
        return pc


    def array2pc_info(self, header, array):

        fields =   [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('rgba', 12, PointField.UINT32, 1),
                    PointField('t', 16, PointField.FLOAT32, 1),
                    PointField('info', 20, PointField.FLOAT32, 1),
                    PointField('c', 24, PointField.FLOAT32, 1),
                    PointField('inst', 28, PointField.FLOAT32, 1)]
        
        points = list()

        for i, p in enumerate(array):
            r = int(p[3])
            g = int(p[4])
            b = int(p[5])
            a = 255

            if p[6] == 6:                   # if its type instance data, make it transparent, information still there
                a = 0

            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

            p_rgb = [p[0], p[1], p[2], rgb, p[6], p[7], p[8], p[9]]
            points.append(p_rgb)

        pc = pc2.create_cloud(header, fields, points)
        return pc


    def pc_info2array(self, ros_pc):
        gen = pc2.read_points(ros_pc, skip_nans=True)   # ROS pointcloud into generator
        pc_np = np.array(list(gen))                     # generator to list to numpy
        pc_np = np.delete(pc_np, 3, 1) 
        return pc_np


    def evaluate(self, data, label, xyz_max):

        is_training = False

        label = np.squeeze(label)

        num_batches = data.shape[0] // self.batch_size

        pred_label_list =list()

        for batch_idx in range(num_batches):
            start_idx = batch_idx * self.batch_size
            end_idx = (batch_idx+1) * self.batch_size
            
            feed_dict = {self.ops['pointclouds_pl']: data[start_idx:end_idx, :, :],
                        self.ops['labels_pl']: label[start_idx:end_idx],
                        self.ops['is_training_pl']: is_training}

            loss_val, pred_val = self.sess.run([self.ops['loss'], self.ops['pred_softmax']],feed_dict=feed_dict)

            pred_label = np.argmax(pred_val, 2)
            pred_label = pred_label.reshape(pred_label.shape[0]*pred_label.shape[1],1)
            pred_label_list.append(pred_label)

        if pred_label_list:
            pred_label_stacked = np.vstack(pred_label_list)  

            data = data.reshape((data.shape[0]*data.shape[1]), data.shape[2])
            data = np.delete(data, [0,1,2], 1)
            data[:, [0,1,2,3,4,5,]] = data[:, [3,4,5,0,1,2]] 
            data[:,0] *= xyz_max[0]
            data[:,1] *= xyz_max[1]
            data[:,2] *= xyz_max[2]
            data[:,3:] *= 255.0
            
            data = data[:pred_label_stacked.shape[0], :]

            pred_sub = np.hstack([data,pred_label_stacked])  
        else:
            pred_sub = np.array([])
        return pred_sub


    def get_transform(self):

        # with open(os.path.join(self.path_in, "graph_vertices.txt"), "r") as file:
        #     tq_ned_baselink = file.readlines()[-1]
        # tq_ned_baselink_f =  np.array([float(x) for x in tq_ned_baselink.split(",")])

        # t_ned_baselink = tq_ned_baselink_f[2:5]
        # q_ned_baselink = tq_ned_baselink_f[5:]

        t_ned_baselink = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z]
        q_ned_baselink = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
    
        tq_baselink_stick = np.array([0.4, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0])
        t_baselink_stick = tq_baselink_stick[:3]
        q_baselink_stick = tq_baselink_stick[3:]

        tq_stick_downbase = np.array([0.0, 0.0, 0.0, 0.4999998414659176, 0.49960183664463365, 0.4999998414659176, 0.5003981633553665])
        t_stick_downbase = tq_stick_downbase[:3]
        q_stick_downbase = tq_stick_downbase[3:]

        tq_downbase_down = np.array([0.0, 0.0, 0.0, -0.706825181105366, 0.0, 0.0, 0.7073882691671998])
        t_downbase_down = tq_downbase_down[:3]
        q_downbase_down = tq_downbase_down[3:]

        tq_down_left = np.array([-0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        t_down_left = tq_down_left[:3]
        q_down_left = tq_down_left[3:]

        tr_ned_baselink = self.get_tr(t_ned_baselink, q_ned_baselink)
        tr_baselink_stick = self.get_tr(t_baselink_stick, q_baselink_stick)
        tr_stick_downbase = self.get_tr(t_stick_downbase, q_stick_downbase)
        tr_downbase_down  = self.get_tr(t_downbase_down, q_downbase_down)
        tr_down_left = self.get_tr(t_down_left, q_down_left)

        tr_ned_stick = np.matmul(tr_ned_baselink, tr_baselink_stick)
        tr_ned_downbase = np.matmul(tr_ned_stick, tr_stick_downbase)
        tr_ned_down = np.matmul(tr_ned_downbase, tr_downbase_down)
        tr_ned_left = np.matmul(tr_ned_down, tr_down_left)

        return tr_ned_left
    

    def update_positions(self):

        tq_baselink_stick = np.array([0.4, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0])
        t_baselink_stick = tq_baselink_stick[:3]
        q_baselink_stick = tq_baselink_stick[3:]

        tq_stick_downbase = np.array([0.0, 0.0, 0.0, 0.4999998414659176, 0.49960183664463365, 0.4999998414659176, 0.5003981633553665])
        t_stick_downbase = tq_stick_downbase[:3]
        q_stick_downbase = tq_stick_downbase[3:]

        tq_downbase_down = np.array([0.0, 0.0, 0.0, -0.706825181105366, 0.0, 0.0, 0.7073882691671998])
        t_downbase_down = tq_downbase_down[:3]
        q_downbase_down = tq_downbase_down[3:]

        tq_down_left = np.array([-0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        t_down_left = tq_down_left[:3]
        q_down_left = tq_down_left[3:]

        tr_baselink_stick = self.get_tr(t_baselink_stick, q_baselink_stick)
        tr_stick_downbase = self.get_tr(t_stick_downbase, q_stick_downbase)
        tr_downbase_down  = self.get_tr(t_downbase_down, q_downbase_down)
        tr_down_left = self.get_tr(t_down_left, q_down_left)


        file_tq = open(self.path_graph, 'r')
        lines = file_tq.readlines()
        for line in lines:

            info = [float(x) for x in line.split(',')]
            t_ned_baselink = info[2:5]
            q_ned_baselink = info[5:]
            
            tr_ned_baselink = self.get_tr(t_ned_baselink, q_ned_baselink)

            tr_ned_stick = np.matmul(tr_ned_baselink, tr_baselink_stick)
            tr_ned_downbase = np.matmul(tr_ned_stick, tr_stick_downbase)
            tr_ned_down = np.matmul(tr_ned_downbase, tr_downbase_down)
            tr_ned_left = np.matmul(tr_ned_down, tr_down_left)

            name = info[0]
            name = name.replace('.', '')
            name = list(name)
            name[-3:] = '000'
            name = ''.join(name)

            file_pc = os.path.join(self.path_out, name + '_info.npy')

            if os.path.exists(file_pc):
                info_array = np.load(file_pc)

                info_array_world = info_array.copy()
                for i in range(info_array.shape[0]):
                    xyz = np.array([[info_array[i,0]],
                                    [info_array[i,1]],
                                    [info_array[i,2]],
                                    [1]])
                    xyz_trans_rot = np.matmul(tr_ned_left, xyz)
                    info_array_world[i,0:3] = [xyz_trans_rot[0], xyz_trans_rot[1], xyz_trans_rot[2]]

                path_out_world_info = os.path.join(self.path_out, name + "_info_world.ply")
                info_pipes_world_list, info_connexions_world_list, info_valves_world_list, info_inst_pipe_world_list = conversion_utils.array_to_info(info_array_world)
                info_world = [info_pipes_world_list, info_connexions_world_list, info_valves_world_list, info_inst_pipe_world_list]
                conversion_utils.info_to_ply(info_world, path_out_world_info)


    def quaternion_multiply(self, q0, q1):
        x0, y0, z0, w0 = q0
        x1, y1, z1, w1 = q1
        return np.array([-x1*x0-y1*y0-z1*z0+w1*w0, x1*w0+y1*z0-z1*y0+w1*x0, -x1*z0+y1*w0+z1*x0+w1*y0, x1*y0-y1*x0+z1*w0+w1*z0], dtype=np.float64)
    
    def get_tr(self, t, q):
        trans = np.array([[1, 0, 0, t[0]], [0, 1, 0, t[1]], [0, 0, 1, t[2]], [0, 0, 0, 1]], np.float)
        rot = Rot.from_quat(q)
        rot_mat = rot.as_matrix()
        trans_rot = copy.deepcopy(trans)
        trans_rot[0:3, 0:3] = rot_mat
        return(trans_rot)

if __name__ == '__main__':
    try:
        rospy.init_node('seg_pc')
        Pointcloud_Seg(rospy.get_name())

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
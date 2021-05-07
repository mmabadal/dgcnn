import tf
import os
import sys
import time
import copy
import rospy
import ctypes
import struct
import get_info
import numpy as np
#import open3d as o3d
#import indoor3d_util
from natsort import natsorted

from sensor_msgs.msg import PointField
import message_filters
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

class Pointcloud_World:

    def __init__(self, name):
        self.name = name

        # init info
        self.info_world_all = list()
        self.info_pipes_list_overall = list()
        self.instances_pipe_list_overall = list()
        self.info_valves_list_overall = list()
        self.info_connexions_list_overall = list()

        # Params inference
        self.fps = 4.0                # target fps        //PARAM
        self.period = 1.0/self.fps    # target period     //PARAM

        self.init = False
        self.new_pc = False

        # listener
        self.listener = tf.TransformListener()

        # set subscribers
        pc_info = message_filters.Subscriber('/stereo_down/scaled_x2/points2_info', PointCloud2)     # //PARAM
        pc_info.registerCallback(self.cb_pc)

        # set publishers
        self.pub_pc_info_world = rospy.Publisher("/stereo_down/scaled_x2/points2_info_world", PointCloud2, queue_size=4)
        self.pub_pc_info_world_all = rospy.Publisher("/stereo_down/scaled_x2/points2_info_world_all", PointCloud2, queue_size=4)

        # Set segmentation timer
        rospy.Timer(rospy.Duration(self.period), self.run)

    def cb_pc(self, pc):
        self.pc = pc
        self.new_pc = True

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
                rospy.loginfo('[%s]: Start frame_id change', self.name)	
        except:
            rospy.logwarn('[%s]: There is no input pc to run the frame_id change', self.name)
            return
            
        left_frame_id = "turbot/stereo_down/left_optical"
        world_frame_id = "world_ned"         
        header.frame_id = "world_ned"    
        
        left2worldned = self.get_transform(world_frame_id, left_frame_id)

        if isinstance(left2worldned,int) == False:

            pc_np_info = self.pc_info2array(pc)
            pc_np_info_world = pc_np_info.copy()

            for i in range(pc_np_info.shape[0]):
                xyz = np.array([[pc_np_info[i,0]],
                                 [pc_np_info[i,1]],
                                 [pc_np_info[i,2]],
                                 [1]])
                xyz_trans_rot = np.matmul(left2worldned, xyz)
                pc_np_info_world[i,0:3] = [xyz_trans_rot[0], xyz_trans_rot[1], xyz_trans_rot[2]]

            info_pipes_list, info_inst_pipe_list, info_valves_list, info_connexions_list = self.extract_info(pc_np_info_world)
            info_world = [info_pipes_list, info_connexions_list, info_valves_list, info_inst_pipe_list]

            if len(info_world)>0:
                pc_np_info_world = get_info.info_to_array(info_world)
                pc_info_world = self.array2pc_info(header, pc_np_info_world)
                self.pub_pc_info_world.publish(pc_info_world)

            print(" ")
            print("INFO VALVES:")
            for valve in info_valves_list:
                print(valve)
            print(" ")

            print("INFO PIPES:")
            for pipe1 in info_pipes_list:
                print(type(pipe1[0]))
                pipe1.pop(0)
                print(pipe1)
            print(" ")

            print("INFO CONNEXIONS:")
            for connexion in info_connexions_list:
                print(connexion)
            print(" ")

            # juntar (si se juntan pipes, pasar union de puntos por 128)
            # si se juntan pipes, recarcular connexions y pipes_near

            # si k = x, hacer check de info world all y borrar instances aparecidas menos de y veces, k = 0
            # si se borra pipe, recalcular pipes near



            
        #if len(info_world_all)>0:
        #    pc_np_info_world_all = get_info.info_to_array(self.info_world_all)
        #    pc_info_world_all = self.array2pc_info(header, pc_np_info_world_all)
        #    self.pub_pc_info_world_all.publish(pc_info_world_all)



    def get_transform(self, parent, child):
        try:
            rospy.logwarn("[%s]: waiting transform from %s to %s", self.name, parent, child)
            self.listener.waitForTransform(parent, child, rospy.Time(), rospy.Duration(0.3))
            (trans, rot) = self.listener.lookupTransform(parent, child, rospy.Time())
            rospy.loginfo("[%s]: transform for %s found", self.name, child)
            transform = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
            rospy.logerr('[%s]: define %s transform!', self.name, child)
            transform = 0
        return transform


    def pc_info2array(self, ros_pc):
        gen = pc2.read_points(ros_pc, skip_nans=True)   # ROS pointcloud into generator
        pc_np = np.array(list(gen))                     # generator to list to numpy
        pc_np = np.delete(pc_np, 3, 1) 
        return pc_np


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


    def extract_info(self, pc_np_info):

        info_pipes_list = list()
        info_inst_pipe_list = list()
        info_valves_list = list()
        info_connexions_list = list()

        for i in set(pc_np_info[:,6]):  # for each instance
            inst = pc_np_info[pc_np_info[:,6] == i]

            if inst[0,5] == 0:

                if inst[0,3] == 0:                              # PIPE
                    info_pipe = list()
                    
                    skeleton = inst[inst[:,3] == 0]
                    info_pipe.append(skeleton[:,0:3])

                    elbows = inst[inst[:,3] == 1]
                    elbows_list = list()
                    for j in range(elbows.shape[0]):
                        elbows_list.append(elbows[j,0:3])
                    info_pipe.append(elbows_list)

                    vectors = inst[inst[:,3] == 2]
                    vectors_list = list()
                    for j in range(int(vectors.shape[0]/2)):
                        k = 2*j
                        vp1 = vectors[k,0:3]
                        vp2 = vectors[k+1,0:3]
                        v = vp2-vp1
                        vectors_list.append(v)
                    info_pipe.append(vectors_list)

                    belong_insts = inst[inst[:,3] == 7]
                    belong_insts_list = list()
                    for j in range(int(belong_insts.shape[0])):
                        belong_insts_list.append(int(belong_insts[j,4]))
                    info_pipe.append(belong_insts_list)

                    info_pipes_list.append(info_pipe)

                else:                                           # INST PIPE
                    info_inst_pipe_list.append(inst[:,0:3])


            elif inst[0,5] == 1:                                # VALVE
                info_valve = list()

                info_valve.append(inst[0,0:3])      # central point

                vp1 = inst[1,0:3]
                vp2 = inst[2,0:3]
                v = vp2 -vp1
                info_valve.append(v)

                info_valve.append(int(inst[3,4]))      # max_id

                near_pipes = inst[inst[:,3] == 4]
                near_pipes_list = list()
                for j in range(int(near_pipes.shape[0])):
                    near_pipes_list.append(int(near_pipes[j,4]))
                info_valve.append(near_pipes_list)

                info_valves_list.append(info_valve)

            else:                                               # CONNEXION
                info_connexion = list()

                info_connexion.append(inst[0,0:3])      # central point

                near_pipes = inst[inst[:,3] == 4]
                near_pipes_list = list()
                for j in range(int(near_pipes.shape[0])):
                    near_pipes_list.append(int(near_pipes[j,4]))
                info_connexion.append(near_pipes_list)

                info_connexions_list.append(info_connexion)

        return info_pipes_list, info_inst_pipe_list, info_valves_list, info_connexions_list



if __name__ == '__main__':
    try:
        rospy.init_node('info_world')
        Pointcloud_World(rospy.get_name())

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
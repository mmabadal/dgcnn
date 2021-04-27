import os
import sys
import time
import copy
import rospy
import ctypes
import struct
import get_info
import numpy as np
from model import *
#import project_inst
import open3d as o3d
import indoor3d_util
import get_instances
from natsort import natsorted

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import PointField

import message_filters
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image, CameraInfo


class Info_Sub:

    def __init__(self, name):
        self.name = name

        # Params inference
        self.fps = 2                # target fps        //PARAM
        self.period = 1/self.fps    # target period     //PARAM

        self.init = False
        self.new_pc = False

        # set subscribers
        pc_sub = message_filters.Subscriber('/stereo_down/scaled_x2/points2_info', PointCloud2)     # //PARAM
        pc_sub.registerCallback(self.cb_pc)

        # Set segmentation timer
        rospy.Timer(rospy.Duration(self.period), self.run)

    def cb_pc(self, pc):
        self.pc = pc
        self.new_pc = True


    def run(self,_):

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

        pc_np = self.pc_info2array(pc)
        print(pc_np)

    
    def pc_info2array(self, ros_pc):
        gen = pc2.read_points(ros_pc, skip_nans=True)   # ROS pointcloud into generator
        pc_np = np.array(list(gen))                     # generator to list to numpy
        pc_np = np.delete(pc_np, 3, 1) 
        return pc_np


if __name__ == '__main__':
    try:
        rospy.init_node('info_sub')
        Info_Sub(rospy.get_name())

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
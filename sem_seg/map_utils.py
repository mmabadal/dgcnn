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
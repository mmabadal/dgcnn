import os
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import copy
import conversion_utils



def get_tr(t, q):
    trans = np.array([[1, 0, 0, t[0]], [0, 1, 0, t[1]], [0, 0, 1, t[2]], [0, 0, 0, 1]], np.float)
    rot = Rot.from_quat(q)
    rot_mat = rot.as_matrix()
    trans_rot = copy.deepcopy(trans)
    trans_rot[0:3, 0:3] = rot_mat
    return(trans_rot)


def update_positions(path_pc, path_tq):

    file_tq = open(path_tq, 'r')
    lines = file_tq.readlines()
    for line in lines:

        info = [float(x) for x in line.split(',')]
        t_ned_baselink = info[2:5]
        q_ned_baselink = info[5:]

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

        tr_ned_baselink = get_tr(t_ned_baselink, q_ned_baselink)
        tr_baselink_stick = get_tr(t_baselink_stick, q_baselink_stick)
        tr_stick_downbase = get_tr(t_stick_downbase, q_stick_downbase)
        tr_downbase_down  = get_tr(t_downbase_down, q_downbase_down)
        tr_down_left = get_tr(t_down_left, q_down_left)

        tr_ned_stick = np.matmul(tr_ned_baselink, tr_baselink_stick)
        tr_ned_downbase = np.matmul(tr_ned_stick, tr_stick_downbase)
        tr_ned_down = np.matmul(tr_ned_downbase, tr_downbase_down)
        tr_ned_left = np.matmul(tr_ned_down, tr_down_left)

        name = info[0]
        name = name.replace('.', '')
        name = list(name)
        name[-3:] = '000'
        name = ''.join(name)

        file_pc = os.path.join(path_pc, name + '_info.npy')

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

            path_out = os.path.join(path_pc, name + "_info_world.ply")
            info_pipes_world_list, info_connexions_world_list, info_valves_world_list, info_inst_pipe_world_list = conversion_utils.array_to_info(info_array_world)
            info_world = [info_pipes_world_list, info_connexions_world_list, info_valves_world_list, info_inst_pipe_world_list]
            conversion_utils.info_to_ply(info_world, path_out)

    
if __name__=='__main__':

    path_pc = "path/pc"
    path_tq = "path/tq"

    loop_local = 0 
    loop_sub = 0
    if loop_sub != loop_local:
        loop_local = loop_sub
        update_positions(path_pc, path_tq)

    

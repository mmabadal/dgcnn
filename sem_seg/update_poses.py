import os
import copy
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def get_tr(t, q):
    trans = np.array([[1, 0, 0, t[0]], [0, 1, 0, t[1]], [0, 0, 1, t[2]], [0, 0, 0, 1]], float)
    rot = Rot.from_quat(q)
    rot_mat = rot.as_matrix()
    trans_rot = copy.deepcopy(trans)
    trans_rot[0:3, 0:3] = rot_mat
    return(trans_rot)


path_out = "/home/miguel/Desktop/data/conboser/experiment_1/output/pipes"
path_graph = "/home/miguel/Desktop/data/conboser/experiment_1/output/keyframes_poses.txt"


tq_baselink_stereodown = np.array([0.57, -0.062, 0.505, 0.0, 0.0, 0.0, 1.0])
t_baselink_stereodown = tq_baselink_stereodown[:3]
q_baselink_stereodown = tq_baselink_stereodown[3:]

tq_stereodown_leftoptical = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.706825181105366, 0.7073882691671998])
t_stereodown_leftoptical = tq_stereodown_leftoptical[:3]
q_stereodown_leftoptical = tq_stereodown_leftoptical[3:]

tr_baselink_stereodown = get_tr(t_baselink_stereodown, q_baselink_stereodown)
tr_stereodown_leftoptical = get_tr(t_stereodown_leftoptical, q_stereodown_leftoptical)

file_tq = open(path_graph, 'r')

lines = file_tq.readlines()[1:]

for idx, line in enumerate(lines):

    info = [float(x) for x in line.split(',')]
    t_ned_baselink = info[1:4]
    q_ned_baselink = info[4:]

    tr_ned_baselink = get_tr(t_ned_baselink, q_ned_baselink)

    tr_ned_stereodown = np.matmul(tr_ned_baselink, tr_baselink_stereodown)
    tr_ned_leftoptical = np.matmul(tr_ned_stereodown, tr_stereodown_leftoptical)

    ts_float = info[0]

    files = os.listdir(path_out)
    for file in files:
        name = file.split('_')[0]
        header_float = float(name[:10] + '.' + name[10:])

        time_dif = abs(ts_float-header_float)
        #print(f"time_dif: {time_dif}")

        if time_dif < 0.1:
            id = idx+1
            break

    file_pc = os.path.join(path_out, name + '_info.npy')

    if os.path.exists(file_pc):
        info_array = np.load(file_pc)

        info_array_slam = info_array.copy()
        for i in range(info_array.shape[0]):
            xyz = np.array([[info_array[i,0]],
                            [info_array[i,1]],
                            [info_array[i,2]],
                            [1]])
            xyz_trans_rot = np.matmul(tr_ned_leftoptical, xyz) # np.matmul(tr_ned_baselink, xyz)   -  Change for lanty
            info_array_slam[i,0:3] = [xyz_trans_rot[0], xyz_trans_rot[1], xyz_trans_rot[2]]

        path_out_info_npy_slam = os.path.join(path_out, name + "_info_slam.npy")
        np.save(path_out_info_npy_slam, info_array_slam)  


file_tq.close()



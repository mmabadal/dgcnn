
import copy
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def quaternion_multiply(q0, q1):
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return np.array([-x1*x0-y1*y0-z1*z0+w1*w0, x1*w0+y1*z0-z1*y0+w1*x0, -x1*z0+y1*w0+z1*x0+w1*y0, x1*y0-y1*x0+z1*w0+w1*z0], dtype=np.float64)

    
def get_tr(t, q):
    trans = np.array([[1, 0, 0, t[0]], [0, 1, 0, t[1]], [0, 0, 1, t[2]], [0, 0, 0, 1]], np.float)
    rot = Rot.from_quat(q)
    rot_mat = rot.as_matrix()
    trans_rot = copy.deepcopy(trans)
    trans_rot[0:3, 0:3] = rot_mat
    return(trans_rot)


    
if __name__=='__main__':

    # with open("graph.txt", "r") as file:
    #    tq_ned_baselink = file.readlines()[-1]

    # tq_ned_baselink_f = [float(x) for x in tq_ned_baselink.split()]
    
    tq_ned_baselink_f  = np.array([1614260512.032102823, 0, 74.567500000, 19.919000000, 2.208750000, 0.021034106, -0.075268223, 0.565774172, 0.820848249])
    t_ned_baselink = tq_ned_baselink_f[2:5]
    q_ned_baselink = tq_ned_baselink_f[5:]

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

    print(tr_ned_left)
    o = np.ones((4, 1))
    o_ned_left = np.matmul(tr_ned_left, o)
    print(o_ned_left)


    print("-------------------")
    print("-------------------")

    o = np.ones((4, 1))
    o_ned_baselink = np.matmul(tr_ned_baselink, o)
    o_ned_stick = np.matmul(tr_baselink_stick, o_ned_baselink)
    o_ned_downbase = np.matmul(tr_stick_downbase, o_ned_stick)
    o_ned_down = np.matmul(tr_downbase_down, o_ned_downbase)
    o_ned_left = np.matmul(tr_down_left, o_ned_down)
    print(o_ned_left)

    print("-------------------")
    print("-------------------")

    # t_ned_left = t_ned_baselink + t_baselink_stick + t_stick_downbase + t_downbase_down + t_down_left

    # q_ned_stick = quaternion_multiply(q_ned_baselink, q_baselink_stick)
    # q_ned_downbase = quaternion_multiply(q_ned_stick, q_stick_downbase)
    # q_ned_down = quaternion_multiply(q_ned_downbase, q_downbase_down)
    # q_ned_left = quaternion_multiply(q_ned_down, q_down_left)

    # tr_down_left = get_tr(t_ned_left, q_ned_left)
    
    # print(tr_down_left)

    # print("-------------------")
    # print("-------------------")






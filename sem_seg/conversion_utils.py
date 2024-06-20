import numpy as np
from natsort import natsorted


def info_to_ply(info, path_out):

    info_pipes_list = info[0]
    info_connexions_list = info[1]
    info_valves_list = info[2]

    pipe_ply = list()       #  X Y Z R G B A 
    startend_ply = list()   #  X Y Z R G B A 
    elbow_ply = list()      #  X Y Z R G B A 
    vector1_ply = list()    #  X Y Z R G B A 
    vector2_ply = list()    #  V1 V2
    connexion_ply = list()  #  X Y Z R G B A 
    valve_ply = list()      #  X Y Z R G B A 


    for i, pipe_info in enumerate(info_pipes_list):
        pipe_list = list(pipe_info[0])
        pipe_list.pop(0)
        pipe_list.pop(-1)
        pipe_ply = pipe_ply + pipe_list
        startend_ply.append(pipe_info[0][0])
        startend_ply.append(pipe_info[0][-1])
        elbow_ply = elbow_ply + pipe_info[1]

        if len(pipe_info[1]) == 0:
            point1 = pipe_info[0][0]
            point2 = pipe_info[0][0]+pipe_info[2][0]
            vector1_ply.append(point1)
            vector1_ply.append(point2)

        else:
            point1 = pipe_info[0][0]
            point2 = pipe_info[0][0]+pipe_info[2][0]
            vector1_ply.append(point1)
            vector1_ply.append(point2)
            
            for i, elbow in enumerate(pipe_info[1]):
                point1 = elbow
                point2 = elbow + pipe_info[2][i+1]
                vector1_ply.append(point1)
                vector1_ply.append(point2)

    
    for i, connexion_info in enumerate(info_connexions_list):
        connexion_ply.append(connexion_info[0])

    for i, valve_info in enumerate(info_valves_list):
        valve_ply.append(valve_info[0])

        point1 = valve_info[0]-(valve_info[1]/2)
        point2 = valve_info[0]+(valve_info[1]/2)
        vector1_ply.append(point1)
        vector1_ply.append(point2)

    pipe_ply_np = np.round(np.array(pipe_ply), 5)
    pipe_color = np.array([[0, 255, 0],]*pipe_ply_np.shape[0])
    pipe_ply_np_color = np.hstack((pipe_ply_np, pipe_color))   

    startend_ply_np = np.round(np.array(startend_ply), 5)
    startend_color = np.array([[0, 150, 0],]*startend_ply_np.shape[0])
    startend_ply_np_color = np.hstack((startend_ply_np, startend_color))  

    elbow_ply_np = np.round(np.array(elbow_ply), 2)
    elbow_color = np.array([[255, 0, 0],]*elbow_ply_np.shape[0])
    elbow_ply_np_color = np.hstack((elbow_ply_np, elbow_color))  

    connexion_ply_np= np.round(np.array(connexion_ply), 2)
    connexion_color = np.array([[0, 0, 0],]*connexion_ply_np.shape[0])
    connexion_ply_np_color = np.hstack((connexion_ply_np, connexion_color))  

    valve_ply_np = np.round(np.array(valve_ply), 5)
    valve_color = np.array([[0, 0, 255],]*valve_ply_np.shape[0])
    valve_ply_np_color = np.hstack((valve_ply_np, valve_color))  

    vector1_ply_np = np.round(np.array(vector1_ply), 5)
    vector1_color = np.array([[150, 150, 150],]*vector1_ply_np.shape[0])
    vector1_ply_np_color = np.hstack((vector1_ply_np, vector1_color))  

    pipe_ply = list(pipe_ply_np_color)     
    startend_ply = list(startend_ply_np_color) 
    elbow_ply = list(elbow_ply_np_color)    
    connexion_ply = list(connexion_ply_np_color)
    valve_ply = list(valve_ply_np_color)  
    vector1_ply = list(vector1_ply_np_color)   

    vertex = pipe_ply + startend_ply + elbow_ply + connexion_ply + valve_ply + vector1_ply
    vertex_np = np.array(vertex)

    disscount = vector1_ply_np.shape[0]-1
    last_idx = vertex_np.shape[0]-1
    for i in range(int(vector1_ply_np.shape[0]/2)):
        vector_idxs = np.array([last_idx-disscount,last_idx-disscount+1])
        vector2_ply.append(vector_idxs)
        disscount -=2
    vector2_ply_np = np.array(vector2_ply)

    f = open(path_out, 'w')

    f.write("ply" + '\n')
    f.write("format ascii 1.0" + '\n')
    f.write("comment VCGLIB generated" + '\n')
    f.write("element vertex " + str(vertex_np.shape[0]) + '\n')
    f.write("property float x" + '\n')
    f.write("property float y" + '\n')
    f.write("property float z" + '\n')
    f.write("property uchar red" + '\n')
    f.write("property uchar green" + '\n')
    f.write("property uchar blue" + '\n')
    f.write("element face 0" + '\n')
    f.write("property list uchar int vertex_indices" + '\n')
    f.write("element edge " + str(vector2_ply_np.shape[0]) + '\n')
    f.write("property int vertex1" + '\n')
    f.write("property int vertex2" + '\n')
    f.write("end_header" + '\n')

    for row in range(vertex_np.shape[0]):
        line = ' '.join(map(str, vertex_np[row, :-3])) + ' ' + str(int(vertex_np[row, 3]))+ ' ' + str(int(vertex_np[row, 4])) + ' ' + str(int(vertex_np[row, 5])) +'\n'
        f.write(line)
    for row in range(vector2_ply_np.shape[0]):
        line = str(int(vector2_ply_np[row, 0]))+ ' ' + str(int(vector2_ply_np[row, 1])) +'\n'
        f.write(line)
    f.close()

def info_to_array(info):

    info_pipes_list = info[0]
    info_connexions_list = info[1]
    info_valves_list = info[2]
    pipe_inst_list = info[3]

    inst = 0

    info_list = list()


    for i, pipe_info in enumerate(info_pipes_list):

        skeleton = pipe_info[0]
        pipe_color = np.array([[0, 255, 0],]*skeleton.shape[0])
        skeleton = np.hstack((skeleton, pipe_color))  
        skeleton = np.insert(skeleton, 6, values=0, axis=1) # insert type 0 - skeleton
        skeleton = np.insert(skeleton, 7, values=0, axis=1) # insert info 0 - nothing

        if len(pipe_info[1]) > 0:
            elbows = np.array(pipe_info[1])
            elbows = np.round(elbows, 2)
            elbow_color = np.array([[255, 0, 0],]*elbows.shape[0])
            elbows = np.hstack((elbows, elbow_color))  
            elbows = np.insert(elbows, 6, values=1, axis=1) # insert type 1 - elbow
            elbows = np.insert(elbows, 7, values=0, axis=1) # insert info 0 - nothing

        vector_list = list()
        vp1 = pipe_info[0][0]
        vp2 = pipe_info[0][0]+pipe_info[2][0]
        vector_list.append(vp1)
        vector_list.append(vp2)
        
        for i, elbow in enumerate(pipe_info[1]):
            vp1 = elbow
            vp2 = elbow + pipe_info[2][i+1]
            vector_list.append(vp1)
            vector_list.append(vp2)

        vectors = np.array(vector_list)
        vector_color = np.array([[127, 127, 127],]*vectors.shape[0])
        vectors = np.hstack((vectors, vector_color))  
        vectors = np.insert(vectors, 6, values=2, axis=1) # insert type 2 - vector
        vectors = np.insert(vectors, 7, values=0, axis=1) # insert info 0 - nothing


        belonging_insts_list = list()
        for i, belonging_inst_idx in enumerate(pipe_info[3]):
            belonging_inst = np.append(pipe_info[0][1], [0, 255, 0, 7, belonging_inst_idx])   # insert color, type 7 - belonging inst and info - belonging inst idx
            belonging_insts_list.append(belonging_inst)
        belonging_insts = np.array(belonging_insts_list)

        if len(pipe_info[1]) > 0:
            pipe = np.vstack((skeleton,elbows,vectors,belonging_insts))
        else:
            pipe = np.vstack((skeleton,vectors,belonging_insts))

        pipe = np.insert(pipe, 8, values=0, axis=1)     # insert class 0 - pipe
        pipe = np.insert(pipe, 9, values=inst, axis=1)  # insert inst

        info_list.append(pipe)
        inst += 1

    for i, pipe_inst in enumerate(pipe_inst_list):

        data = pipe_inst[:,0:3]
        inst_color = np.array([[0, 127, 0],]*data.shape[0])
        data = np.hstack((data, inst_color))  
        data = np.insert(data, 6, values=6, axis=1) # insert type 6 - inst data
        data = np.insert(data, 7, values=i, axis=1) # insert info i - instance number

        data = np.insert(data, 8, values=0, axis=1)     # insert class 0 - pipe
        data = np.insert(data, 9, values=inst, axis=1)  # insert inst

        info_list.append(data)
        inst += 1

    for i, valve_info in enumerate(info_valves_list):

        central = np.append(valve_info[0], [0, 0, 255, 3, 0])   # insert color, type 3 - central point and info 0 - nothing
        
        vp1 = valve_info[0]-(valve_info[1]/2)
        vp1 = np.append(vp1, [127,127,127, 2, 0])   # insert color, type 2 - vector and info 0 - nothing
        vp2 = valve_info[0]+(valve_info[1]/2)
        vp2 = np.append(vp2, [127,127,127, 2, 0])   # insert color, type 2 - vector and info 0 - nothing

        max_id = np.append(valve_info[0], [0, 0, 255, 5, valve_info[2]])   # insert color, type 5 - max_id and info - max id

        if len(valve_info[3]) > 0:
            near_pipes_list = list()
            for i, near_pipe_idx in enumerate(valve_info[3]):
                near_pipe = np.append(valve_info[0], [0, 0, 255, 4, near_pipe_idx])   # insert color, type 4 - near pipe and info - near_pipe_idx
                near_pipes_list.append(near_pipe)
            near_pipes = np.array(near_pipes_list)

            valve = np.vstack((central,vp1,vp2,max_id,near_pipes))

        else:
            valve = np.vstack((central,vp1,vp2,max_id))

        valve = np.insert(valve, 8, values=1, axis=1)     # insert class 1 - valve
        valve = np.insert(valve, 9, values=inst, axis=1)  # insert inst

        info_list.append(valve)
        inst += 1

    for i, connexion_info in enumerate(info_connexions_list):

        central = np.append(connexion_info[0], [0, 0, 0, 3, 0])   # insert color, type 3 - central point and info - nothing
        
        near_pipes_list = list()
        for i, near_pipe_idx in enumerate(connexion_info[1]):
            near_pipe = np.append(connexion_info[0], [0, 0, 0, 4, near_pipe_idx])   # insert color, type 4 - near pipe and info - near_pipe_idx
            near_pipes_list.append(near_pipe)
        near_pipes = np.array(near_pipes_list)

        if near_pipes.size == 0:
            near_pipes = np.array([99, 99, 99])

        connexion = np.vstack((central,near_pipes))

        connexion = np.insert(connexion, 8, values=2, axis=1)     # insert class 2 - connexion
        connexion = np.insert(connexion, 9, values=inst, axis=1)  # insert inst
        info_list.append(connexion)
        inst += 1

    info_array = np.array(info_list)
    info_array = np.vstack(info_array)

    return info_array


def array_to_info(pc_np_info):

    if pc_np_info.shape[1] == 10:               # delete R G B info
        pc_np_info = np.delete(pc_np_info, [3,4,5], 1)
        
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

    return info_pipes_list, info_connexions_list, info_valves_list, info_inst_pipe_list

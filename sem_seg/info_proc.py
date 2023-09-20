import os
import copy
import ros_numpy
import numpy as np
from PIL import Image
from skimage import io, color
from dgcnn.msg import info_bbs
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point32, Polygon


def points_to_img(box_list, id, c_info, path):

    decimation = c_info.binning_x

    P = c_info.P 

    height = c_info.height/decimation
    width = c_info.width/decimation
    
    P_array = np.array([[P[0]/decimation, P[1]/decimation, P[2]/decimation, P[3]/decimation],
                       [P[4]/decimation, P[5]/decimation, P[6]/decimation, P[7]/decimation],
                       [P[8],            P[9],            P[10],           P[11]]])

     # 1988    0     971         x        u  =  1988*x + 971*z 
     #   0    1988   714   *     y    =   v  =  1988*y + 714*z 
     #   0     0      1          z        w  =  z 

    box_list_2d = list()

    # keyframes = os.listdir(path)
    # for keyframe in keyframes:
    #     if "left" in keyframe:
    #         if id in keyframe:
    #             key = Image.open(os.path.join(path, keyframe))
    #             break

    for box in box_list:

        box2 = list()

        for point in box:

            xyz = np.array([[point[0]],
                           [point[1]],
                           [point[2]],
                           [1]])

            uvw = np.matmul(P_array, xyz)

            u = uvw[0][0]
            v = uvw[1][0]
            w = uvw[2][0]

            xdisp = int(u/w) # = (1988*x + 971*z) / z
            ydisp = int(v/w) # = (1988*y + 714*z) / z

            xdisp = int(np.clip(xdisp, 1, width-1))
            ydisp = int(np.clip(ydisp, 1, height-1))

            # key.putpixel((xdisp, ydisp), (255,0,0))

            xydisp = np.array((ydisp, xdisp))
            box2.append(xydisp)

        box_list_2d.append(box2)

    # key.save("/home/bomiquel/Desktop/" + str(id) + "_colour.png")

    return box_list_2d

def get_bb(info, pointcloud, margin, id, c_info, path, ):

    infobbs = info_bbs()
    p1 = Point32()
    p2 = Point32()
    p3 = Point32()
    p4 = Point32()
    # p5 = Point32()
    # p6 = Point32()
    polygon = Polygon()

    info_pipes_list = info[0]
    info_valves_list = info[2]

    box_list = list()

    for pipe_info in info_pipes_list:  # TODO controlar que al otener 3 4 5 6 y añadir margin a puntos 1 2 3 4 5 6 estos no caigan despues fuera en img, (comprovar que no queden por encima o debajo de maxx minx maxyminy)

        chain = pipe_info[0]
        elbow_list = pipe_info[1]
        vector_list = pipe_info[2]

        vector = vector_list[0][0:3]

        point1 = chain[0][0:3]
        center = point1 + vector/2

        vector_orth = np.array([-vector[1], vector[0], 0])
        vector_orth = vector_orth/np.linalg.norm(vector_orth)
        vector_orth = (0.06+margin) * vector_orth

        vector_margin = vector/np.linalg.norm(vector)
        vector_margin = margin * 2 * vector_margin
        vector_margin = vector + vector_margin

        point1_margin = center - (vector_margin/2)
        point2_margin = center + (vector_margin/2)
        point3_margin = point1_margin + ((vector_orth/2))
        point4_margin = point1_margin - ((vector_orth/2))
        point5_margin = point2_margin + ((vector_orth/2))
        point6_margin = point2_margin - ((vector_orth/2))

        box = [point3_margin, point4_margin, point5_margin, point6_margin, point1_margin, point2_margin]
        box_list.append(box)

        for i, elbow in enumerate(elbow_list):
            
            vector = vector_list[i+1][0:3]

            point1 = elbow[0:3]
            center = point1 + vector/2

            vector_orth = np.array([-vector[1], vector[0],0])
            vector_orth = vector_orth/np.linalg.norm(vector_orth)
            vector_orth = (0.06+margin) * vector_orth

            vector_margin = vector/np.linalg.norm(vector)
            vector_margin = margin * 2 * vector_margin
            vector_margin = vector + vector_margin

            point1_margin = center - (vector_margin/2)
            point2_margin = center + (vector_margin/2)
            point3_margin = point1_margin + ((vector_orth/2))
            point4_margin = point1_margin - ((vector_orth/2))
            point5_margin = point2_margin + ((vector_orth/2))
            point6_margin = point2_margin - ((vector_orth/2))

            box = [point3_margin, point4_margin, point5_margin, point6_margin, point1_margin, point2_margin]
            box_list.append(box)

    for valve_info in info_valves_list:

        center = valve_info[0][0:3]
        vector = valve_info[1][0:3]
        
        point1 = center - (vector/2)

        vector_orth = np.array([-vector[1], vector[0],0])
        vector_orth = vector_orth/np.linalg.norm(vector_orth)
        vector_orth = (0.08+margin) * vector_orth

        vector_margin = vector/np.linalg.norm(vector)
        vector_margin = margin * 2 * vector_margin
        vector_margin = vector + vector_margin

        point1_margin = center - (vector_margin/2)
        point2_margin = center + (vector_margin/2)
        point3_margin = point1_margin + ((vector_orth/2))
        point4_margin = point1_margin - ((vector_orth/2))
        point5_margin = point2_margin + ((vector_orth/2))
        point6_margin = point2_margin - ((vector_orth/2))

        box = [point3_margin, point4_margin, point5_margin, point6_margin, point1_margin, point2_margin]
        box_list.append(box)
    
    box_list_2d = points_to_img(box_list, id, c_info, path)

    pc_xmin, pc_ymin, *_ = pointcloud.min(axis=0)
    pc_xmax, pc_ymax, *_ = pointcloud.max(axis=0)

    minmaxs_pc = [[np.array([pc_xmin,pc_ymin]),np.array([pc_xmax,pc_ymax])]]
    minmaxs_2d = points_to_img(minmaxs_pc, id, c_info, path)

    minmaxs = np.array([minmaxs_2d[0][0], minmaxs_2d[0][1], minmaxs_2d[1][0], minmaxs_2d[1][1]])

    box_list_2d_expanded = expand_polygons(box_list_2d, minmaxs)

    for i, box in enumerate(box_list_2d_expanded):

        p1.x = box[0][0]
        p1.y = box[0][1]
        p1.z = 0
        p2.x = box[1][0]
        p2.y = box[1][1]
        p2.z = 0
        p3.x = box[2][0]
        p3.y = box[2][1]
        p3.z = 0
        p4.x = box[3][0]
        p4.y = box[3][1]
        p4.z = 0
        # p5.x = box[4][0]
        # p5.y = box[4][1]
        # p5.z = 0
        # p6.x = box[5][0]
        # p6.y = box[5][1]
        # p6.z = 0

        polygon.points.append(p1)
        polygon.points.append(p2)
        polygon.points.append(p3)
        polygon.points.append(p4)
        # polygon.points.append(p5)
        # polygon.points.append(p6)

        polygon2 = copy.deepcopy(polygon)
        infobbs.bbs.append(polygon2)
        
        polygon.points.clear()

    return infobbs


def expand_polygons(box_list, minmaxs, c_info):

    new_box_list = list()

    decimation = c_info.binning_x
    height = c_info.height/decimation
    width = c_info.width/decimation

    margin = 25
    dist = 10
    cthr = 0
    nthr = 15
    vstride = 4

    for box in box_list:

        border = check_box(box, minmaxs, margin) # TODO buscar minmaxs a partir de los max y min de la pointcloud pasados a coordenadas img
        if border == False:                       
            next()

        vector56 = box[5]-box[4]
        vector56_unit = vector56/np.linalg.norm(vector56)
        vector65_unit = vector56_unit*-1

        vector56_iter = vector56_unit * vstride
        vector65_iter = vector65_unit * vstride


        iter = 0
        p_list = list()
        while 1:
            iter += 1
            point = int(box[5] + iter * vector56_iter)
            p_list.append(point)
            if point[0] < 0 or point[0] > height or point[1] < 0 or point[1] > width:
                p_end1 = p_list[-2] # el ultimo que tuvo tuberia antes de salirse
                break
            else:
                end = check_near(point, dist, img, cthr, nthr)
                if end == True:
                    a = -1 - dist/vstride #  -1 - dist/vstride para tirar para atras los puntos que añadirá de mas al ir encontrando tuberia por atras (/vstride pq vamos a saltos de vstride pixeles)
                    p_end1 = p_list[a]  
                    break

        iter = 0
        p_list = list()
        while 1:
            iter += 1
            point = int(box[4] + iter * vector65_iter)
            p_list.append(point)
            if point[0] < 0 or point[0] > height or point[1] < 0 or point[1] > width:
                p_end2 = p_list[-2] # el ultimo que tuvo tuberia antes de salirse
                break
            else:
                end = check_near(point, dist, img, cthr, nthr)
                if end == True:
                    a = -1 - dist/vstride #  -1 - dist/vstride para tirar para atras los puntos que añadirá de mas al ir encontrando tuberia por atras (/vstride pq vamos a saltos de vstride pixeles)
                    p_end2 = p_list[a]  
                    break
   
    vector_orth = box[2]-box[1]
    new_p3 = p_end1 + ((vector_orth/2))
    new_p4 = p_end1 - ((vector_orth/2))
    new_p5 = p_end2 + ((vector_orth/2))
    new_p6 = p_end2 - ((vector_orth/2))
    new_box = (new_p3, new_p4, new_p5, new_p6, p_end1, p_end2)
    new_box_list.append(new_box)


# añadir nuevas iteraciones para vectores ortogonales que se haran a partir de los puntos end y que sera +- vector unitario ortogonal a los vectores unit o directamente iter y 
# sera en loop hasta que no se encuentren near, tener en cuuenta que no se vaya a infinito ya que al principio detectara la propia tuberia, se puede anular el principio, hacer
# un salto grande al principio ... pero al ser sobre 2d no tenemos tamaños, lo cual lo dificulta.


def check_box(box, minmaxs, margin):
    border = False

    minx, miny, maxx, maxy = minmaxs

    for point in box:
        if (point[0] < minx+margin) or (point[0] > maxx[0]-margin) or (point[1] < miny+margin) or (point[1] > maxy[1]-margin):
            border = True
            break
    return border


def check_near(point, dist, img, cthr, nthr):

    color_ref_rgb = np.array([210,210,0])
    color_ref_lab = color.rgb2lab([[[color_ref_rgb[0] / 255, color_ref_rgb[1] / 255, color_ref_rgb[2] / 255]]])
    end = True 
    imshape = img.shape

    row0 = max(point[0]-dist,0)
    row1 = min(point[0]+dist+1, imshape[0])
    col0 = max(point[1]-dist,0)
    col1 = min(point[1]+dist+1, imshape[1])

    n = 0

    for row in range(row0, row1):                               # for each row
        for col in range(col0, col1):                           # for each col
            pixel = np.array([img[row,col,0],img[row,col,1],img[row,col,2]])
            pixel_lab = color.rgb2lab([[[pixel[0] / 255, pixel[1] / 255, pixel[2] / 255]]])
            color_dist = color.deltaE_cie76(color_ref_lab, pixel_lab, channel_axis=-1)
            if color_dist < cthr:
                n += 1 
    if n > nthr:
        end = False
    return end
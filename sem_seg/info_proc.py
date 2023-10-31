import os
import copy
import ros_numpy
import numpy as np
from PIL import Image
from skimage import io, color
from dgcnn.msg import info_bbs
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point32, Polygon


def points_to_img(points_list, c_info):

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

    points_list_2d = list()

    for points in points_list:

        points2 = list()

        for point in points:

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

            xydisp = np.array((xdisp, ydisp))
            points2.append(xydisp)

        points_list_2d.append(points2)

    return points_list_2d


def get_bb(info, pointcloud, margin, id, img, disp_msg, c_info):

    infobbs = info_bbs()
    p = Point32()
    polygon_ros = Polygon()

    info_pipes_list = info[0]
    info_valves_list = info[2]
    inst_pipe_list = info[3]
    
    expand_list = list()

    for pipe_info in info_pipes_list: 

        chain = pipe_info[0]
        chain_list = [row for row in chain]

        elbow_list = pipe_info[1]
        vector_list = pipe_info[2]

        vector = vector_list[0][0:3]
        point1 = chain_list[0][0:3]
        center = point1 + (vector/2)
        point2 = center + (vector/2)

        vector_orth = np.array([-vector[1], vector[0], 0])
        vector_orth = vector_orth/np.linalg.norm(vector_orth)
        vector_orth = (0.06+margin) * vector_orth

        point3 = point1 + vector_orth
        
        expand = [point1, point2, point3]
        expand_list.append(expand)

        for i, elbow in enumerate(elbow_list):
            
            vector = vector_list[i+1][0:3]

            point1 = elbow[0:3]
            center = point1 + vector/2
            point2 = center + (vector/2)

            vector_orth = np.array([-vector[1], vector[0],0])
            vector_orth = vector_orth/np.linalg.norm(vector_orth)
            vector_orth = (0.06+margin) * vector_orth
            point3 = point1 + vector_orth

            expand = [point1, point2, point3]
            expand_list.append(expand)

    for valve_info in info_valves_list:

        center = valve_info[0][0:3]
        vector = valve_info[1][0:3]
        
        point1 = center - (vector/2)
        point2 = center + (vector/2)

        vector_orth = np.array([-vector[1], vector[0],0])
        vector_orth = vector_orth/np.linalg.norm(vector_orth)
        vector_orth = (0.08+margin) * vector_orth
        point3 = point1 + vector_orth

        expand = [point1, point2, point3]
        expand_list.append(expand)
    
    expand_list_2d = points_to_img(expand_list, c_info)

    for expand_2d in expand_list_2d:
        expand_2d[2] = expand_2d[2] - expand_2d[0]

    # pc_ymin, pc_xmin, *_ = pointcloud.min(axis=0)
    # pc_ymax, pc_xmax, *_ = pointcloud.max(axis=0)
    # minmaxs_pc = [[np.array([pc_xmin, pc_ymin, 1]), np.array([pc_xmax, pc_ymax, 1])]]
    # minmaxs_2d = points_to_img(minmaxs_pc, c_info)
    # minmaxs2 = np.array([minmaxs_2d[0][0][0], minmaxs_2d[0][0][1], minmaxs_2d[0][1][0], minmaxs_2d[0][1][1]])

    disp = ros_numpy.numpify(disp_msg.image)
    disp_pos = np.where(disp>15)
    disp_pos_np = np.vstack(disp_pos).T
    disp_ymin, disp_xmin = disp_pos_np.min(axis=0)
    disp_ymax, disp_xmax = disp_pos_np.max(axis=0)
    minmaxs = np.array([disp_xmin, disp_ymin, disp_xmax, disp_ymax])

    polygon_list = create_polygons(expand_list_2d, minmaxs, img, c_info)

    colors = ((255,0,0,),(0,255,0),(0,0,255))
    lp= len(expand_list_2d)


    img_pil = Image.fromarray(img) 

    for i, polygon in enumerate(polygon_list):
        k = int(i/lp)
        for point in polygon:
            img_pil.putpixel((point[0], point[1]), colors[k])
    img_pil.save("../out/img/" + str(id) + "_colour.png")

    for box in polygon_list:
        for point in enumerate(box):
            p.x = point[0]
            p.y = point[1]
            p.z = 0
            p2 = copy.deepcopy(p)
            polygon_ros.points.append(p2)

        polygon_ros_2 = copy.deepcopy(polygon_ros)
        infobbs.bbs.append(polygon_ros_2)

        polygon_ros.points.clear()

    return infobbs

def create_polygons(expand_list, minmaxs, img, c_info):

    box_list = list()

    decimation = c_info.binning_x
    height = c_info.height/decimation
    width = c_info.width/decimation
    imshape = np.array([height, width])

    margin = 25
    dist = 5
    cthr = 40
    nthr = 50
    vstride = 5

    for expand in expand_list:

        border = check_expand(expand, minmaxs, margin) 

        if border == True:  
            #print("---- expand accepted ----")
            col = get_color(expand, img)

            vector1 = expand[1]-expand[0]
            vector1_unit = vector1/np.linalg.norm(vector1)
            vector2_unit = vector1_unit*-1

            vector1_iter = vector1_unit * vstride
            vector2_iter = vector2_unit * vstride

            iter = 0
            p_list = list()
            p_list.append(expand[1])
            while 1:
                iter += 1
                point = (expand[1] + iter * vector1_iter).astype(int)
                p_list.append(point)
                if point[0] < 0 or point[0] > imshape[0] or point[1] < 0 or point[1] > imshape[1]:
                    p_end1 = p_list[-2] # el ultimo que tuvo tuberia antes de salirse
                    #print("out")
                    break
                else:
                    end = check_near(point, col, dist, img, cthr, nthr)
                    if end == True:
                        #print("col check fail")
                        p_end1 = p_list[-2]
                        break
                    #print("col check ok")

            iter = 0
            p_list = list()
            p_list.append(expand[0])
            while 1:
                iter += 1
                point = (expand[0] + iter * vector2_iter).astype(int)
                p_list.append(point)
                if point[0] < 0 or point[0] > imshape[0] or point[1] < 0 or point[1] > imshape[1]:
                    p_end2 = p_list[-2] # el ultimo que tuvo tuberia antes de salirse
                    #print("out")
                    break
                else:
                    end = check_near(point, col, dist, img, cthr, nthr)
                    if end == True:
                        #print("col check fail")
                        p_end2 = p_list[-2]
                        break
                    #print("col check ok")

        else:
            #print("---- expand discarted ----")  
            #print(minmaxs)
            #print(expand)
            p_end1 = expand[1]
            p_end2 = expand[0]

        vector_orth = expand[2]
        p1 = (p_end1 + ((vector_orth/2))).astype(int)
        p2 = (p_end1 - ((vector_orth/2))).astype(int)
        p3 = (p_end2 + ((vector_orth/2))).astype(int)
        p4 = (p_end2 - ((vector_orth/2))).astype(int)
        box = (p1, p2, p3, p4)
        box_list.append(box)
    
    polygon_list = box_to_polygon(box_list, imshape)

    return polygon_list


def box_to_polygon(box_list, imshape):

    polygon_list = list()

    for box in box_list:

        polygon = list()
        
        w = int(imshape[0])
        h = int(imshape[1])

        for n, point in enumerate(box):
            if point[0] in range (0,h) and point[1] in range(0,w):
                polygon.append(point)
            else:
                previous = n-1
                following = (n+1) % len(box)

                v1 = box[following] - box[n]
                v2 = box[previous] - box[n]

                for i in range(1,100):
                    p = (point + (v1 * (i/100))).astype(int)
                    if p[0] in range(0,h) and p[1] in range(0,w):
                        polygon.append(p)
                        break
                for i in range(1,100):
                    p = (point + (v2 * (i/100))).astype(int)
                    if p[0] in range(0,h) and p[1] in range(0,w):
                        polygon.append(p)
                        break

        for i, a in enumerate([0,h]):
            for j, b in enumerate([0,w]):
                corner = np.array([a,b])
                inside = is_inside(corner, box)
                if inside == True:
                    polygon.append(corner)

        polygon_list.append(polygon)

    return polygon_list


def is_left(p, p1, p2):
    return (p2[0] - p1[0]) * (p[1] - p1[1]) - (p[0] - p1[0]) * (p2[1] - p1[1])


def is_inside(p, box):
    p1 = box[0]
    p2 = box[1]
    p3 = box[2]
    p4 = box[3]

    left1 = is_left(p, p1, p2)
    left2 = is_left(p, p2, p3)
    left3 = is_left(p, p3, p4)
    left4 = is_left(p, p4, p1)

    if ((left1 > 0 and left2 > 0 and left3 > 0 and left4 > 0) or
            (left1 < 0 and left2 < 0 and left3 < 0 and left4 < 0)):
        return True
    else:
        return False


def check_expand(expand, minmaxs, margin):
    border = False
    minx, miny, maxx, maxy = minmaxs
    for point in expand[:-1]:
        if (point[0] < minx+margin) or (point[0] > maxx-margin) or (point[1] < miny+margin) or (point[1] > maxy-margin):
            border = True
            break
    return border


def check_near(point, col, dist, img, cthr, nthr):
    color_rgb = col
    color_lab = color.rgb2lab([[[color_rgb[0] / 255, color_rgb[1] / 255, color_rgb[2] / 255]]])
    end = True 
    imshape = img.shape

    row0 = max(point[1]-dist,0)
    row1 = min(point[1]+dist+1, imshape[0])
    col0 = max(point[0]-dist,0)
    col1 = min(point[0]+dist+1, imshape[1])

    n = 0

    for row in range(row0, row1):                               # for each row
        for col in range(col0, col1):                           # for each col
            pixel = np.array([img[row,col,0],img[row,col,1],img[row,col,2]])
            pixel_lab = color.rgb2lab([[[pixel[0] / 255, pixel[1] / 255, pixel[2] / 255]]])
            color_dist = color.deltaE_cie76(color_lab, pixel_lab)
            if color_dist < cthr:
                n += 1 
    if n > nthr:
        end = False
        print("col check")
    return end


def get_color(expand, img):

    samples = 10
    p1 = expand[0]
    p2 = expand[1]
    v12 = p2-p1 

    # TODO pasar instance pasada a disp, coger puntos que han pasado el check, añadirlos a la instance, volver a calcular vector ....

    pixel_col_list = list()

    for i in range(samples+1):
        pixel = (p1+((v12/samples)*i)).astype(int)
        pixel_col = np.array([img[pixel[1],pixel[0],0],img[pixel[1],pixel[0],1],img[pixel[1],pixel[0],2]])
        pixel_col_list.append(pixel_col)

    color_stack = np.stack(pixel_col_list, axis=0)
    color = (np.mean(color_stack, axis=0)).astype(int)

    return color


def points_within_distance(points_array, target_points, distance):
    result = []
    
    for point in points_array:
        for target_point in target_points:
            # Calculate the Euclidean distance between the current point and the target point.
            dist = np.linalg.norm(point - target_point)
            
            if dist <= distance:
                result.append(point)
                break  # Break the inner loop when a match is found
    
    return np.array(result)
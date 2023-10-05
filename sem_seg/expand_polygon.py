import numpy as np
from skimage import io, color


def main():

    expand_list = list()
    box_list = list()

    img = io.imread("/home/miguel/dgcnn/sem_seg/left.jpeg")
    img2 = io.imread("/home/miguel/dgcnn/sem_seg/left.jpeg")
    img3 = io.imread("/home/miguel/dgcnn/sem_seg/left.jpeg")
    imshape = img.shape

    margin = 25
    dist = 5
    cthr = 80
    nthr = 50
    vstride = 5
    minmaxs = np.array([120,100,400,600])

    p1 = np.array([253, 116])
    p2 = np.array([468, 83 ])
    vector_orth = np.array([269, 168]) - np.array([249, 49 ])
    expand = (p1, p2, vector_orth)
    expand_list.append(expand)

    p1 = np.array([252, 110])
    p2 = np.array([147, 129])
    vector_orth = np.array([151, 155]) - np.array([137, 101])
    expand = (p1, p2, vector_orth)
    expand_list.append(expand)

    p1 = np.array([612, 477])
    p2 = np.array([693, 535])
    vector_orth =  np.array([591, 505]) - np.array([630, 445])
    expand = (p1, p2, vector_orth)
    expand_list.append(expand)

    colors = ((255,0,0,),(0,255,0),(0,0,255))


    for c, expand in enumerate(expand_list):
        for i, point in enumerate(expand):
            if i < 2:
                img[point[0], point[1], 0] = colors[c][0]
                img[point[0], point[1], 1] = colors[c][1]
                img[point[0], point[1], 2] = colors[c][2]

    for expand in expand_list:

        border = check_box(expand, minmaxs, margin) 
        if border == False:                       
            #next() # TODO test
            a = 1

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
                break
            else:
                end = check_near(point, dist, img, cthr, nthr)
                if end == True:
                    a = int(-1 - dist/vstride) #  -1 - dist/vstride para tirar para atras los puntos que añadirá de mas al ir encontrando tuberia por atras (/vstride pq vamos a saltos de vstride pixeles)
                    p_end1 = p_list[-1] # TODO Change to a
                    break

        iter = 0
        p_list = list()
        p_list.append(expand[0])
        while 1:
            iter += 1
            point = (expand[0] + iter * vector2_iter).astype(int)
            p_list.append(point)
            if point[0] < 0 or point[0] > imshape[0] or point[1] < 0 or point[1] > imshape[1]:
                p_end2 = p_list[-2] # el ultimo que tuvo tuberia antes de salirse
                break
            else:
                end = check_near(point, dist, img, cthr, nthr)
                if end == True:
                    a = int(-1 - dist/vstride) #  -1 - dist/vstride para tirar para atras los puntos que añadirá de mas al ir encontrando tuberia por atras (/vstride pq vamos a saltos de vstride pixeles)
                    p_end2 = p_list[-1]  # TODO Change to a
                    break

        vector_orth = expand[2]
        p1 = (p_end1 + ((vector_orth/2))).astype(int)
        p2 = (p_end1 - ((vector_orth/2))).astype(int)
        p3 = (p_end2 + ((vector_orth/2))).astype(int)
        p4 = (p_end2 - ((vector_orth/2))).astype(int)
        box = (p1, p2, p3, p4)
        box_list.append(box)

    polygon_list = box_to_polygon(box_list, imshape)
    
    for c, box in enumerate(box_list):
        for i, point in enumerate(box):

            px = np.clip(point[0], 0, imshape[0]-1)
            py = np.clip(point[1], 0, imshape[1]-1)

            img2[px, py, 0] = colors[c][0]
            img2[px, py, 1] = colors[c][1]
            img2[px, py, 2] = colors[c][2]


    for c, polygon in enumerate(polygon_list):
        for i, point in enumerate(polygon):
            img3[point[0], point[1], 0] = colors[c][0]
            img3[point[0], point[1], 1] = colors[c][1]
            img3[point[0], point[1], 2] = colors[c][2]

    io.imshow(img)
    io.show()
    io.imsave('/home/miguel/Desktop/1.png', img)

    io.imshow(img2)
    io.show()
    io.imsave('/home/miguel/Desktop/2.png', img2)

    io.imshow(img3)
    io.show()
    io.imsave('/home/miguel/Desktop/3.png', img3)


def box_to_polygon(box_list, imshape):

    polygon_list = list()

    for box in box_list:

        polygon = list()
        
        h = int(imshape[0])
        w = int(imshape[1])

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


def check_box(box, minmaxs, margin):
    border = False
    minx, miny, maxx, maxy = minmaxs
    for point in box:
        if (point[0] < minx+margin) or (point[0] > maxx-margin) or (point[1] < miny+margin) or (point[1] > maxy-margin):
            border = True
            break
    return border


def check_near(point, dist, img, cthr, nthr):
    color_ref_rgb = np.array([210,210,0])   # TODO dinamico?
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
            color_dist = color.deltaE_cie76(color_ref_lab, pixel_lab)
            if color_dist < cthr:
                n += 1 
    if n > nthr:
        end = False
    return end


if __name__ == "__main__":
    main()


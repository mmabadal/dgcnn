import numpy as np
from skimage import io, color


def main():

    box_list = list()
    new_box_list = list()

    img = io.imread("/home/miguel/dgcnn/sem_seg/left.jpeg")
    img2 = io.imread("/home/miguel/dgcnn/sem_seg/left.jpeg")
    imshape = img.shape

    margin = 25
    dist = 5
    cthr = 80
    nthr = 50
    vstride = 10
    minmaxs = np.array([120,100,400,600])


    p1 = np.array([448, 26 ])
    p2 = np.array([472, 139])
    p3 = np.array([249, 49 ])
    p4 = np.array([269, 168])
    p5 = np.array([253, 116])
    p6 = np.array([468, 83 ])
    box = (p1, p2, p3, p4, p5, p6)
    box_list.append(box)

    p1 = np.array([246, 81 ])
    p2 = np.array([258, 142])
    p3 = np.array([137, 101])
    p4 = np.array([151, 155])
    p5 = np.array([252, 110])
    p6 = np.array([147, 129])
    box = (p1, p2, p3, p4, p5, p6)
    box_list.append(box)

    p1 = np.array([630, 445])
    p2 = np.array([591, 505])
    p3 = np.array([713, 504])
    p4 = np.array([674, 569])
    p5 = np.array([612, 477])
    p6 = np.array([693, 535])
    box = (p1, p2, p3, p4, p5, p6)
    box_list.append(box)


    for box in box_list:
        for i, point in enumerate(box):
            img[point[0], point[1], 0] = 0
            img[point[0], point[1], 1] = 255
            img[point[0], point[1], 2] = 0
            if i > 3:
                img[point[0], point[1], 0] = 255
                img[point[0], point[1], 1] = 0
                img[point[0], point[1], 2] = 0


    for box in box_list:

        border = check_box(box, minmaxs, margin) # TODO buscar minmaxs a partir de los max y min de la pointcloud pasados a coordenadas img
        if border == False:                       
            #next()
            a = 1

        vector56 = box[5]-box[4]
        vector56_unit = vector56/np.linalg.norm(vector56)
        vector65_unit = vector56_unit*-1

        vector56_iter = vector56_unit * vstride
        vector65_iter = vector65_unit * vstride


        iter = 0
        p_list = list()
        while 1:
            iter += 1
            point = (box[5] + iter * vector56_iter).astype(int)
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
        while 1:
            iter += 1
            point = (box[4] + iter * vector65_iter).astype(int)
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

        vector_orth = box[2]-box[1]
        new_p1 = (p_end1 + ((vector_orth/2))).astype(int)
        new_p2 = (p_end1 - ((vector_orth/2))).astype(int)
        new_p3 = (p_end2 + ((vector_orth/2))).astype(int)
        new_p4 = (p_end2 - ((vector_orth/2))).astype(int)
        new_box = (p_end1, p_end1, p_end1, p_end1, p_end1, p_end2) # TODO new_box = (new_p1, new_p2, new_p3, new_p4, p_end1, p_end2) # TODO check new_ps que caigan fuera y proyectar detro 
        new_box_list.append(new_box)

    for box in new_box_list:
        for i, point in enumerate(box):
            img2[point[0], point[1], 0] = 0
            img2[point[0], point[1], 1] = 255
            img2[point[0], point[1], 2] = 0
            if i > 3:
                img2[point[0], point[1], 0] = 255
                img2[point[0], point[1], 1] = 0
                img2[point[0], point[1], 2] = 0


    io.imshow(img)
    io.show()

    io.imshow(img2)
    io.show()

# añadir nuevas iteraciones para vectores ortogonales que se haran a partir de los puntos end y que sera +- vector unitario ortogonal a los vectores unit o directamente iter y 
# sera en loop hasta que no se encuentren near, tener en cuuenta que no se vaya a infinito ya que al principio detectara la propia tuberia, se puede anular el principio, hacer
# un salto grande al principio ... pero al ser sobre 2d no tenemos tamaños, lo cual lo dificulta.


def check_box(box, minmaxs, margin):
    border = False

    minx, miny, maxx, maxy = minmaxs

    for point in box:
        if (point[0] < minx+margin) or (point[0] > maxx-margin) or (point[1] < miny+margin) or (point[1] > maxy-margin):
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
            color_dist = color.deltaE_cie76(color_ref_lab, pixel_lab)
            if color_dist < cthr:
                n += 1 
    if n > nthr:
        end = False
    return end




if __name__ == "__main__":
    main()


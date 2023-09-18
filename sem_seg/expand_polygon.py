



import numpy as np
from skimage import io



im = io.imread("/home/miguel/Desktop/test_polygon/left.jpeg")
box_list = list()


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
        im[point[0], point[1], 0] = 0
        im[point[0], point[1], 1] = 255
        im[point[0], point[1], 2] = 0
        if i > 3:
            im[point[0], point[1], 0] = 255
            im[point[0], point[1], 1] = 0
            im[point[0], point[1], 2] = 0




for box in box_list:

    border = check_box(box, imshape, margin) # TODO en realidad se le tendra que pasar un xmax xmin ymax ymin i ver si estan cerca del borde de la disparidad no de la imagen, primera box de la lista que sea
    if border = False:                       # las 4 esquinas? o como lo pasa bo de disp a imagen?
        next()

    point_check_list = list()

    vector45 = box[5]-box[4]
    vector45_unit = vector45/np.linalg.norm(vector45)
    vector54_unit = vector45_unit*-1

    vector45_iter = vector45_unit * 4
    vector54_iter = vector54_unit * 4


    iter = 0
    p_list = list()
    while 1:
        iter += 1
        point = int(box[5] + iter * vector45_iter)
        p_list.append(point)
        if point[0] < 0 or point[0] > imshape[0] or point[1] < 0 or point[1] > imshape[1]: # TODO se le puede poner aqui el margen del buscador manhattan y nos ahorrariamos tener que considerar out of range en éste
            p_end1 = p_list[-2]
            break
        else:
            end = check_near(point, imshape)
            if end == True:
                p_end1 = p_list[-1]  # TODO -1 + (- dist manhatan de check near/4) - para tirar para atras los puntos que añadirá de mas al ir encontrando tuberia por atras (/4 pq vamos a saltos de 4 en 4, check vector_iter)
                break

    iter = 0
    p_list = list()
    while 1:
        iter += 1
        point = int(box[4] + iter * vector54_iter)
        p_list.append(point)
        if point[0] < 0 or point[0] > imshape[0] or point[1] < 0 or point[1] > imshape[1]: # TODO se le puede poner aqui el margen del buscador manhattan y nos ahorrariamos tener que considerar out of range en éste
            p_end2 = p_list[-2]
            break
        else:
            end = check_near(point, imshape)
            if end == True:
                p_end2 = p_list[-1]  # TODO -1 + (- dist manhatan de check near/4) - para tirar para atras los puntos que añadirá de mas al ir encontrando tuberia por atras (/4 pq vamos a saltos de 4 en 4, check vector_iter)
                break


    

def check_box(box, imshape, margin):
    border = False
    for point in box:
        if point[0] < margin or point[0] > imshape[0]-margin or point[1] < margin or point[1] > imshape[1]-margin:
            border = True
            break
    return border


def check_near(point, imshape)

    end = True

    # TODO for por los ixj pixeles de alrededor (distancia manhatan) y si alguno forma aprte de tuberia (por color), end = False y break

    return end



# añadir nuevas iteraciones para vectores ortogonales que se haran a partir de los puntos end y que sera +- vector unitario ortogonal a los vectores unit o directamente iter y 
# sera en loop hasta que no se encuentren near, tener en cuuenta que no se vaya a infinito ya que al principio detectara la propia tuberia, se puede anular el principio, hacer
# un salto grande al principio ... pero al ser sobre 2d no tenemos tamaños, lo cual lo dificulta.



















    



io.imshow(im)
io.show()







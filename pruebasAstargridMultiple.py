"""
Código para probar algoritmos, y determinar la menor distancia 
alcanzada entre dos robots durante la ejecución a velocidad 
constante
"""


import numpy as np
import math as m
import sys
import matplotlib.pyplot as plt
import os
from AlgoritmoMR import a_multiples_robots
from Algoritmo import a_star
import cv2
import time
ti = time.time()
""" 
    CONSTANTS ---------------------------------------------
"""
#NoR = 12
map_file = 'mapaVacio.txt'
robot_radius = 0.25 # meters
gap = 0.2 # meters
cell_size = 0.1 #meters

#random_seed = 151
# 21 interferencia entre metas

radius = m.ceil((robot_radius + gap/2)/cell_size)
# print(radius)
minimun_distance = (radius*2)**2
# print(minimun_distance)
"""--------------------------------------------------------"""


if os.path.exists(map_file):
    # print('Map found. Loading...')
    occgrid = np.loadtxt(map_file)
else:
    # print('Map not found. Exiting...')
    sys.exit()

disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def randomPoint(map, random_seed, NoR):
    gcell = []
    carcell = []
    r_pos = []

    # Assign a seed to generate random scenarios
    # print('Creating scenario...')
    rnd = np.random.default_rng(seed = random_seed) # 666

    for k in range(NoR): 
        while True:
            xr = rnd.integers(10, 90)
            yr = rnd.integers(10, 90)
            x_o = xr*0.1 - 5
            y_o = (50 - yr)*0.1 

            not_valid = any( ((xx - xr)**2 + (yy - yr)**2) <= minimun_distance  for yy, xx in r_pos )

            if ((map[yr, xr] == 0) and not not_valid):
                break
        r_pos.append( (yr, xr) )
        carcell.append((yr, xr))

    i_pos = []

    for k in range(NoR): 
        while True:
            xr = rnd.integers(10, 90)
            yr = rnd.integers(10, 90)
            not_valid = any( ((xx - xr)**2 + (yy - yr)**2) <= minimun_distance  for yy, xx in i_pos )
            
            if map[yr, xr] == 0 and not not_valid:
                break
        i_pos.append( (yr, xr) )

        gcell.append((yr, xr))

    return carcell, gcell




def algorithm( random_seed, NoR, obstacles):
    map = np.flipud(cv2.dilate(occgrid, disk))

    carpos = []
    carrot = []
    carcell = []
    gcell = []

    """""
    CREATES STARTING POINTS AND GOALS 
    """""
    carcell, gcell = randomPoint(map, random_seed, NoR)

    patharr = []
    # print('Calculatings paths...')
    if obstacles:
        patharr, steps, mes = a_multiples_robots(map, carcell, gcell, radius)
    else:
        patharr, steps, mes = a_star(map, carcell, gcell, radius)
        
    if len(patharr) == 0:
        print(mes)
        exit()


    # print('Calculating distances...')
    dis_min = 10000
    maxNsteps = max([len(stp) for stp in steps])-1
    xx = []
    yy = []
    for i in range(maxNsteps):
        for robot1 in range(NoR-1):
            lenpath = len(patharr[robot1])
            if lenpath == 0:
                continue
            if i > lenpath-1:
                i = lenpath-1
            val = steps[robot1][i]
            y1 = patharr[robot1][i][0]
            x1 = patharr[robot1][i][1]

            for robot2 in range(robot1+1, NoR):
                lenpath = len(patharr[robot2])
                if lenpath == 0:
                    continue
                if val >= steps[robot2][-1]:
                    y2 = patharr[robot2][-1][0]
                    x2 = patharr[robot2][-1][1]
                else:
                    valor_mas_cercano = min(steps[robot2], key=lambda elemento: abs(elemento - val))
                    ind = steps[robot2].index(valor_mas_cercano)
                    y2 = patharr[robot2][ind][0]
                    x2 = patharr[robot2][ind][1]

                
                dis = m.sqrt( (x1-x2)**2 + (y1-y2)**2 )
                if dis < dis_min:
                    dis_min = dis
                    xx = [x1, x2]
                    yy = [y1, y2]

    dis_min = 0.1*(dis_min-1)

    tf = time.time() - ti


    ###
    """ for k in range(NoR):
        r, c = path2cells(patharr[k])
        #print(steps[k])
        map[r, c] = 2 + k

    plt.imshow(map)
    plt.scatter(xx, yy, color='red')
    plt.show() """

    distances = [s[-1] for s in steps]

    res = []
    for i in range( len(carcell) ):
        aux = [i,carcell[i], gcell[i], distances[i]*0.1]
        res.append(aux)

    mindis = min(distances)
    return res, dis_min


if __name__ == '__main__':
    seed = 10
    NoR = 7
    datos, d_min = algorithm(seed, NoR)
    
    print('Robot - Inicio - Meta - Distancia recorrida')
    for d in datos:
        print(d)
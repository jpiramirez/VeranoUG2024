"""
Occupancy grid navigation using multiple DDR vehicles
This initial version does not check for collisions between agents.

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Vision, Robotics and Artificial Intelligence Laboratory
University of Guanajuato, 2024
"""


import numpy as np
import math as m
import sys
import matplotlib.pyplot as plt
import scipy.interpolate as spi
import os
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from astargrid import a_star, path2cells
import cv2

def v2u(v, omega, r, L):
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul

def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)

def cells2metric(path):
    route = []
    for cell in path:
        r, c = cell
        x = 0.1*(c - 50)
        y = -0.1*(r - 50)
        route.append([x, y])
    return route






client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(True)

motorL = []
motorR = []
robot = []

Nag = 3

t_paro = [[] for i in range(Nag)]
t_reanuda = [[] for i in range(Nag)]
t_desfase = [0 for i in range(Nag)]

ind_tr = [0 for i in range(Nag)]
tt_desfase = [0 for i in range(Nag)]

for k in range(Nag):
    name = '/robot' + str(k)
    motorL.append(sim.getObject(name + '/leftMotor'))
    motorR.append(sim.getObject(name + '/rightMotor'))
    robot.append(sim.getObject(name))

if os.path.exists('pracmap.txt'):
    print('Map found. Loading...')
    occgrid = np.loadtxt('pracmap.txt')
else:
    print('Map not found. Exiting...')
    sys.exit()

disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
map = np.flipud(cv2.dilate(occgrid, disk))

carpos = []
carrot = []
carcell = []


# CREA PUNTOS DE PARTIDA Y DESTINO
xp = [30, 50, 70]
yp = [50, 50, 60]
zop = sim.getObjectPosition(robot[0], -1)[2]
for k in range(Nag): 
    """ sim.setObjectPosition(robot[k], (xp[k], 0.5, z))
    sim.setObjectOrientation(robot[k], (0,0,-90.0))
    carpos.append(sim.getObjectPosition(robot[k])) """

    xop= xp[k]*0.1 - 5
    yop = (50 - yp[k])*0.1 

    sim.setObjectPosition(robot[k], (xop, yop, zop))
    sim.setObjectOrientation(robot[k], (0,0,-90.0))
    carpos.append(sim.getObjectPosition(robot[k]))


    carcell.append((yp[k], xp[k]))


# Getting goals for all agents (at random)
gcell = []

gcell.append((85, 50))
gcell.append((85, 30))
gcell.append((60, 10))



patharr = []
for k in range(Nag):
    start = carcell[k]
    end = gcell[k]
    patharr.append(a_star(map, start, end))
    r, c = path2cells(patharr[k])
    map[r, c] = 2 + k
""" plt.imshow(map)
plt.show()
print(carcell)
print(gcell) """

# Creating interpolators for the trajectories of all robots
routes = []
times = []
xspline = []
yspline = []
topvel = 0.2
graphtime = []
xgraph = []
ygraph = []
plt.figure()
for k in range(Nag):
    rt = np.array(cells2metric(patharr[k]))
    routes.append(rt)
    times.append(0.1*rt.shape[0]/topvel)
    xpt = rt[:,0]
    ypt = rt[:,1]
    tarr = np.linspace(0, times[k], rt.shape[0])
    graphtime.append(np.linspace(0, times[k], 200))
    xspline.append(spi.splrep(tarr, xpt, s=0))
    yspline.append(spi.splrep(tarr, ypt, s=0))
    xgraph.append(spi.splev(graphtime[k], xspline[k], der=0))
    ygraph.append(spi.splev(graphtime[k], yspline[k], der=0))
    #plt.plot(xgraph[k], ygraph[k])
#plt.show()


pex = []
pey = []

t_valid = True
t = 0
print(f'ti - {times}')
while(t<min(times)):
    t += 1
    x = []
    y = []
    dis = []

    # x[], y[] de todos los robot en tiempo t
    for s in range(len(xspline)):
        cond = ind_tr[s] < len(t_paro[s]) 
        #print(ind_tr[s], len(t_paro[s]) )
        
        if cond and t >= t_paro[s][ind_tr[s]] and t < t_reanuda[s][ind_tr[s]]:
            
            x.append(1000)
            y.append(1000)

        else:
            x.append(spi.splev(t-t_desfase[s], xspline[s], der=0))
            y.append(spi.splev(t-t_desfase[s], yspline[s], der=0))
    
        if cond and t >= t_reanuda[s][ind_tr[s]]:
            ind_tr[s] += 1
            t_desfase[s] += 5.5
            print(f'pasaaaa seg {t}  -> {s}')

    # Verifica coliciones por distancia
    for i in range(len(xspline) - 1):
        
        for j in range(i+1, len(xspline)):
            #print(f'{i},{j}') 
            distance = (x[i]-x[j])**2+(y[i]-y[j])**2
            dis.append( (x[i]-x[j])**2+(y[i]-y[j])**2 )

            if distance < 0.3 and x[i] < 900 and x[j] < 900:
                print(f'{distance}')
                t_paro[j].append(t-1.5)
                t_reanuda[j].append(t+4)
                times[j] += 5.5
                x1 = 50 + m.ceil(x[j]/0.1)
                y1 = 50 - m.floor(y[j]/0.1)
                #
                pex.append(x1)
                pey.append(y1)
                
print(f'to - {times}')



    
plt.imshow(map)
plt.plot(pex, pey, 'o', color='red', markersize=5)
plt.show()

#print('Trajectory execution times')


T = 0.05
Kv = 0.1/T
Kh = 0.2/T
rr = 0.5*0.195
L = 0.311







sim.startSimulation()

t_desfase = [0 for i in range(Nag)]
tottime = max(times)
t = sim.getSimulationTime()
trt = 0
ban = True
while sim.getSimulationTime()-t < tottime:
    

    for k in range(Nag):
        ctime = sim.getSimulationTime()-t - t_desfase[k]

        # Verifica si existe un paro en el segundo t
        if len(t_paro[k]) > 0:
            # Para
            if ctime >= t_paro[k][0] and  ctime < t_reanuda[k][0]:
                sim.setJointTargetVelocity(motorL[k], 0)
                sim.setJointTargetVelocity(motorR[k], 0)
                if ban:
                    trt = sim.getSimulationTime()
                continue

            # Termina paro y elimina los valores de este
            if ctime >= t_reanuda[k][0]:
                t_paro[k].pop(0)
                t_reanuda[k].pop(0)
                t_desfase[k] += 5.5
                times[k] += 5.5
                tottime = max(times)
                trt = sim.getSimulationTime() - trt
                print(f'tespera -> {trt}')

        



        carpos = sim.getObjectPosition(robot[k], -1)
        xd = spi.splev(ctime, xspline[k], der=0)
        yd = spi.splev(ctime, yspline[k], der=0)
        carrot = sim.getObjectOrientation(robot[k], -1)
        errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
        angd = m.atan2(yd-carpos[1], xd-carpos[0])
        errh = angdiff(carrot[2], angd)
        v = Kv*errp
        omega = Kh*errh

        ur, ul = v2u(v, omega, rr, L)
        if ctime < times[k]:
            sim.setJointTargetVelocity(motorL[k], ul)
            sim.setJointTargetVelocity(motorR[k], ur)
        else:
            sim.setJointTargetVelocity(motorL[k], 0)
            sim.setJointTargetVelocity(motorR[k], 0)
    
    for k in range(1):
        sim.step()



sim.stopSimulation()
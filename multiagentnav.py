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

for k in range(Nag):
    name = '/robot' + str(k)
    motorL.append(sim.getObject(name + '/leftMotor'))
    motorR.append(sim.getObject(name + '/rightMotor'))
    robot.append(sim.getObject(name))

if os.path.exists('obsmap.txt'):
    print('Map found. Loading...')
    occgrid = np.loadtxt('obsmap.txt')
else:
    print('Map not found. Exiting...')
    sys.exit()

disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
map = np.flipud(cv2.dilate(occgrid, disk))

carpos = []
carrot = []
carcell = []
for k in range(Nag):
    carpos.append(sim.getObjectPosition(robot[k], -1))
    xw = carpos[k][0]
    yw = carpos[k][1]
    xr = 50 + m.ceil(xw/0.1)
    yr = 50 - m.floor(yw/0.1)
    carcell.append((yr, xr))

# Getting goals for all agents (at random)
rng = np.random.default_rng(seed=0)
gcell = []
for k in range(Nag):
    col = True
    while col:
        rg = rng.integers(0, 100)
        cg = rng.integers(0, 100)
        if map[rg, cg] == 0 and abs(carcell[k][0]-cg)+abs(carcell[k][1]-rg) > 10:
            col = False
    gcell.append((rg, cg))

patharr = []
for k in range(Nag):
    start = carcell[k]
    end = gcell[k]
    patharr.append(a_star(map, start, end))
    r, c = path2cells(patharr[k])
    map[r, c] = 2 + k
plt.imshow(map)
plt.show()
print(carcell)
print(gcell)

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
    plt.plot(xgraph[k], ygraph[k])
plt.show()
    
print('Trajectory execution times')
print(times)

T = 0.05
Kv = 0.1/T
Kh = 0.2/T
r = 0.5*0.195
L = 0.311


sim.startSimulation()

tottime = max(times)
t = sim.getSimulationTime()
while sim.getSimulationTime()-t < tottime:
    ctime = sim.getSimulationTime()-t

    for k in range(Nag):
        carpos = sim.getObjectPosition(robot[k], -1)
        xd = spi.splev(ctime, xspline[k], der=0)
        yd = spi.splev(ctime, yspline[k], der=0)
        carrot = sim.getObjectOrientation(robot[k], -1)
        errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
        angd = m.atan2(yd-carpos[1], xd-carpos[0])
        errh = angdiff(carrot[2], angd)
        v = Kv*errp
        omega = Kh*errh

        ur, ul = v2u(v, omega, r, L)
        if ctime < times[k]:
            sim.setJointTargetVelocity(motorL[k], ul)
            sim.setJointTargetVelocity(motorR[k], ur)
        else:
            sim.setJointTargetVelocity(motorL[k], 0)
            sim.setJointTargetVelocity(motorR[k], 0)
    
    for k in range(1):
        sim.step()

    
sim.stopSimulation()

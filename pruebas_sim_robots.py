"""
Occupancy grid navigation using multiple DDR vehicles
This initial version does not check for collisions between agents.

"""


import numpy as np
import math as m
import sys
import matplotlib.pyplot as plt
import os
import scipy.interpolate as spi
from astargridMultiple import a_star, path2cells
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
import time
ti = time.time()
""" 
    -----------------CONSTANTS ---------------------------
"""
T = 0.05
Kv = 0.1/T
Kh = 0.2/T
rr = 0.5*0.195
L = 0.311


NoR = 6
map_file = 'mapaVacio.txt'
robot_radius = 0.25 # meters
gap = 0.05 # meters
cell_size = 0.1 #meters
random_seed = 666

radius = (robot_radius + gap/2)/cell_size
minimun_distance = (radius*2)**2

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(True)


"""
------------------ FUNCTIONS- ---------------------------
"""
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

def cal_ran_dis(x1, x2, y1, y2, dm):
    dis = m.sqrt( (x1-x2)**2 + (y1-y2)**2 )
    if dis > dm:
        return True
    else:
        return False



""" 
    ----------------- LOADING MAP ---------------------------
"""

if os.path.exists(map_file):
    print('Map found. Loading...')
    occgrid = np.loadtxt(map_file)
else:
    print('Map not found. Exiting...')
    sys.exit()

disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
map = np.flipud(cv2.dilate(occgrid, disk))

carpos = []
carrot = []
carcell = []
gcell = []


"""
    -------- ASIGN ROBOTS-----------
"""
motorL = []
motorR = []
robot = []

for k in range(NoR):
    name = '/robot' + str(k)
    motorL.append(sim.getObject(name + '/leftMotor'))
    motorR.append(sim.getObject(name + '/rightMotor'))
    robot.append(sim.getObject(name))



"""""
  -----------CREATES STARTING POINTS AND GOALS---------------
"""""
r_pos = []
# Assign a seed to generate random scenarios
print('Creating scenario...')
zop = sim.getObjectPosition(robot[0], -1)[2]
rnd = np.random.default_rng(seed = random_seed) 

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
    sim.setObjectPosition(robot[k], (x_o, y_o, zop))

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

print(carcell)
print(gcell)


"""
    ---------------- CALCULATING PATHS (A*)----------------------------
"""

patharr = []
print('Calculatings paths...')
patharr, steps, mes = a_star(map, carcell, gcell, radius)
if len(patharr) == 0:
    print(mes)
    exit()


for k in range(NoR):
    r, c = path2cells(patharr[k])
    map[r, c] = 2 + k


""""
 CREATING INTERPOLATORS FOR THE TRAJCTORIES OF ALL ROBOTS----------------
"""
routes = []
times = []
xspline = []
yspline = []
topvel = 0.2
graphtime = []
xgraph = []
ygraph = []
plt.figure()
for k in range(NoR):
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

plt.imshow(map)
plt.show()




# Ajusta el angulo de los robots
for k in range(NoR): 
    p1 = routes[k][0]
    p2 = routes[k][2]
    angd = m.atan2(p2[1]-p1[1], p2[0]-p1[0])
    sim.setObjectOrientation(robot[k], (0,0,angd), -1)



""""
    Start simulation
"""
sim.startSimulation()


tottime = max(times)
t = sim.getSimulationTime()

tx = [[] for i in range(NoR)]
ty = [[] for i in range(NoR)]

while sim.getSimulationTime()-t < tottime:
      

    for k in range(NoR):
        

        ctime = sim.getSimulationTime()-t

        ### Determines the expected robot position
        carpos = sim.getObjectPosition(robot[k], -1)
        xd = spi.splev(ctime, xspline[k], der=0)
        yd = spi.splev(ctime, yspline[k], der=0)


        ### Calculates the error between the expected and the actual position
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
        
        ### Data to final graphics
        poa = sim.getObjectPosition(robot[k], -1)
        tx[k].append(poa[0])
        ty[k].append(poa[1])

    for k in range(1):
        sim.step()

    

sim.stopSimulation()



for i in range(NoR):
    plt.plot(tx[i], ty[i], color='b')
    plt.plot(xgraph[i], ygraph[i], color='r')
plt.show()
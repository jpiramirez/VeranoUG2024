""" 
A-star implementation for occupancy grid maps, represented as a numpy array of shape (rows, cols)

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Vision, Robotics and Artificial Intelligence Laboratory
University of Guanajuato, 2024
"""

import heapq
import numpy as np
import matplotlib.pyplot as plt
import math as mt
import cv2

SQR2 = mt.sqrt(2)

class Node:
  def __init__(self, position, parent=None):
    self.position = position
    self.parent = parent
    self.g = 0
    self.h = 0
    self.f = 0

  def __lt__(self, other):
    return self.g < other.g



def heuristic(current, goal):
    return mt.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2) 


def pintaRobotDisco(map, no, r):
    cv2.circle(map, (no[1], no[0]), r, 1, -1)
    return map
    

def a_star(mapO, starts, goals, radius, diagonals = True):
    paths = []
    steps = []
    a_steps = []
    diagonal = mt.ceil(2*radius)
    NoR = len(starts)

    distances = [mt.sqrt( (g[0] - s[0])**2 + (g[1] - s[1])**2 ) for s, g in zip(starts, goals)]


    for k in range(NoR - 1):
        g1 = starts[k]
        for l in range(k+1, NoR):
            g2 = starts[l]
            dis = mt.sqrt( (g1[0]-g2[0])**2 + (g1[1]-g2[1])**2 )
            if dis < diagonal:
                return ([],[],'interferencia entre punto de salida')


    for k in range(NoR - 1):
        g1 = goals[k]
        for l in range(k+1, NoR):
            g2 = goals[l]
            dis = mt.sqrt( (g1[0]-g2[0])**2 + (g1[1]-g2[1])**2 )
            if dis < diagonal:
                return ([],[],'interferencia entre metas')
    

    for i in range(NoR):
        map_in = mapO.copy()
        flag = False
        start = starts[i]
        goal = goals[i]
        goal_added = [False for _ in range(len(goals))]

        rows, cols = map_in.shape
        open_set = []
        closed_set = set()
        start_node = Node(start)
        heapq.heappush(open_set, start_node)
        if diagonals:
            movs = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
        else:
            movs = ((1, 0), (-1, 0), (0, 1), (0, -1))
        ls = 1
        while open_set:
            current = heapq.heappop(open_set)
            closed_set.add(current.position)

            if current.position == goal:
                path = []
                step = []
                while current:

                    path.append(current.position)
                    step.append(current.g)
                    current = current.parent
                paths.append(path[::-1])
                steps.append(step[::-1])
                flag = True
                break


            ### Add robot trajetories as obstacles
            for k in range(NoR):
                ind = current.g

                if k == i or goal_added[k] == True or ind<=distances[k]: 
                    continue
                map_in = cv2.circle(map_in, (goals[k][1], goals[k][0]), diagonal, 1, -1)
                goal_added[k] = True

            map = map_in.copy()
            if len(paths) > 0:      
                
                for k in range(len(paths)):
                    valor_mas_cercano = min(steps[k], key=lambda elemento: abs(elemento - current.g))
                    ind = steps[k].index(valor_mas_cercano)

                    path = paths[k]
                    step = steps[k]
                    lenre = len(path)

                    if ind < lenre-1:
                        map = cv2.circle(map, (path[ind][1], path[ind][0]), diagonal, 1, -1)



            ### Iterates over all posible moves
            for dx, dy in movs:
                x, y = current.position[0] + dx, current.position[1] + dy
                add_cost = SQR2 if (abs(dx) + abs(dy)) > 1 else 1

                if 0 <= x < rows and 0 <= y < cols and map[x, y] == 0:
                    neighbor = Node((x, y), current)
                    if neighbor.position not in closed_set:
                        if neighbor.position not in (node.position for node in open_set):
                            neighbor.g = current.g + add_cost
                            neighbor.h = heuristic(neighbor.position, goal)
                            neighbor.f = neighbor.g + neighbor.h
                            heapq.heappush(open_set, neighbor)
                        else:
                            new_cost = current.g + add_cost
                            better = any(node.g >= new_cost for node in open_set if node.position == neighbor.position)
                            if not better:
                                neighbor.g = current.g + add_cost
                                neighbor.f = neighbor.g + neighbor.h
                


        if not flag:
            print(f' path {i} no tiene solucion')
            paths.append([])  # No path found
            steps.append([])
            return([], [], 'Una trayectoria no se pudo cumplir')
    res = (paths, steps, ' ')
    return res

def path2cells(path):
    rows = []
    cols = []
    for p in path:
        rows.append(p[0])
        cols.append(p[1])
    return rows, cols

if __name__ == '__main__':
    map = np.array([
            [0 for _ in range(120)] for _ in range(120)
        ])
    
    for i in range(1, 6):
        #rdx = random_number1 = random.randint(10, 230)
        #rdy = random_number1 = random.randint(10, 230)
        rdx = i*20
        rdy = 50
        for ry in range(rdy-4, rdy+4):
            for rx in range(rdx-4, rdx+4):
                map[ry][rx] = 1

    start = [(50, 0)]
    goal = [(50, 110)]

    path, step, m = a_star(map, start, goal, diagonals=True)
    print(path[0])

    for i in range(len(path)):
        for x, y in path[i]:
            map[x][y] = 2

    plt.imshow(map)
    plt.show() 


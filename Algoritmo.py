"""
implementación de a-star. no toma en cuenta colisiones con robots
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
    self.s = 0 # Number of steps to this node

  def __lt__(self, other):
    return self.g < other.g



def heuristic(current, goal):
    return mt.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2) 

def a_star(map, starts, goals, radius, diagonals = True):
    paths = []
    steps = []
    a_steps = []

    NoR = len(starts)

    for i in range(NoR):
        flag = False
        start = starts[i]
        goal = goals[i]
        goal_added = [False for _ in range(len(goals))]

        rows, cols = map.shape
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
                            neighbor.s = current.s + 1
                            heapq.heappush(open_set, neighbor)
                        else:
                            new_cost = current.g + add_cost
                            better = any(node.g >= new_cost for node in open_set if node.position == neighbor.position)
                            if not better:
                                neighbor.g = current.g + add_cost
                                neighbor.f = neighbor.g + neighbor.h
                                neighbor.s = current.s + 1
                


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


"""
    Creación de prueba
"""
if __name__ == '__main__':
    map = np.array([
            [0 for _ in range(120)] for _ in range(120)
        ])
    
    for i in range(1, 6):
        rdx = i*20
        rdy = 50
        for ry in range(rdy-4, rdy+4):
            for rx in range(rdx-4, rdx+4):
                map[ry][rx] = 1

    start = [(50, 0), (40,10)]
    goal = [(50, 110), (80, 10)]

    path, step, m = a_star(map, start, goal, 3, diagonals=True)

    for i in range(len(path)):
        for x, y in path[i]:
            map[x][y] = 2

    plt.imshow(map)
    plt.show() 


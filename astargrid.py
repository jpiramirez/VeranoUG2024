""" 
A-star implementation for occupancy grid maps, represented as a numpy array of shape (rows, cols)

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Vision, Robotics and Artificial Intelligence Laboratory
University of Guanajuato, 2024
"""

import heapq
import numpy as np

class Node:
  def __init__(self, position, parent=None):
    self.position = position
    self.parent = parent
    self.g = 0
    self.h = 0
    self.f = 0

  def __lt__(self, other):
    return self.f < other.f

def heuristic(current, goal):
  # Manhattan distance heuristic 
  return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

def a_star(map, start, goal, diagonals = True):
  rows, cols = map.shape
  open_set = []
  closed_set = set()
  start_node = Node(start)
  heapq.heappush(open_set, start_node)
  if diagonals:
    movs = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
  else:
    movs = ((1, 0), (-1, 0), (0, 1), (0, -1))

  while open_set:
    current = heapq.heappop(open_set)
    closed_set.add(current.position)

    if current.position == goal:
      path = []
      while current:
        path.append(current.position)
        current = current.parent
      return path[::-1]

    for dx, dy in movs:
      x, y = current.position[0] + dx, current.position[1] + dy
      if 0 <= x < rows and 0 <= y < cols and map[x, y] == 0:
        neighbor = Node((x, y), current)
        if neighbor.position not in closed_set:
          if neighbor.position not in (node.position for node in open_set):
            neighbor.g = current.g + 1
            neighbor.h = heuristic(neighbor.position, goal)
            neighbor.f = neighbor.g + neighbor.h
            heapq.heappush(open_set, neighbor)
          else:
            new_cost = current.g + 1
            better = any(node.f > new_cost for node in open_set if node.position == neighbor.position)
            if better:
              neighbor.g = current.g + 1
              neighbor.f = neighbor.g + neighbor.h
  return None  # No path found

def path2cells(path):
    rows = []
    cols = []
    for p in path:
        rows.append(p[0])
        cols.append(p[1])
    return rows, cols

if __name__ == '__main__':
    map = np.array([
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 0],
        [0, 1, 0, 0]
        ])
    start = (0, 0)
    goal = (3, 3)

    path = a_star(map, start, goal, diagonals=True)
    print(path)
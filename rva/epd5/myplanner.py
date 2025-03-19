"""
implement here your path planning method
"""

import rospy
import math
from nav_msgs.msg import OccupancyGrid
import numpy as np


class Planner:
    def __init__(self, costmap):
        """ 
        Initialize a map from a ROS costmap
        
        costmap: ROS costmap
        """
        # Copy the map metadata
        self.resolution = costmap.info.resolution
        self.min_x = costmap.info.origin.position.x
        self.min_y = costmap.info.origin.position.y
        self.y_width = costmap.info.height
        self.x_width = costmap.info.width
        self.max_x = self.min_x + self.x_width *self.resolution
        self.max_y = self.min_y + self.y_width *self.resolution
        print("min corner x: %.2f m, y: %.2f m", self.min_x, self.min_y)
        print("max corner x: %.2f m, y: %.2f m", self.max_x, self.max_y)
        print("Resolution: %.3f m/cell", self.resolution)
        print("Width: %i cells, height: %i cells", self.x_width, self.y_width)

        self.nodes = {}

        # Copy the actual map data from the map
        x = 0
        y = 0
         # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        obstacles = 0
        for value in costmap.data:
            if value > 90:  # This value could change depending on the map
                obstacles += 1
                self.obstacle_map[x][y] = True
            # Update the iterators
            x += 1
            if x == self.x_width:
                x = 0
                y += 1
        print("Loaded %d obstacles"%(obstacles))

        # NOTE: alternatively, instead of computing a binary 
        # obstacle map, you can try to store directly the costmap 
        # values and use them as costs.
        
           
    class Node:
        def __init__(self, cx, cy, g, h, parent):
            self.x_cell = cx  # x index in the obstacle grid
            self.y_cell = cy  # y index in the obstacle grid

            # TODO: add the node costs that you may need
            self.g = g
            self.h = h
            self.f = g + h

            self.parent = parent  # index of the previous Node


    def h(self, x, y, goal):
        return math.sqrt((goal.x_cell - x)**2 + (goal.y_cell - y)**2)


    def get_node(self, x, y):
        hash_id = f"{x};{y}"
        if hash_id in self.nodes:
            return self.nodes[hash_id]
        else:
            return None


    def update_node(self, x, y, g, goal, parent):
        hash_id = f"{x};{y}"
        if hash_id in self.nodes:
            node = self.nodes[hash_id]
            node.g = g
            node.f = node.g + node.h
            node.parent = parent
            return node
        else:
            node = self.Node(x, y, g, self.h(x, y, goal), parent)
            self.nodes[hash_id] = node
            return node


    def plan(self, sx, sy, gx, gy):
        """
        Fill with your search method

        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        
        # first check if we are already very close
        d = math.sqrt((gx-sx)*(gx-sx) + (gy-sy)*(gy-sy))
        if d <= self.resolution*2.0:
            return None

        # create the start node and the goal node
        goal_cell_x, goal_cell_y = self.real2cell(gx, gy)
        goal_node = self.Node(goal_cell_x, goal_cell_y, math.inf, 0.0, None)
        self.nodes[f"{goal_cell_x};{goal_cell_y}"] = goal_node
        start_cell_x, start_cell_y = self.real2cell(sx, sy)  
        start_node = self.update_node(start_cell_x, start_cell_y, 0.0, goal_node, None)

        # check if the positions are valid (no obstacle)
        if (not self.node_is_valid(start_node)):
            print("Error: start position not valid!!")
            return None
        
        if (not self.node_is_valid(goal_node)):
            print("Error: goal position not valid!!")
            return None
        
        # TODO: fill the rest of the function
        unchecked = [start_node]
        checked = []
        while len(unchecked) > 0:

            # current = node in the list with the smallest f value
            current = unchecked[0]
            for node in unchecked:
                if node.f < current.f:
                    current = node

            # remove current from list
            unchecked.remove(current)
            checked.append(current)

            # If (current == goal) return Success
            if current.x_cell == goal_node.x_cell and current.y_cell == goal_node.y_cell:
                goal_node.parent = current
                break



            neighbour_kernels = [
                [-1, -1],
                [0, -1],
                [1, -1],
                [-1, 0],
                [1, 0],
                [-1, 1],
                [0, 1],
                [1, 1],
            ]

            # For each node, n that is adjacent to current:
            for kernel in neighbour_kernels:
                # Get or create neighboring Node
                x_n = current.x_cell + kernel[0]
                y_n = current.y_cell + kernel[1]
                n = self.get_node(x_n, y_n)
                if n is None:
                    n = self.update_node(x_n, y_n, math.inf, self.h(x_n, y_n, goal_node), -1)
                if not self.node_is_valid(n):
                    continue

                edge_cost = np.linalg.norm(np.array(kernel))
                # if n.g > (current.g + cost of edge from n to current)
                if n.g > current.g + edge_cost:
                    g = current.g + edge_cost

                    # n.g = current.g + cost of edge from n to current
                    # n.f = n.g + h(n)
                    # n.parent = current
                    n = self.update_node(n.x_cell, n.y_cell, g, goal_node, current)

                    # Add n to list if it is not there already
                    if n not in unchecked and n not in checked:
                        unchecked.append(n)

        # store you path points here
        x, y = self.cell2real(goal_node.x_cell, goal_node.y_cell)
        rx = [x]
        ry = [y]
        current = goal_node.parent
        while current is not None:
            x, y = self.cell2real(current.x_cell, current.y_cell)
            rx.append(x)
            ry.append(y)
            current = current.parent

        # return the path
        rx.reverse()
        ry.reverse()
        return rx, ry

    

    # Transform map coordinates in meters
    # to cell indexes in the grid
    def real2cell(self, rx, ry):
        cellx = round((rx - self.min_x) / self.resolution)
        celly = round((ry - self.min_y) / self.resolution)
        return cellx, celly
    
    # Tranform cell indexes of the grid
    # to map coordinates in meters
    def cell2real(self, cx, cy):
        rx = cx * self.resolution + self.min_x
        ry = cy * self.resolution + self.min_y
        return rx, ry

    
    def node_is_valid(self, node):

        # check that the node is inside the grid limits
        rx, ry = self.cell2real(node.x_cell, node.y_cell)
        if rx < self.min_x:
            return False
        if ry < self.min_y:
            return False
        if rx >= self.max_x:
            return False
        if ry >= self.max_y:
            return False
        
        # check if the cell is an obstacle
        if self.obstacle_map[int(node.x_cell)][int(node.y_cell)]:
            return False

        return True

    



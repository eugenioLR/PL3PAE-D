""" This module implements the A* path planning algorithm.

Two variants are included: grid-based, and mesh-based.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""

__author__ = "Mario Cobos Maestre"
__authors__ = ["Mario Cobos Maestre"]
__contact__ = "mario.cobos@edu.uah.es"
__copyright__ = "Copyright 2019, UAH"
__credits__ = ["Mario Cobos Maestre"]
__date__ = "2019/03/29"
__deprecated__ = False
__email__ =  "mario.cobos@edu.uah.es"
__license__ = "GPLv3"
__maintainer__ = "Mario Cobos Maestre"
__status__ = "Development"
__version__ = "0.0.1"


"""
    Code modified from https://gist.github.com/jamiees2/5531924
"""

import path_planning as pp

def children(point,grid):
    """
        Calculates the children of a given node over a grid.
        Inputs:
            - point: node for which to calculate children.
            - grid: grid over which to calculate children.
        Outputs:
            - list of children for the given node.
    """
    x,y = point.grid_point
    if x > 0 and x < len(grid) - 1:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x,y + 1),(x+1,y),\
                      (x-1, y-1), (x-1, y+1), (x+1, y-1),\
                      (x+1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x+1,y),\
                      (x-1, y-1), (x+1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y + 1),(x+1,y),\
                      (x-1, y+1), (x+1, y+1)]]
    elif x > 0:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x,y + 1),\
                      (x-1, y-1), (x-1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x-1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y), (x,y + 1), (x-1, y+1)]]
    else:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y),(x,y - 1),(x,y + 1),\
                      (x+1, y-1), (x+1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y),(x,y - 1),(x+1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y), (x,y + 1), (x+1, y+1)]]
    return [link for link in links if link.value != 9]

def thetaStar(start, goal, grid, heur='naive'):
    """
        Executes the A* path planning algorithm over a given grid.
        Inputs:
            - start: node from which to start.
            - goal: node to which it is desired to arrive.
            - grid: grid over which to execute the algorithm
            - heur: heuristic function to use for the algorithm,
            expressed as a string. Results will vary depending on
            it. Must be implemented separatedly.
        Outputs:
            - ordered list of nodes representing the shortest path found
            from start to goal.
    """
    #The open and closed sets
    openset = set()
    closedset = set()
    #Current point is the starting point
    current = start
    #Add the starting point to the open set
    openset.add(current)
    #While the open set is not empty
    while openset:
        #Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o:o.G + o.H)
        pp.expanded_nodes += 1
        #If it is the item we want, retrace the path and return it
        if current == goal:
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            return path[::-1]
        #Remove the item from the open set
        openset.remove(current)
        #Add it to the closed set
        closedset.add(current)
        #Loop through the node's children/siblings
        for node in children(current,grid):
            #If it is already in the closed set, skip it
            if node in closedset:
                continue
            """
            if node in openset:
                #Check if we beat the G score
                new_g = current.G + current.move_cost(node)
                if node.G > new_g:
                    #If so, update the node to have a new parent
                    node.G = new_g
                    node.parent = current
            else:
                #If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + current.move_cost(node)
                node.H = pp.heuristic[heur](node, goal)
                #Set the parent to our current item
                node.parent = current
                #Add it to the set
                openset.add(node)
            """
            #Otherwise if it is already in the open set
            if node in openset:
                if lineOfSight(node.parent, node, grid):
                    new_g = node.parent.G + node.parent.move_cost(node)
                    if node.G > new_g:
                        node.G = new_g
                else:
                    new_g = current.G + current.move_cost(node)
                    if node.G > new_g:
                        node.G = new_g
                        node.parent = current
            else:
                #If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + current.move_cost(node)
                node.H = pp.heuristic[heur](node, goal)
                #Set the parent to our current item
                node.parent = current
                #Add it to the set
                openset.add(node)

    #Throw an exception if there is no path
    raise ValueError('No Path Found')

pp.register_search_method('Theta*', thetaStar)



def lineOfSight(current, new_node, grid):
    x0, y0 = current.grid_point[0], current.grid_point[1]
    x1, y1 = new_node.grid_point[0], new_node.grid_point[1]
    dx, dy = y1 - y0, x1 - x0
    f = 0
    if dy < 0:
        dy = -dy
        sy = -1
    else:
        sy = 1

    if dx < 0:
        dx = -dx
        sx = -1
    else:
        sx = 1

    if dx >= dy:
        while x0 != x1:
            f = f + dy
            if f >= dx:
                if grid[x0 + (sx - 1)//2][y0 + (sy - 1)//2]:
                    return False
                y0 = y0 + sy
                f = f - dx

            if f != 0 and grid[x0 + (sx - 1)//2][y0 + (sy - 1)//2]:
                return False

            if dy == 0 and grid[x0 + (sx - 1)//2][y0] and grid[x0 + (sx - 1)//2][y0 - 1]:
                return False

            x0 = x0 + sx
    else:
        while y0 != y1:
            f = f + dx
            if f >= dx:
                if grid[x0 + (sx - 1)//2][y0 + (sy - 1)//2]:
                    return False
                x0 = x0 + sx
                f = f - dy

            if f != 0 and grid[x0 + (sx - 1)//2][y0 + (sy - 1)//2]:
                return False

            if dx == 0 and grid[x0][y0 + (sy - 1)//2] and grid[x0 - 1][y0 + (sy - 1)//2]:
                return False

            y0 = y0 + sy

    return True



def thetaStar_mesh(start, goal, grid, heur='naive'):
    """
        Executes the A* path planning algorithm over a given nav mesh.
        Inputs:
            - start: node from which to start.
            - goal: node to which it is desired to arrive.
            - grid: mesh over which to execute the algorithm
            - heur: heuristic function to use for the algorithm,
            expressed as a string. Results will vary depending on
            it. Must be implemented separatedly.
        Outputs:
            - ordered list of nodes representing the shortest path found
            from start to goal.
    """
    #The open and closed sets
    openset = set()
    closedset = set()
    #Current point is the starting point
    current = start
    #Add the starting point to the open set
    openset.add(current)
    #While the open set is not empty
    while openset:
        #Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o:o.G + o.H)
        pp.expanded_nodes += 1
        #If it is the item we want, retrace the path and return it
        if current == goal:
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            return path[::-1]
        #Remove the item from the open set
        openset.remove(current)
        #Add it to the closed set
        closedset.add(current)
        #Loop through the node's children/siblings
        for node in current.neighbors.values():
            #If it is already in the closed set, skip it
            if node in closedset:
                continue
            #Otherwise if it is already in the open set
            if node in openset:
                #Check if we beat the G score
                new_g = current.G + current.move_cost(node)
                if node.G > new_g:
                    #If so, update the node to have a new parent
                    node.G = new_g
                    node.parent = current
            else:
                #If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + current.move_cost(node)
                node.H = pp.heuristic[heur](node, goal)
                #Set the parent to our current item
                node.parent = current
                #Add it to the set
                openset.add(node)
    #Throw an exception if there is no path
    raise ValueError('No Path Found')

pp.register_search_method('Theta* mesh', thetaStar_mesh)

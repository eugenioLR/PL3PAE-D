""" This module implements several heuristics.

They assume the Node class provided with this simulator is being used.

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


import path_planning as pp
import math

def manhattan(point,point2):
    """
        Function that calculates the manhattan distance between 2 points
        d = |p1.x - p2.x| + |p1.y - p2.y|
    """
    return abs(point.grid_point[0] - point2.grid_point[0]) + abs(point.grid_point[1] - point2.grid_point[1])

pp.register_heuristic('manhattan', manhattan)

def naive(point, point2):
    """
        Function that performs a naive heuristic.
    """
    return 1

pp.register_heuristic('naive', naive)

def euclidean(point, point2):
    """
        Function that calculates the euclidean distance between 2 points
        d = sqrt((p1.x - p2.x)² + (p1.y - p2.y)²)
    """
    x = point.grid_point[0] - point2.grid_point[0]
    y = point.grid_point[1] - point2.grid_point[1]
    return math.sqrt(x*x + y*y)

pp.register_heuristic('euclidean', euclidean)

def octile(point, point2):
    """
        Function that calculates the octile distance between 2 points
        d = max(p1.x - p2.x, p1.y - p2.y) + (sqrt(2)-1) * min(p1.x - p2.x, p1.y - p2.y)
    """
    x = abs(point.grid_point[0] - point2.grid_point[0])
    y = abs(point.grid_point[1] - point2.grid_point[1])

    return math.sqrt(2) * min(x, y) + max(x, y) 

pp.register_heuristic('octile', octile)

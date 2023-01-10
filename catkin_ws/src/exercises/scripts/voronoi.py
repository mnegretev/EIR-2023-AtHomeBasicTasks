#!/usr/bin/env python3
#
# ESCUELA DE INVIERNO DE ROBOTICA 2022-2023
# DIAGRAMAS DE VORONOI GENERALIZADOS
#

import rospy
import numpy
import queue
import sys
import math
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapResponse
from nav_msgs.srv import GetMapRequest

NAME = "FULL_NAME"

def brushfire(grid_map):
    print("Executing brushfire algorithm...")
    distances = numpy.full(grid_map.shape, -1.0)
    [height, width] = grid_map.shape
    offsets_4 = [[1,0],[0,1],[-1,0],[0,-1]]
    offsets_8 = [[1,1],[-1,1],[-1,-1],[1,-1]]

    Q = queue.Queue()
    for i in range(height):
        for j in range(width):
            if grid_map[i,j] != 0:
                distances[i,j] = 0
                if i > 0 and i < height-1 and j > 0 and j < width-1:
                    Q.put([i,j])

    while not Q.empty():
        sys.stdout.write("\rRemaining cells: %d            " % (Q.qsize()))
        sys.stdout.flush()
        [i,j] = Q.get_nowait()
        d = distances[i,j]
        for k1, k2 in offsets_4:
            if distances[i+k1, j+k2] == -1:
                Q.put([i+k1, j+k2])
                distances[i+k1, j+k2] = d + 1
            else:
                distances[i+k1, j+k2] = min(distances[i+k1, j+k2], d+1)
        for k1, k2 in offsets_8:
            if distances[i+k1, j+k2] == -1:
                Q.put([i+k1, j+k2])
                distances[i+k1, j+k2] = d + math.sqrt(2.0)
            else:
                distances[i+k1, j+k2] = min(distances[i+k1, j+k2], d+math.sqrt(2.0))
    return distances

def find_maxima(distances):
    [height, width] = distances.shape
    maxima = numpy.full(distances.shape, 0)
    offsets = [[1,0], [1,1], [0,1], [-1,1], [-1,0], [-1,-1], [0,-1], [1,-1]]
    for i in range(height):
        for j in range(width):
            if distances[i,j] == 0:
                continue
            is_maximum = False
            for k1, k2 in offsets:
                is_maximum |= (distances[i+k1, j+k2] <= distances[i,j] and distances[i-k1, j-k2] <  distances[i,j])
            if is_maximum:
                maxima[i,j] = 100
    return maxima

def callback_point_stamped(msg):
    global map_info, voronoi_map, distances
    [x,y] = msg.point.x,  msg.point.y
    r = int((y - map_info.origin.position.y)/map_info.resolution)
    c = int((x - map_info.origin.position.x)/map_info.resolution)
    print("Checking point: " + str([x,y]) + " = " + str([r,c]))
    print(numpy.flipud(distances[r-6:r+7, c-6:c+7]))
    
    
def main():
    global map_info, voronoi_map, distances
    print("INITIALIZING GVD BY BRUSHFIRE...")
    rospy.init_node("voronoi_diagram")
    pub_map  = rospy.Publisher("/generalized_voronoi_diagram", OccupancyGrid, queue_size=10)
    sub_point= rospy.Subscriber("/clicked_point", PointStamped, callback_point_stamped)
    print("Waiting for inflated map...")
    rospy.wait_for_service('/inflated_map')
    grid_map = rospy.ServiceProxy("/inflated_map", GetMap)().map
    map_info = grid_map.info
    width, height, res = map_info.width, map_info.height, map_info.resolution
    grid_map = numpy.reshape(numpy.asarray(grid_map.data, dtype='int'), (height, width))
    distances = brushfire(grid_map)
    voronoi_data = find_maxima(distances)
    print("Executed brushfire succesfully")
    voronoi_data = numpy.ravel(numpy.reshape(voronoi_data, (width*height, 1)))
    voronoi_map  = OccupancyGrid(info=map_info, data=voronoi_data)
    loop = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub_map.publish(voronoi_map)
        loop.sleep()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    

#!/usr/bin/env python3
#
# ESCUELA DE INVIERNO DE ROBOTICA 2022-2023
# ALGORITMO SPLIT AND MERGE
#

import rospy
import numpy
import math
import tf
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker

NAME = "FULL NAME"

def adjust_line(points):
    [xm,ym] = numpy.mean(points, 0)
    n,d = 0,0
    for x,y in points:
        n += (xm - x)*(ym - y)
        d += (ym - y)**2 - (xm - x)**2
    theta = 0.5*math.atan2(-2*n , d)
    rho   = xm*math.cos(theta) + ym*math.sin(theta)
    length= numpy.linalg.norm(points[0] - points[-1])
    return rho, theta, xm, ym, length

def find_farthest_point(points, rho, theta):
    distances = [abs(points[i][0]*math.cos(theta) + points[i][1]*math.sin(theta) - rho) for i in range(len(points))]
    idx = numpy.argmax(distances)
    return idx, distances[idx]
        
def split(points, threshold, min_points):
    if len(points) < min_points:
        return []
    rho, theta, xm, ym, length = adjust_line(points)
    idx, dist  = find_farthest_point(points, rho, theta)
    if dist < threshold:
        return [[rho, theta, xm, ym, length]]
    lines1 = split(points[0:idx], threshold, min_points)
    lines2 = split(points[idx+1:len(points)], threshold, min_points)
    return lines1 + lines2

def merge(lines, rho_tol, theta_tol):
    if len(lines) < 2:
        return lines
    new_lines = []
    for i in range(1, len(lines)):
        rho1, theta1, xm1, ym1, length1 = lines[i]
        rho2, theta2, xm2, ym2, length2 = lines[i-1]
        e_rho   = abs((rho1 - rho2)/min(rho1, rho2))
        e_theta = abs(theta1 - theta2)
        if e_rho < rho_tol and e_theta < theta_tol:
            new_lines.append([(rho1+rho2)/2, (theta1+theta2)/2, (xm1+xm2)/2, (ym1+ym2)/2, length1+length2])
        else:
            new_lines.append([rho1, theta1, xm1, ym1, length1])
            new_lines.append([rho2, theta2, xm2, ym2, length2])
    return new_lines

def filter_by_distance(points, dist_threshold):
    new_points = []
    for i in range(1,len(points)):
        if numpy.linalg.norm(points[i] - points[i-1]) < dist_threshold:
            new_points.append(points[i])
    return numpy.asarray(new_points)
            
def split_and_merge(points, point2point_threshold, point2line_threshold, min_points, rho_tol, theta_tol):
    points = filter_by_distance(points, point2point_threshold)
    lines  = split(points, point2line_threshold, min_points)
    lines  = merge(lines, rho_tol, theta_tol)
    return lines

def get_line_markers(lines):
    marker = Marker(ns="segmented_lines", id=7, type=Marker.LINE_LIST, action=Marker.ADD)
    marker.header  = Header(frame_id="map", stamp=rospy.Time.now())
    marker.scale.x = 0.07;
    marker.color   = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)
    for [rho, theta, xm, ym, length] in lines:
        a  = math.cos(theta)
        b  = math.sin(theta)
        marker.points.append(Point(xm + length/2*(-b), ym + length/2*(a), 0.5))
        marker.points.append(Point(xm - length/2*(-b), ym - length/2*(a), 0.5))
    return marker

def get_laser_pose():
    global listener
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'laser_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, z, a]
    except:
        pass
    return [0,0,0,0]

def laser_scan_to_points(msg_scan, laser_x, laser_y, laser_a):
    points = []
    for i in range(len(msg_scan.ranges)):
        if not (math.isnan(msg_scan.ranges[i]) or msg_scan.ranges[i] >= msg_scan.range_max):
            r,theta = msg_scan.ranges[i], i*msg_scan.angle_increment + msg_scan.angle_min + laser_a
            points.append([r*math.cos(theta) + laser_x, r*math.sin(theta) + laser_y])
    points = numpy.asarray(points)
    return points

def callback_scan(msg):
    global obstacle_detected, pub_line_markers
    global point2point_threshold, point2line_threshold, min_points, rho_tol, theta_tol
    x,y,z,a = get_laser_pose()
    points  = laser_scan_to_points(msg, x, y, a)
    lines   = split_and_merge(points, point2point_threshold, point2line_threshold, min_points, rho_tol, theta_tol)
    pub_line_markers.publish(get_line_markers(lines))
    return

def main():
    global pub_line_markers, listener
    global point2point_threshold, point2line_threshold, min_points, rho_tol, theta_tol
    point2point_threshold = 0.1
    point2line_threshold  = 0.1
    min_points = 5
    rho_tol    = 0.05
    theta_tol  = 0.05

    print("INITIALIZING SPLIT AND MERGE...")
    rospy.init_node("split_and_merge")
    if rospy.has_param("~p2p_thr"):
        point2point_threshold = rospy.get_param("~p2p_thr")
    if rospy.has_param("~p2l_thr"):
        point2line_threshold  = rospy.get_param("~p2l_thr")
    if rospy.has_param("~min_points"):
        min_points = rospy.get_param("~min_points")
    if rospy.has_param("~rho_tol"):
        rho_tol    = rospy.get_param("~rho_tol")
    if rospy.has_param("~theta_tol"):
        theta_tol = rospy.get_param("~theta_tol")
        
    pub_line_markers = rospy.Publisher("/navigation/segmented_lines_marker", Marker, queue_size=1)
    listener = tf.TransformListener()
    listener.waitForTransform('map', 'laser_link', rospy.Time(), rospy.Duration(10.0))
    rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    

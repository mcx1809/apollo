#!/usr/bin/python
from math import cos
from math import sin
import rosbag
from sys import argv
import matplotlib.pyplot as plt
import numpy as np


def read_bag(filename):
    """Extraxt obstacle messages and odometry messages from bag file
    Args:
            filename: ROS Bag file name
    Returns:
            tuple: (obstaclemsg_list, odometrymsg_list)
    """
    obs_list = []      # obstacle message list
    odo_list = []      # odometry message list
    bag = rosbag.Bag(filename)
    for topic, msg, t in bag.read_messages(topics=['/apollo/perception/obstacles']):
        obs_list.append(msg)
    for topic, msg, t in bag.read_messages(topics=['/apollo/localization/pose']):
        odo_list.append(msg)
    bag.close()
    return (obs_list, odo_list)


def calculate_offset(theta, relative_x, relative_y):
    """Calculate flu to enu transform offset according to heading
    Args:
            theta: heading value
            relative_x: x value in relative
            relative_y: y value in relative
    Returns:
            tuple: (x_offset_value, y_offset_value)
    """
    cos_theta = cos(theta)
    sin_theta = sin(theta)
    return (relative_x * cos_theta + relative_y * sin_theta,
            - relative_x * sin_theta + relative_y * cos_theta)


def get_curve_value(x_value, c0, c1, c2, c3):
    """Get the curve y_value according to x_value and curve analysis formula
        y = c3 * x**3 + c2 * x**2 + c1 * x + c0
    Args:
            x_value: value of x
            c3: curvature_derivative
            c2: curvature
            c1: heading_angle
            c0: position
    Returns:
            y_value according to the analysis formula with x = x_value
    """
    return c3 * (x_value ** 3) + c2 * (x_value ** 2) + c1 * x_value + c0


def cal_proportion(obs_list, odo_list, obs_index):
    """calculate the proportion of two gps localization proportion
     according to their timestamp and obstacle timestamp and
     get the suitable odo_index whose timestamp is nearest to
     the desired obstacle's timestamp
    Args:
        source_tuple:(obstaclemsg_list, odometrymsg_list)
    Returns:
        tuple: (proportion, odo_index)
    """
    c = obs_list[obs_index].header.timestamp_sec
    for odo_index in range(len(odo_list) - 1):
        a = odo_list[odo_index].header.timestamp_sec
        b = odo_list[odo_index + 1].header.timestamp_sec
        if(a < c and c < b):
            return ((b - c)/(b - a), odo_index)
        elif(c == a):
            return (1, odo_index)
        elif (c == b):
            return (0, odo_index)
        else:
            continue
    return(1, odo_index)


def collect_lane_marker_points(obs_list, odo_list):
    """collect ten points in every lane marker and 
    return the points list of left_lane_marker group 
    and right_lane_marker group
    Args:
        obs_list: obstacle message list from bag
        odo_list: odometry message list from bag
    Returns:
        left_lane_points: points list of left_lane_marker group
        right_lane_points: points list of right_lane_marker group
    """
    left_lane_points = []
    right_lane_points = []
    for obs_index in range(len(obs_list)):
        proportion, odo_index = cal_proportion(obs_list, odo_list, obs_index)
        localization_former = odo_list[odo_index].pose.position
        localization_next = odo_list[odo_index + 1].pose.position
        vehicle_heading_former = odo_list[odo_index].pose.heading
        vehicle_heading_next = odo_list[odo_index + 1].pose.heading
        heading_value = (vehicle_heading_former * proportion +
                         vehicle_heading_next * (1 - proportion))
        initial_x = (localization_former.x * proportion +
                     localization_next.x * (1 - proportion))
        initial_y = (localization_former.y * proportion +
                     localization_next.y * (1 - proportion))
        initial_z = (localization_former.z * proportion +
                     localization_next.z * (1 - proportion))
        left_c0_position = (
            obs_list[obs_index].lane_marker.left_lane_marker.c0_position)
        left_c1_heading_angle = (
            obs_list[obs_index].lane_marker.left_lane_marker.c1_heading_angle)
        left_c2_curvature = (
            obs_list[obs_index].lane_marker.left_lane_marker.c2_curvature)
        left_c3_curvature_derivative = (
            obs_list[obs_index].lane_marker.left_lane_marker.c3_curvature_derivative)
        for i in range(10):
            location_offset = calculate_offset(- heading_value, i * 0.1, get_curve_value(
                i * 0.1, left_c0_position, left_c1_heading_angle, left_c2_curvature, left_c3_curvature_derivative))
            px = initial_x + location_offset[0]
            py = initial_y + location_offset[1]
            #insert_point = True
            # for point in left_lane_points:
            #    if(abs(point[0] - px) < 0.10 and abs(point[1] - py) < 0.10):
            #        point[0] = (point[0] + px)/2.0
            #        point[1] = (point[1] + py)/2.0
            #        insert_point = False
            # if insert_point:
            #    left_lane_points.append([px, py])
            left_lane_points.append([px, py])

        right_c0_position = (
            obs_list[obs_index].lane_marker.right_lane_marker.c0_position)
        right_c1_heading_angle = (
            obs_list[obs_index].lane_marker.right_lane_marker.c1_heading_angle)
        right_c2_curvature = (
            obs_list[obs_index].lane_marker.right_lane_marker.c2_curvature)
        right_c3_curvature_derivative = (
            obs_list[obs_index].lane_marker.right_lane_marker.c3_curvature_derivative)
        for i in range(10):
            location_offset = calculate_offset(- heading_value, i * 0.1, get_curve_value(
                i * 0.1, right_c0_position, right_c1_heading_angle, right_c2_curvature, right_c3_curvature_derivative))
            px = initial_x + location_offset[0]
            py = initial_y + location_offset[1]
            #insert_point = True
            # for point in right_lane_points:
            #    if(abs(point[0] - px) < 0.10 and abs(point[1] - py) < 0.10):
            #        point[0] = (point[0] + px)/2.0
            #        point[1] = (point[1] + py)/2.0
            #        insert_point = False
            # if insert_point:
            #    right_lane_points.append([px, py])
            right_lane_points.append([px, py])
    return left_lane_points, right_lane_points


def plot_lane(left_points, right_points):
    """plot the points list in matplotlib 
    Args:
        left_points: points list of left lane marker group
        right_points: points list of right lane marker group
    """
    x_left = np.array(left_points)[:, 0]
    y_left = np.array(left_points)[:, 1]
    plt.scatter(x_left, y_left)
    x_right = np.array(right_points)[:, 0]
    y_right = np.array(right_points)[:, 1]
    plt.scatter(x_right, y_right)
    plt.show()


def main():
    """main entry of lane_markers_displayer
    Args:
        argv[1]:raw rosbag input to generate lanemarker msgs 
    """
    if(len(argv) != 2):
        print(
            "Please provide --argv[1]:raw rosbag input to generate lanemarker msgs")
        sys.exit()
    obs_list, odo_list = read_bag(argv[1])
    left_lane_points, right_lane_points = collect_lane_marker_points(
        obs_list, odo_list)
    plot_lane(left_lane_points, right_lane_points)


if __name__ == '__main__':
    main()

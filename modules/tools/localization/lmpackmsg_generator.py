#!/usr/bin/python
import rosbag
from modules.localization.proto import odometry_lane_marker_pb2


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
    for topic, msg, t in bag.read_messages(topics=['/apollo/sensor/gnss/odometry']):
        odo_list.append(msg)
    bag.close()
    return (obs_list, odo_list)


def write_info(source_tuple, filename):
    """1. acquire the correct gps msg from odometrymsg according to the 
              timestamp from obstaclemsg 
       2. set the value of properties of OdometryLaneMarker message
       3. group the OdometryLaneMarker message to ContourOdometryLaneMarkers
       4. set the property value of OdometryLaneMarkersPack according 
              to ContourOdometryLaneMarkers
       5. write the serialized string of OdometryLaneMarkersPack message to file
    Args: 
            source_tuple:(obstaclemsg_list, odometrymsg_list)
            filename: file to store protobuf msg
    """
    f = open(filename, "wb")
    odometry_lane_markers_pack = odometry_lane_marker_pb2.OdometryLaneMarkersPack()
    left_lane_marker_group = odometry_lane_markers_pack.lane_markers.add()
    right_lane_marker_group = odometry_lane_markers_pack.lane_markers.add()
    # each message in obstacle message list
    for obs_index in range(len(source_tuple[0])):
        # each message in odometry message list
        for odo_index in range(len(source_tuple[1]) - 1):
            a = source_tuple[1][odo_index].header.timestamp_sec
            b = source_tuple[1][odo_index + 1].header.timestamp_sec
            c = source_tuple[0][obs_index].header.timestamp_sec
            if(a < c and c < b):
                proportion = (c - a)/(b - a)
            elif c == a:
                proportion = 0
            elif c == b:
                proportion = 1
        localization_former = source_tuple[1][odo_index].localization.position
        localization_next = source_tuple[1][odo_index +
                                            1].localization.position
        left_lane_marker = left_lane_marker_group.lane_marker.add()
        left_c0_position = (source_tuple[0][obs_index].lane_marker
                            .left_lane_marker.c0_position)
        left_c1_heading_angle = (source_tuple[0][obs_index].lane_marker
                                 .left_lane_marker.c1_heading_angle)
        left_c2_curvature = (source_tuple[0][obs_index].lane_marker
                             .left_lane_marker.c2_curvature)
        left_c3_curvature_derivative = (source_tuple[0][obs_index].lane_marker
                                        .left_lane_marker.c3_curvature_derivative)
        # the endpoint shift 1m from start point in x direction
        left_endpoint_xshift = 1
        # the endpoint shift left_endpoint_yshift from start point in y direction accordingly
        left_endpoint_yshift = (left_c0_position + left_c1_heading_angle
                                + left_c2_curvature + left_c3_curvature_derivative)
        left_lane_marker.start_position.x = (localization_former.x * proportion
                                             + localization_next.x * (1 - proportion))
        left_lane_marker.start_position.y = (localization_former.y * proportion
                                             + localization_next.y * (1 - proportion) + left_c0_position)
        left_lane_marker.start_position.z = (localization_former.z * proportion
                                             + localization_next.z * (1 - proportion))
        left_lane_marker.end_position.x = (localization_former.x * proportion
                                           + localization_next.x * (1 - proportion) + left_endpoint_xshift)
        left_lane_marker.end_position.y = (localization_former.y * proportion
                                           + localization_next.y * (1 - proportion) + left_endpoint_yshift)
        left_lane_marker.end_position.z = (localization_former.z * proportion
                                           + localization_next.z * (1 - proportion))
        left_lane_marker.c0_position = left_c0_position
        left_lane_marker.c1_heading_angle = left_c1_heading_angle
        left_lane_marker.c2_curvature = left_c2_curvature
        left_lane_marker.c3_curvature_derivative = left_c3_curvature_derivative
        left_lane_marker.z_length = 1

        right_lane_marker = right_lane_marker_group.lane_marker.add()
        right_c0_position = (source_tuple[0][obs_index].lane_marker
                             .right_lane_marker.c0_position)
        right_c1_heading_angle = (source_tuple[0][obs_index].lane_marker
                                  .right_lane_marker.c1_heading_angle)
        right_c2_curvature = (source_tuple[0][obs_index].lane_marker
                              .right_lane_marker.c2_curvature)
        right_c3_curvature_derivative = (source_tuple[0][obs_index].lane_marker
                                         .right_lane_marker.c3_curvature_derivative)
        # the endpoint shift 1m from start point in x direction
        right_endpoint_xshift = 1
        # the endpoint shift left_endpoint_yshift from start point in y direction accordingly
        right_endpoint_yshift = (right_c0_position + right_c1_heading_angle
                                 + right_c2_curvature + right_c3_curvature_derivative)
        right_lane_marker.start_position.x = (localization_former.x * proportion
                                              + localization_next.x * (1 - proportion))
        right_lane_marker.start_position.y = (localization_former.y * proportion
                                              + localization_next.y * (1 - proportion) + right_c0_position)
        right_lane_marker.start_position.z = (localization_former.z * proportion
                                              + localization_next.z * (1 - proportion))
        right_lane_marker.end_position.x = (localization_former.x * proportion
                                            + localization_next.x * (1 - proportion) + right_endpoint_xshift)
        right_lane_marker.end_position.y = (localization_former.y * proportion
                                            + localization_next.y * (1 - proportion) + right_endpoint_yshift)
        right_lane_marker.end_position.z = (localization_former.z * proportion
                                            + localization_next.z * (1 - proportion))
        right_lane_marker.c0_position = right_c0_position
        right_lane_marker.c1_heading_angle = right_c1_heading_angle
        right_lane_marker.c2_curvature = right_c2_curvature
        right_lane_marker.c3_curvature_derivative = right_c3_curvature_derivative
        right_lane_marker.z_length = 1
    f.write(odometry_lane_markers_pack.SerializeToString())
    f.close()


def main():
    resource = read_bag(
        '/apollo/data/bag/2018-09-20-11-14-59/2018-09-20-11-15-00_0.bag')
    write_info(
        resource, "/apollo/modules/localization/testdata/OdometryLaneMarkers.bin")


if __name__ == '__main__':
    main()

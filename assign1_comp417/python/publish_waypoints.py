#!/usr/bin/env python

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
import rospy


def create_waypoints_publisher():
    # Make temporary ros node and publish message to follow_waypoints_node
    rospy.init_node('publish_waypoints', anonymous=True)
    rospy.loginfo("Publish waypoints node start")
    waypoint_publisher = rospy.Publisher('/waypoints_command', PointCloud, queue_size=1)

    while waypoint_publisher.get_num_connections() < 1:
        rospy.loginfo("Waiting for connection with /waypoint publisher")
        rospy.sleep(0.5)
    return waypoint_publisher


def create_waypoints_msg():
    waypoints_msg = PointCloud()
    waypoints_msg.header.stamp = rospy.Time.now()
    waypoints_msg.header.frame_id = 'odom'
    return waypoints_msg


def publish_waypoints(plan):

    waypoints_publisher = create_waypoints_publisher()
    waypoints_msg = create_waypoints_msg()

    # TODO Q3: Convert plan to list of points on ROS PointCloud message
    # See the main method for an example of publishing a single waypoint
    waypoints_msg.points = []

    rospy.loginfo("Sending waypoints command of length %d" % len(waypoints_msg.points))
    waypoints_publisher.publish(waypoints_msg)


if __name__ == '__main__':
    # Example program for publishing waypoints path to ROS follow_waypoints_node.py
    waypoints_publisher = create_waypoints_publisher()
    waypoints_msg = create_waypoints_msg()
    start_point = Point(250, 250, 0)
    dest_point = Point(250, 350, 0)
    waypoints_msg.points = [start_point, dest_point]
    rospy.loginfo("Sending waypoints command of length %d" % len(waypoints_msg.points))
    waypoints_publisher.publish(waypoints_msg)




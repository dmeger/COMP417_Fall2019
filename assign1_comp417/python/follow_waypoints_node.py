#!/usr/bin/env python

import cv2
import math
import os

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

GAZEBO_PLANE_SIZE = (30.0, 30.0)


def create_marker_message(marker_type, marker_id, ns="marker", color=ColorRGBA(0, 0.0, 1.0, 1.0)):
    return Marker(
        type=marker_type,
        id=marker_id,
        lifetime=rospy.Duration(),
        pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        scale=Vector3(0.2, 0.2, 0.2),
        header=Header(frame_id='/odom', stamp=rospy.Time.now()),
        color=color,
        action=Marker.ADD,
        ns=ns)


def to_image_coordinates(pos_x, pos_y, yaw, sample_points, map_image_shape):
    R = np.array([[np.cos(yaw), -np.sin(yaw)],
                  [np.sin(yaw), np.cos(yaw)]])
    proj_samples = np.matmul(R, sample_points)
    # Do manual translation instead of matmul by T avoid dealing with homogenous coordinates in T matrix
    cols = ((pos_x + proj_samples[0, :] + GAZEBO_PLANE_SIZE[1] / 2) / GAZEBO_PLANE_SIZE[1] * map_image_shape[
        1]).astype(int)
    rows = ((1 - (pos_y + proj_samples[1, :] + GAZEBO_PLANE_SIZE[0] / 2) / GAZEBO_PLANE_SIZE[0]) * map_image_shape[
        0]).astype(int)
    return cols, rows


class CarBoundingBox:
    """
    Logic for bounding box collision detection
    """
    def __init__(self, bbox_x, bbox_y, map_image):
        self.bbox_x = bbox_x
        self.bbox_y = bbox_y
        self.map_image = map_image
        sample_points = np.meshgrid(np.linspace(-bbox_x, bbox_y, 10), np.linspace(-bbox_x, bbox_y, 10))
        samples_x = sample_points[0].flatten()
        samples_y = sample_points[1].flatten()
        self.sample_points = np.transpose(np.concatenate((np.expand_dims(samples_x, 1), np.expand_dims(samples_y, 1)), axis=1))

    def is_collision(self, pos_x, pos_y, yaw):
        """
        Check for any non-white pixels beneath the vehicle
        """
        c, r = to_image_coordinates(pos_x, pos_y, yaw, self.sample_points, self.map_image.shape)
        inv_pixels = 255 - self.map_image[r, c]
        return np.sum(inv_pixels) > 0


class WaypointFollower:

    def __init__(self, end_on_collision, throttle, control_hz,
                 pid_p, waypoint_dist_threshold, stop_to_turn_threshold,
                 bounding_box):
        self.end_on_collision = end_on_collision
        self.throttle = throttle
        self.control_hz = control_hz
        self.pid_p = pid_p
        self.stop_to_turn_threshold = stop_to_turn_threshold
        self.waypoint_dist_threshold = waypoint_dist_threshold
        self.bounding_box = bounding_box
        self.map_image = self.bounding_box.map_image
        self.waypoint_itr = 0
        self.is_navigating = False

        self.robot_command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.waypoints_publisher = rospy.Publisher('/waypoint_path', Marker, queue_size=1)
        self.collision_publisher = rospy.Publisher('/collisions', PointCloud, queue_size=1)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=1)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.Subscriber('/waypoints_command', PointCloud, self.on_waypoints_received)
        self.pose = (0, 0, 0)

        while self.map_pub.get_num_connections() < 1:
            rospy.loginfo("Waiting for RVIZ connection to display map")
            rospy.sleep(0.5)
        rospy.loginfo("Waypoints follower node ready")
        self.publish_occupancy_grid()

    def publish_engine_command(self, throttle, steering):
        robot_command = Twist()
        robot_command.linear.x = throttle
        robot_command.angular.z = steering
        self.robot_command_pub.publish(robot_command)

    def waypoint_reached(self, waypoint, x, y):
        return np.linalg.norm((waypoint[0]-x, waypoint[1]-y)) < self.waypoint_dist_threshold

    def step(self, waypoints):

        if self.waypoint_itr >= len(waypoints):
            return True

        agent_x, agent_y, agent_angle = self.pose
        target_waypoint = waypoints[self.waypoint_itr]

        if self.waypoint_reached(target_waypoint, agent_x, agent_y):
            if self.waypoint_itr < len(waypoints)-1:
                self.waypoint_itr += 1
            else:
                rospy.loginfo("Navigation complete")
                self.publish_engine_command(0.0, 0.0)
                return True

        target_x = waypoints[self.waypoint_itr][0]
        target_y = waypoints[self.waypoint_itr][1]
        dx = target_x - agent_x
        dy = target_y - agent_y

        target_angle = math.atan2(dy, dx)

        error_angle = target_angle - agent_angle
        if error_angle > math.pi:
            error_angle = error_angle - 2 * math.pi
        if error_angle < -math.pi:
            error_angle = error_angle + 2 * math.pi

        # If error in angle too big: stop, turn then go
        applied_throttle = 0.0 if abs(error_angle) > self.stop_to_turn_threshold else self.throttle
        # rospy.loginfo("throttle=%.3f | target_angle=%.3f | agent_angle=%.3f | error=%.3f | dx=%.3f | dy=%.3f"
        #               % (applied_throttle, target_angle, agent_angle, error_angle, dx, dy))
        steer = self.pid_p * error_angle
        self.publish_engine_command(applied_throttle, steer)
        return False

    def publish_path_status(self, waypoints, collisions):

        marker_msg = create_marker_message(Marker.LINE_STRIP, 0)
        marker_msg.points = [Point(p[0], p[1], 0) for p in waypoints]
        self.waypoints_publisher.publish(marker_msg)

        collision_msg = PointCloud()
        collision_msg.header.stamp = rospy.Time.now()
        collision_msg.header.frame_id = '/odom'
        collision_msg.points = [Point(p[0], p[1], 0) for p in collisions]
        self.collision_publisher.publish(collision_msg)

    def on_odom(self, odom):
        pos = odom.pose.pose.position
        orientation_q = odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.pose = pos.x, pos.y, yaw

    def publish_occupancy_grid(self):
        origin = Pose()
        origin.position.x = -GAZEBO_PLANE_SIZE[0]/2.0
        origin.position.y = -GAZEBO_PLANE_SIZE[1]/2.0

        data = [0 if self.map_image[i][j] > 0 else 100 for i
                in range(self.map_image.shape[0]-1, -1, -1) for j in range(self.map_image.shape[1])]
        msg = OccupancyGrid(
            info=MapMetaData(
                width=self.map_image.shape[1],
                height=self.map_image.shape[0],
                resolution=GAZEBO_PLANE_SIZE[0]/self.map_image.shape[0],
                origin=origin
            ),
            data=data
        )
        msg.header.frame_id = 'odom'
        self.map_pub.publish(msg)

    def set_model_pose(self, x, y, yaw=0.0):
        state_msg = ModelState()
        state_msg.model_name = 'robot_model'
        state_msg.pose.position = Point(x, y, 0.1)
        state_msg.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, yaw))
        try:
            resp = self.set_state(state_msg)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s" % e)

    def on_waypoints_received(self, path):

        if len(path.points) < 2:
            rospy.logwarn("Ignoring waypoints command. "
                          "Must have atleast 2 waypoints to navigate between but received %d." % len(path.points))
            return

        if self.is_navigating:
            rospy.logwarn("Already executing path. New path ignored.")
            return

        rospy.loginfo("Received new waypoints path of length %d." % len(path.points))
        waypoints = to_gazebo_coordinates(path.points, map_image.shape, GAZEBO_PLANE_SIZE)

        self.is_navigating = True
        try:
            self.navigate(waypoints)
        finally:
            self.is_navigating = False
            self.publish_engine_command(0.0, 0.0)

    def navigate(self, waypoints):
        is_navgoal_reached = False

        self.set_model_pose(waypoints[0][0], waypoints[0][1])
        self.waypoint_itr = 1

        collisions = []
        rate = rospy.Rate(self.control_hz)

        self.publish_occupancy_grid()
        while not is_navgoal_reached:
            self.publish_path_status(waypoints, collisions)
            current_pose = self.pose
            if self.bounding_box.is_collision(*current_pose):
                collisions.append(current_pose)
                rospy.loginfo("Collision detected at pose = (x=%.3f, y=%.3f, yaw=%.3f)" % current_pose)
                if self.end_on_collision:
                    self.publish_engine_command(0.0, 0.0)
                    break

            is_navgoal_reached = self.step(waypoints)
            rate.sleep()


def to_gazebo_coordinates(states_in, img_shape, world_shape):

    gazebo_coords = np.array([[s.x * float(world_shape[0])/img_shape[1] - world_shape[0]/2.0,
                               world_shape[1] / 2.0 - s.y * float(world_shape[1]) / img_shape[0]]
                              for s in states_in])
    return gazebo_coords


if __name__ == '__main__':

    rospy.init_node('follow_waypoints', anonymous=True)
    rospy.loginfo("Follow waypoints node starting")

    pkg_dir = os.popen('rospack find assign1_comp417').read().rstrip()

    map_file = rospy.get_param("~map_file", os.path.join(pkg_dir, 'materials', 'map.png'))
    end_on_collision = rospy.get_param("~end_on_collision", False)
    throttle = rospy.get_param("~throttle", 0.6)
    update_hz = rospy.get_param("~update_hz", 60)
    pid_p = rospy.get_param("~pid_p", 0.4)
    waypoint_dist_threshold = rospy.get_param("~waypoint_dist_threshold", 0.1)
    stop_to_turn_threshold = rospy.get_param("~stop_to_turn_threshold", np.pi/9)
    bbox_x = rospy.get_param("~bbox_x", 0.205)
    bbox_y = rospy.get_param("~bbox_y", 0.165)
    map_image = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)

    bounding_box = CarBoundingBox(bbox_x, bbox_y, map_image)
    waypoint_follower = WaypointFollower(end_on_collision, throttle,
                                         update_hz, pid_p, waypoint_dist_threshold,
                                         stop_to_turn_threshold, bounding_box)

    while not rospy.is_shutdown():
        rospy.spin()

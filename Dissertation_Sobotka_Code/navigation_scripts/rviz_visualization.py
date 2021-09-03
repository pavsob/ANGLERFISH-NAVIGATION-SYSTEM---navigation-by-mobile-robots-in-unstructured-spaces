import rospy
# Markers
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
# Path
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import math

# Publishing marger message
def publish_start_goal_marker(marker_publisher, x, y, goal_x, goal_y):
    # START point
    start_point = Marker()
    start_point.header.frame_id = "/odom"
    start_point.header.stamp = rospy.get_rostime()
    start_point.id = 0
    start_point.type = Marker.SPHERE
    start_point.action = Marker.ADD
    # Stays displayed for ever
    #start_point.lifetime = 0
    my_point = Point()
    my_point.x = x
    my_point.y = y
    start_point.pose.position = my_point
    # Orientation
    start_point.pose.orientation.x = 0
    start_point.pose.orientation.y = 0
    start_point.pose.orientation.z = 0
    start_point.pose.orientation.w = 1
    # Scale
    start_point.scale.x = 0.1
    start_point.scale.y = 0.1
    start_point.scale.z = 0.05
    # Color
    start_point.color.r = 0
    start_point.color.g = 255
    start_point.color.b = 0
    # transparency
    start_point.color.a = 1
    # GOAL point
    goal_point = Marker()
    goal_point.header.frame_id = "/odom"
    goal_point.header.stamp = rospy.get_rostime()
    goal_point.id = 3
    goal_point.type = Marker.SPHERE
    goal_point.action = Marker.ADD
    # Stays displayed for ever
    #goal_point.lifetime = 0
    my_point = Point()
    my_point.x = goal_x
    my_point.y = goal_y
    goal_point.pose.position = my_point
    # Orientation
    goal_point.pose.orientation.x = 0
    goal_point.pose.orientation.y = 0
    goal_point.pose.orientation.z = 0
    goal_point.pose.orientation.w = 1
    # Scale
    goal_point.scale.x = 0.1
    goal_point.scale.y = 0.1
    goal_point.scale.z = 0.05
    # Color
    goal_point.color.r = 0
    goal_point.color.g = 0
    goal_point.color.b = 255
    # transparency
    goal_point.color.a = 1

    marker_publisher.publish(start_point)
    marker_publisher.publish(goal_point)

def publish_lantern_marker(marker_publisher, lantern_x, lantern_y):
    ###### Progress contour point - Lantern point
    lantern_point = Marker()
    lantern_point.header.frame_id = "/odom"
    lantern_point.header.stamp = rospy.get_rostime()
    lantern_point.id = 4
    lantern_point.type = Marker.SPHERE
    lantern_point.action = Marker.ADD
    # Stays displayed for ever
    #goal_point.lifetime = 0
    my_point = Point()
    my_point.x = lantern_x
    my_point.y = lantern_y
    lantern_point.pose.position = my_point
    # Orientation
    lantern_point.pose.orientation.x = 0
    lantern_point.pose.orientation.y = 0
    lantern_point.pose.orientation.z = 0
    lantern_point.pose.orientation.w = 1
    # Scale
    lantern_point.scale.x = 0.1
    lantern_point.scale.y = 0.1
    lantern_point.scale.z = 0.05
    # Color
    lantern_point.color.r = 255
    lantern_point.color.g = 255
    lantern_point.color.b = 0
    # transparency
    lantern_point.color.a = 1
    # Publishing the point of least resistance
    marker_publisher.publish(lantern_point)

# Publishing marger message
def publish_least_resistance_marker(marker_publisher, subgoal_coord):
    marker_point = Marker()
    marker_point.header.frame_id = "/odom"
    marker_point.header.stamp = rospy.get_rostime()
    marker_point.id = 1
    marker_point.type = Marker.SPHERE
    marker_point.action = Marker.ADD
    my_point = Point()
    my_point.x = subgoal_coord[0]
    my_point.y = subgoal_coord[1]
    marker_point.pose.position = my_point
    # Orientation
    marker_point.pose.orientation.x = 0
    marker_point.pose.orientation.y = 0
    marker_point.pose.orientation.z = 0
    marker_point.pose.orientation.w = 1
    # Scale
    marker_point.scale.x = 0.05
    marker_point.scale.y = 0.05
    marker_point.scale.z = 0.025
    # Color
    marker_point.color.r = 255
    marker_point.color.g = 0
    marker_point.color.b = 0
    # transparency
    marker_point.color.a = 1
    # Publishing the point of least resistance
    marker_publisher.publish(marker_point)

def publish_control_contour(marker_publisher, sample_angles, sample_dist, yaw, x, y):
    sample_coordinates = []
    for s_angle in sample_angles:
        sample_coordinates.append(calculation_to_odom(s_angle, sample_dist, yaw, x, y))
    line_strip = Marker()
    line_strip.header.frame_id = "/odom"
    line_strip.header.stamp = rospy.get_rostime()
    line_strip.id = 2
    line_strip.type = Marker.LINE_STRIP
    line_strip.action = Marker.ADD
    for line_coord in sample_coordinates:
        my_point = Point()
        my_point.x = line_coord[0]
        my_point.y = line_coord[1]
        line_strip.points.append(my_point)
    # Orientation
    #line_strip.pose.orientation.x = 0
    #line_strip.pose.orientation.y = 0
    #line_strip.pose.orientation.z = 0
    line_strip.pose.orientation.w = 1
    # Scale
    line_strip.scale.x = 0.05
    # Color
    line_strip.color.b = 1
    # transparency
    line_strip.color.a = 1
    # Publishing the point of least resistance
    marker_publisher.publish(line_strip)

#** For trajectory visualization **#
def publish_trajectory_visualization(path, path_publisher,curr_x,curr_y):

    pose_now = PoseStamped()
    pose_now.header.frame_id = '/odom'
    pose_now.header.stamp = rospy.get_rostime()
    pose_now.pose.position.x = curr_x
    pose_now.pose.position.y = curr_y
    path.poses.append(pose_now)
    path_publisher.publish(path)
    return path

def publish_progress_contour(sample_dist, progress,progress_path, progress_contour_publisher, to_goal_ratio, obstacle_potential,x, y, subgoal_x, subgoal_y, goal_x, goal_y, lantern_x, lantern_y):

    lantern_to_goal = euclidianDistance(lantern_x, lantern_y, goal_x, goal_y)
    # Here we need to compute new_alpha from subgoal_x, subgoal_y
    angle_to_subgoal = math.atan2(subgoal_y-lantern_y, subgoal_x - lantern_x)
    angle_to_goal = math.atan2(goal_y-lantern_y, goal_x - lantern_x)
    for step in range(-60, 65, 5):
        new_alpha = angle_to_subgoal + math.radians(step)
        triangle_angle = abs(angle_to_goal - new_alpha)
        #Basic version
        # a = ((1-progress)**2 - progress**2)
        # b = 2*lantern_to_goal*math.cos(triangle_angle)*progress**2
        # c = - (lantern_to_goal*progress)**2 - (obstacle_potential*progress)**2
        # to_goal_ratio VERSION
        a = ((1-progress*to_goal_ratio)**2 - (to_goal_ratio*progress)**2)
        b = 2*lantern_to_goal*math.cos(triangle_angle)*(to_goal_ratio*progress)**2
        c = - (lantern_to_goal*progress*to_goal_ratio)**2 - (obstacle_potential*progress*to_goal_ratio)**2
        discriminant = b**2 - 4*a*c
        if (discriminant >= 0):
            # Discriminant is above 0, therefore there are two solutions - only one is used here because that one is infront of the robot the other solution models contours beind the robot
            lantern_to_next = (-b + math.sqrt(discriminant)) / (2 * a)
            contour_coord = progress_contour_calculation_to_odom(new_alpha, lantern_to_next, lantern_x, lantern_y)
            contour_now = PoseStamped()
            contour_now.header.frame_id = '/odom'
            contour_now.header.stamp = rospy.get_rostime()
            contour_now.pose.position.x = contour_coord[0]
            contour_now.pose.position.y = contour_coord[1]
            progress_path.poses.append(contour_now)
    # Stops drawing when the robot reaches the goal
    to_goal = euclidianDistance(goal_x, goal_y, x, y)
    if(to_goal > sample_dist):
        progress_contour_publisher.publish(progress_path)
        progress_path = Path()
        progress_path.header.frame_id = '/odom'
        progress_path.header.stamp = rospy.get_rostime()
    return progress_path

def progress_contour_calculation_to_odom(point_odom_angel, point_dist, lantern_x, lantern_y):
    point_x = lantern_x + math.cos(point_odom_angel) * point_dist
    point_y = lantern_y + math.sin(point_odom_angel) * point_dist
    return (point_x, point_y)

# Calculates euclidian distance between two coordinate points
def euclidianDistance(from_x, from_y, to_x, to_y):
    return abs(math.sqrt(((to_x - from_x)**2 + (to_y - from_y)**2)))

# Calculates coordinates of the point with respect to odometry according to the robot's current position, point's angle and distance from the robot (point_angel, point_dist)
# Note: Robot's angles are negative on the left from the robot's facing direction, which is 0 degrees and positive on the right side
def calculation_to_odom(point_angel, point_dist, yaw, x, y):
    # Current position of the robot with respect to odometry - yaw, x, y
    if (yaw < 0):
        point_odom_angel = yaw + point_angel + 2*math.pi
    else:
        point_odom_angel = yaw + point_angel
    point_x = x + math.cos(point_odom_angel) * point_dist
    point_y = y + math.sin(point_odom_angel) * point_dist
    return (point_x, point_y)
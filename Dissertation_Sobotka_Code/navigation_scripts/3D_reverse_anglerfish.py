#!/usr/bin/env python

# # # # # # # # # # * * * * * * * * * Imported libraries and global variables * * * * * * * * * # # # # # # # # # #
import os
import sys
import threading
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math
import time
import numpy as np

# Simplex method
from scipy.optimize import minimize

# Visualization module
import rviz_visualization

# Marker messages
from visualization_msgs.msg import Marker

# Path messages
from nav_msgs.msg import Path
#from geometry_msgs.msg import PoseStamped

# Robot's obstacle memory - queue used for storing obstacle coordinates
import Queue
global obstacle_memory
num_in_memory = 1
obstacle_memory = Queue.Queue(num_in_memory)
# Secondary memory - keeps searching for the new closest obstacle when the main obstacle_memory is occupied
global flexible_memory
flexible_memory = Queue.Queue(num_in_memory)

# Visualization activation
global visualization_activated

# Deviation of the lantern point
global left_deviation_activated
left_deviation_activated = False
global right_deviation_activated
right_deviation_activated = False
global dive_lastly_passed
dive_lastly_passed = False
global dived
dived = False
global start_to_goal_angle
global loop_count
loop_count = 0

# Note: All angles in RADIANS

# # # # # # # # # # * * * * * * * * * Location Functions * * * * * * * * * # # # # # # # # # #
# Creates an array list of angles for given number of sample points
# control_span_angle - span of the control contour in radians, sample_num - chosen number of sample points
def create_sample_points_angles(control_span_angle, sample_num):
    # The angle starts from the left of the robot -> negative sign
    from_angle = - control_span_angle / 2
    # Angle increment that is gained with each step to the right
    angle_increment = control_span_angle / (sample_num-1)
    cur_sample_angle = from_angle
    sample_angles = []
    sample_angles.append(cur_sample_angle)
    for i in range(1,sample_num):
        cur_sample_angle += angle_increment
        sample_angles.append(cur_sample_angle)
    return sample_angles

# Calculates euclidian distance between two coordinate points
def euclidianDistance(from_x, from_y, to_x, to_y):
    return abs(math.sqrt(((to_x - from_x)**2 + (to_y - from_y)**2)))

# Calculates coordinates of the point with respect to odometry according to the robot's current position, point's angle and distance from the robot
# Note: Robot's angles are negative on the left from the robot's facing direction, which is 0 degrees and positive on the right side
def calculation_to_odom(point_angle, point_dist):
    # Current position of the robot with respect to odometry
    global x
    global y
    global yaw
    # Calculation to odometry
    if (yaw < 0):
        point_odom_angle = yaw + point_angle + 2*math.pi
    else:
        point_odom_angle = yaw + point_angle
    point_x = x + math.cos(point_odom_angle) * point_dist
    point_y = y + math.sin(point_odom_angle) * point_dist
    return (point_x, point_y)

# # # # # # # # # # * * * * * * * * * Lantern Point Deviation * * * * * * * * * # # # # # # # # # #
#** Percentage progress field algoritm's main function to decide lantern point deviation **#
# Function that decides when to deviate the lantern point
def deviate_lantern_point():
    # Values
    global yaw
    global start_to_goal_angle

    # True/False
    global left_deviation_activated
    global right_deviation_activated
    global dive_lastly_passed
    global dived

    # Loop count
    global loop_count

    # Angles
    critical_angle_left = -math.radians(120)
    critical_angle_right = math.radians(120)
    dive_angle_left = -math.radians(60)
    dive_angle_right = math.radians(60)
    deactivate_angle_left = -math.radians(90)
    deactivate_angle_right = math.radians(90)
    angle_interval = math.radians(5)

    # deviation of the lantern point angles
    straight_deviation_angle =  math.radians(180)
    left_deviation_angle =  math.radians(90)
    right_deviation_angle =  math.radians(-90)

    # Converts the current yaw angle of the robot to the system where the vector pointing from start to goal is 0
    conv_yaw = convert_yaw_deviation_system(start_to_goal_angle, yaw)

    ##** Deviation mode **##
    # Right deviation activated
    if (right_deviation_activated):
        # Checking the direction from where the robot is coming
        if ((conv_yaw > critical_angle_right) and (conv_yaw < critical_angle_right + angle_interval)):
            dive_lastly_passed = False
        elif ((conv_yaw > dive_angle_right - angle_interval) and (conv_yaw < dive_angle_right)):
            dive_lastly_passed = True
        # Checking for deactivation
        if((conv_yaw > (deactivate_angle_right - angle_interval)) and (conv_yaw < deactivate_angle_right)):
            if (dive_lastly_passed and not dived):
                loop_count+=1
            elif (not dive_lastly_passed and dived):
                loop_count-=1
            dived = False
            deactivate_deviation()
            return straight_deviation_angle
        else:
            return right_deviation_angle
    # Left deviation activate
    elif (left_deviation_activated):
        # Checking the direction from where the robot is coming
        if ((conv_yaw < critical_angle_left) and (conv_yaw > critical_angle_left - angle_interval)):
            dive_lastly_passed = False
        elif ((conv_yaw < dive_angle_left + angle_interval) and (conv_yaw > dive_angle_left)):
            dive_lastly_passed = True
        # Checking for deactivation
        if ((conv_yaw < (deactivate_angle_left + angle_interval)) and (conv_yaw > deactivate_angle_left)):
            if (dive_lastly_passed and not dived):
                loop_count+=1
            elif (not dive_lastly_passed and dived):
                loop_count-=1
            dived = False
            deactivate_deviation()
            return straight_deviation_angle
        else:
            return left_deviation_angle
    ##** Free mode **##
    else:
        # Checks if the robot done any loops and activates deviation - to be able to avoid snail type obstacles
        if ((conv_yaw < 0) and (conv_yaw > dive_angle_left) and not dived and loop_count > 0):
            dived = True
            left_deviation_activated = True
        elif ((conv_yaw > 0) and (conv_yaw < dive_angle_right) and not dived and loop_count > 0):
            dived = True
            right_deviation_activated = True

        # When one of the critical angles is exceeded the deviation is activated
        if (conv_yaw < critical_angle_left):
            left_deviation_activated = True
        elif (conv_yaw > critical_angle_right):
            right_deviation_activated = True
        return straight_deviation_angle

# Converts the current yaw angle of the robot to the system where the vector pointing from start to goal is 0
def convert_yaw_deviation_system(start_to_goal_angle, yaw):
    # start_to_goal_angle is 0 in a new system
    if (((start_to_goal_angle <= 0) and (yaw <= 0)) or ((start_to_goal_angle >= 0) and (yaw >= 0))):
        conv_yaw = start_to_goal_angle - yaw
    elif((start_to_goal_angle < 0) and (yaw >= 0)):
        conv_yaw = 2*math.pi + start_to_goal_angle - yaw
        if(conv_yaw > math.pi):
            conv_yaw = -2*math.pi + conv_yaw
    elif((start_to_goal_angle > 0) and (yaw <= 0)):
        conv_yaw = -2*math.pi + start_to_goal_angle - yaw
        if(conv_yaw < -math.pi):
            conv_yaw = 2*math.pi + conv_yaw
    return conv_yaw

# Function used for deactivating deviation mode
def deactivate_deviation():
    global left_deviation_activated
    global right_deviation_activated
    left_deviation_activated = False
    right_deviation_activated = False

# # # # # # # # # # * * * * * * * * * Sensor Filter Function * * * * * * * * * # # # # # # # # # #
# Filters the range information gained from the laser scanner - (ranges is the list that contains information of range of each laser beam)
# It reduces number of measurements accepted from the sensor to the specified number of beams (num_ranges)
# ranges - array of sensor measurements, angle_increment - angle distance between laser beams, sensor_span_angle - sensors maximum angle span, num_ranges - specified number of beams
def range_points_filter(ranges, angle_increment, num_ranges):

    # Filtering range measurements
    # Calculates the step to take to receive desired numbre of range measurements
    step = int(math.ceil(len(ranges) / num_ranges))
    ranges = ranges[::step]

    # UPDATES
    # Updates number of ranges in case the step was not able to split it evenly
    num_ranges = len(ranges)
    # Updating angle increment according to the filtered number of laser beams that are taken into account
    angle_increment = step * angle_increment

    # Angles for range information
    range_angles = []
    # Starts assigning angles from left - thus negative sign
    # sensor_half_angle_span is the positive half of the angle of the sensor's span - gained from the sensor rospy message
    global sensor_half_angle_span
    curr_range_angle = - sensor_half_angle_span
    range_angles.append(curr_range_angle)
    for i in range(1, num_ranges):
        curr_range_angle += angle_increment
        range_angles.append(curr_range_angle)

    # Drops nan - because we only care about the distance of detected obstacles
    # nan received from the sensor means that obstacles are further then 10 meters or too close below 0.44m
    range_data = zip(ranges, range_angles)
    cleaned_range_data = [t for t in range_data if str(t[0]) != 'nan']

    return cleaned_range_data

# # # # # # # # # # * * * * * * * * * Progress and Potential Calculation Functions * * * * * * * * * # # # # # # # # # #
# Calculates repulsive obstacle potential
def obstacle_potential(distance):
    # This condition is for obstacles in memory when they get further then sensor range, it stops calculating obstacle potential for them
    # This ensures that the robot won't still calculate obstacle potential for the obstacle in the memory that is more then 10 meters away
    sensor_range = 10
    if (distance > sensor_range):
        return 0
    # Power law function - determined from experiments
    return 0.06696128381/(distance**31)


# This function calculates potential in a given sample point according to the closest obstacle potential
### Modified to prove use of Reverse Angerfish method with deviating lantern point in 3-D ###
def progress_calculation_nearest_obstacle(x, sample_dist, range_data, goal_x, goal_y):
    # Current obstacle potential is made global because it is used for visualization
    global curr_obstacle_potential
    # In order to limit search to the control contour for optimal subgoal angle direction, everything that is out of the span of the control contour is asigned infinite value
    global control_span
    positive_control_span = control_span / 2
    negative_control_span = - control_span / 2
    if (x[0] > positive_control_span or x[0] < negative_control_span):
        result_progress = np.inf

    # Incorporates velocity
    max_vel=1
    min_vel=0.1
    if (x[1] > max_vel or x[1] < min_vel):
        result_progress = np.inf

    else:
        # Calculates the coordinates with respect to odometry for sample point and detected obstacles
        sample_coordinates = calculation_to_odom(x[0], sample_dist)
        obstacle_coordinates = []
        for r_data in range_data:
            obstacle_coordinates.append(calculation_to_odom(r_data[1], r_data[0]))

        # Activating the memory
        global obstacle_memory
        obstacle_memory_activation(sample_coordinates, obstacle_coordinates)
        if(not obstacle_memory.empty()):
            obstacle_coordinates.extend(obstacle_memory.queue)

        # Obstacle potential
        obst_distance_list = []
        if(len(obstacle_coordinates) > 0):
            for oc in obstacle_coordinates:
                obst_distance_list.append(euclidianDistance(sample_coordinates[0], sample_coordinates[1], oc[0], oc[1]))
            closest_obstacle_distance = min(obst_distance_list)
            # Current obstacle potential calculated from closest detected obstacle
            curr_obstacle_potential = obstacle_potential(closest_obstacle_distance)
        else:
            curr_obstacle_potential = 0

        # This makes the robot to prefer lower velocity (x[1]) because high velocity would have high potential - this makes it very slow around obstacles because 
        # around obstacles there is high obstacle potential when it is far from obstacle the numbers are very low and the speed is more of a random because it is 
        # rather hard for it to find some precise minimum
        # Nevertheless this serves a purpose and proves that this method is possible to be used in 3-D or even N-D spaces

        # Obstacle potential influenced by the velocity of the robot
        obst_pot_with_vel = curr_obstacle_potential + (curr_obstacle_potential/(1.1 - x[1]))

        # Resulting progress with modified obstacle potential
        result_progress = resulting_progress(sample_coordinates[0], sample_coordinates[1], goal_x, goal_y, obst_pot_with_vel)


    return result_progress

# Obstacle MEMORY with 2 memory storages
def obstacle_memory_activation(sample_coordinates, obstacle_coordinates):
    global obstacle_memory
    global flexible_memory
    if (len(obstacle_coordinates) > 0):
        # Currently seen obstacles and their distances with respect to the robot
        curr_obst_distance_list = []
        for oc in obstacle_coordinates:
            curr_obst_distance_list.append(euclidianDistance(sample_coordinates[0], sample_coordinates[1], oc[0], oc[1]))
        closest_obstacle_distance = min(curr_obst_distance_list)
        index_to_pop = curr_obst_distance_list.index(closest_obstacle_distance)
        closest_obstacle_coord = obstacle_coordinates.pop(index_to_pop)
        # we popped coordinates we have to pop distance as well
        curr_obst_distance_list.pop(index_to_pop)
        if (obstacle_memory.empty() and flexible_memory.empty()):
            obstacle_memory.put(closest_obstacle_coord)
            if (len(curr_obst_distance_list)>=1):
                # point that is second closest to the robot
                flexible_closest_obstacle_distance = min(curr_obst_distance_list)
                flexible_closest_obstacle_coord = obstacle_coordinates.pop(curr_obst_distance_list.index(flexible_closest_obstacle_distance))
                flexible_memory.put(flexible_closest_obstacle_coord)
        elif ((not obstacle_memory.empty()) and flexible_memory.empty()):
            obstacle_memory_dist_list = []
            for oc in obstacle_memory.queue:
                obstacle_memory_dist_list.append(euclidianDistance(sample_coordinates[0], sample_coordinates[1], oc[0], oc[1]))
            memory_closest_obstacle_distance = min(obstacle_memory_dist_list)
            if (closest_obstacle_distance > memory_closest_obstacle_distance):
                flexible_memory.put(closest_obstacle_coord)
            else:
                obstacle_memory.get()
                obstacle_memory.put(closest_obstacle_coord)
                if (len(curr_obst_distance_list)>=1):
                    # point that is second closest to the robot
                    flexible_closest_obstacle_distance = min(curr_obst_distance_list)
                    flexible_closest_obstacle_coord = obstacle_coordinates.pop(curr_obst_distance_list.index(flexible_closest_obstacle_distance))
                    flexible_memory.put(flexible_closest_obstacle_coord)
        else:
            obstacle_memory_dist_list = []
            for oc in obstacle_memory.queue:
                obstacle_memory_dist_list.append(euclidianDistance(sample_coordinates[0], sample_coordinates[1], oc[0], oc[1]))
            memory_closest_obstacle_distance = min(obstacle_memory_dist_list)
            flexible_memory_dist_list = []
            for oc in flexible_memory.queue:
                flexible_memory_dist_list.append(euclidianDistance(sample_coordinates[0], sample_coordinates[1], oc[0], oc[1]))
            flexible_closest_obstacle_distance = min(flexible_memory_dist_list)
            if (flexible_closest_obstacle_distance < memory_closest_obstacle_distance):
                if (not obstacle_memory.empty()):
                    obstacle_memory.get()
                obstacle_memory.put(flexible_memory.queue[0])
                # empty the flexible memory so it can find new point so it is never the same as current memory point
                flexible_memory.get()
            if ((flexible_closest_obstacle_distance > closest_obstacle_distance) or flexible_memory.empty()):
                if (closest_obstacle_distance > memory_closest_obstacle_distance):
                    if (not flexible_memory.empty()):
                        flexible_memory.get()
                    flexible_memory.put(closest_obstacle_coord)
                else:
                    if (len(curr_obst_distance_list)>=1):
                        # point that is second closest to the robot
                        flexible_second_closest_obstacle_distance = min(curr_obst_distance_list)
                        if(flexible_closest_obstacle_distance > flexible_second_closest_obstacle_distance):
                            flexible_closest_obstacle_coord = obstacle_coordinates.pop(curr_obst_distance_list.index(flexible_second_closest_obstacle_distance))
                            if (not flexible_memory.empty()):
                                flexible_memory.get()
                            flexible_memory.put(flexible_closest_obstacle_coord)

# Calculates over all progress for the given sample point
def resulting_progress(samp_x, samp_y, goal_x, goal_y, obstacle_potential):
    global start_x
    global start_y
    global x
    global y
    # Keeps to_goal_ratio global - for drawing progress contour
    global to_goal_ratio
    # Position of the lantern also kept global for the visualization purposes
    global lantern_x
    global lantern_y

    # Latern positioning exactly behind the robot in a free mode and under 90 or -90 degrees in a deviation mode
    # Deviation angle is decided accorging to the deviation function
    global start_to_goal_angle
    deviation_angle = deviate_lantern_point()
    lantern_position_in_odometry = calculation_to_odom(deviation_angle, 1)
    lantern_x = lantern_position_in_odometry[0]
    lantern_y = lantern_position_in_odometry[1]

    # Progress Contour
    lantern_to_next = euclidianDistance(samp_x, samp_y, lantern_x, lantern_y)
    next_to_goal = euclidianDistance(samp_x, samp_y, goal_x, goal_y)
    lantern_to_goal = euclidianDistance(lantern_x, lantern_y, goal_x, goal_y)
    progress_contour = (lantern_to_next / (lantern_to_next + next_to_goal + obstacle_potential))

    # Using current for to goal ratio to makes the contours more flexed towards the goal
    current_to_next = euclidianDistance(samp_x, samp_y, x, y)
    current_to_goal = euclidianDistance(goal_x, goal_y, x, y)
    to_goal_ratio = (current_to_next + next_to_goal) / current_to_goal

    # Result progress:
    resulting_progress = progress_contour / to_goal_ratio
    # The minus is placed here so it can use minimization method
    return -progress_contour

# # # # # # # # # # * * * * * * * * * Optimization Functions * * * * * * * * * # # # # # # # # # #
#** Simplex method **#
### Modified to prove use of Reverse Angerfish method with deviating lantern point in 3-D ###
def simplex_method_coordinate(sample_dist, range_data, goal_x, goal_y):
    # control span angles
    global control_span
    positive_control_span = control_span / 2
    negative_control_span = - control_span / 2
    # Define the starting point for a simplex
    x0 = [0,0.5]
    # Sets up initial simplex instead of x0
    initialSimplex = [[negative_control_span,0.1],[positive_control_span,0.1],[0,1]]

    # Performs the Simplex method
    # Simplex is generated automatically around x0 - which is updated every time for a previous minimum to speed up simplex method          
    result = minimize(progress_calculation_nearest_obstacle, x0, args=(sample_dist, range_data, goal_x, goal_y), options={'xatol': 0.01, 'initial_simplex': initialSimplex, 'maxiter': 40, 'adaptive': True}, method='Nelder-Mead')

    # Gets solution
    solution = result['x']
    global xopt
    xopt = solution[0]
    velopt =solution[1]

    # Summarize the result
    # print('Simplex Method info:')
    # print('Status : %s' % result['message'])
    # print('Total Function Evaluations: %d' % result['nfev'])
    # print('Method result: %s' % result['x'])
    # print('Solution-sample angle: %s' % (xopt))

    # Calculates coordinates of the sample point with the best progress
    sample_coordinates = calculation_to_odom(xopt, sample_dist)

    ## Progress Contour visualization
    global visualization_activated
    if (visualization_activated):
        global curr_obstacle_potential
        global progress_path
        global progress_contour_publisher
        global to_goal_ratio
        # Value of the best progress. There must be minus to make it positive again when before there was placed minus in order to use minimization
        progress_contour_percentage = - result['fun']
        progress_path = rviz_visualization.publish_progress_contour(progress_contour_percentage, progress_path, progress_contour_publisher, to_goal_ratio, curr_obstacle_potential, x, y, sample_coordinates[0], sample_coordinates[1], goal_x, goal_y, lantern_x, lantern_y)

    return sample_coordinates, velopt

# # # # # # # # # # * * * * * * * * * NAVIGATION * * * * * * * * * # # # # # # # # # #
# Calculates angular speed to achieve desired arch motion
def angular_speed_arch(linear_speed, subgoal_x, subgoal_y):
    # Global parameters of position
    global x
    global y
    global yaw

    # Euclidian distance to the subgoal
    distance = euclidianDistance(x, y, subgoal_x, subgoal_y)

    # Robot's angle to the subgoal, is calculated with respect to odometry by atan2 function - the vector pointing from the robot's position to the subgoal
    # Calculating alpha - alpha is angle between the direction the robot's facing and direction to the subgoal
    angle_to_subgoal = math.atan2(subgoal_y-y, subgoal_x-x)
    alpha = angle_to_subgoal - yaw
        
    # Calculating a radius of an arch to achieve desired subgoal point
    # if alpha is 0 it sets angular speed to 0 to avoid division by 0 (going straight motion)
    if (alpha == 0):
        angular_speed = 0
    else:
        radius = distance / (2 * math.sin(alpha))
        # Calculating angular velocity to achieve desired arch with calculated radius
        angular_speed = linear_speed / radius

    return angular_speed

# Function that moves the robot to the goal
# It calls all other functions needed to navigate to the ultimate goal - it accepts program parameters that can be altered
# control_dist is a chosen distance of the control contour
# num_ranges - is the number of diststance measurements taken from the sensor
def move_to_goal(vel_publisher, marker_publisher, control_dist, linear_speed, chosen_control_span, num_ranges, goal_x, goal_y, in_time, vis_activation):

    # Global parameters of position and the message type
    global x
    global y
    global yaw
    global angle_increment
    global laser_ranges

    # Control span describes a span of the control contour - the contour from where the sample points are taken and the subgoal is decided
    global control_span
    control_span = chosen_control_span

    # Visualization activated
    global visualization_activated
    visualization_activated = vis_activation

    # Robot's starting position
    global start_x
    global start_y
    start_x = x
    start_y = y

    # Calculates the angle direction to the goal from start - used in the deviate_lantern_point() function
    global start_to_goal_angle
    start_to_goal_angle = math.atan2(goal_y - start_y, goal_x - start_x)

    # Makes goal coordinates global so they can be changed during the run of the program
    global g_x, g_y
    g_x = goal_x
    g_y = goal_y

    # Velocity message type
    vel_msg = Twist()

    # FOR LINE_STRIP MARKERS - control contour
    num_points_control_contour = 6
    sample_angles_dist = create_sample_points_angles(control_span, num_points_control_contour)

    # Distance to the goal tolerance
    to_goal_tolerance = 0.2

    # Calculating arriving to goal time
    in_goal_time = rospy.get_time() + in_time

    # Starts the loop that continues until reaching the goal
    while not rospy.is_shutdown():
        # Checking if we did not reach the ultimate goal
        goal_dist = euclidianDistance(x, y, g_x, g_y)
        if (goal_dist < to_goal_tolerance):
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            vel_publisher.publish(vel_msg)
            break

        # This creates an array of data gained from sensor - in format of tuple with beam length and angle
        range_data = range_points_filter(laser_ranges, angle_increment, num_ranges)
        
        #** Gets the subgoal coordinates for the robot **#
        # Subgoal coordinates by Golden Search
        #subgoal_coord = golden_search_coordinate(sample_dist, range_data, goal_x, goal_y, -math.radians(30), math.radians(30), 0.01)
        # Subgol coordinates by Simplex Method
        subgoal_coord, velopt = simplex_method_coordinate(control_dist, range_data, goal_x, goal_y)

        #** VISUALIZATION **#
        if (visualization_activated):
            # Publishing marker for visualization
            rviz_visualization.publish_least_resistance_marker(marker_publisher, subgoal_coord)
            rviz_visualization.publish_control_contour(marker_publisher, sample_angles_dist, control_dist, yaw, x, y)
            # PUBLISHES start and the goal points
            rviz_visualization.publish_start_goal_marker(marker_publisher, start_x, start_y, goal_x, goal_y)
            global lantern_x, lantern_y
            # Publishes the lantern point
            rviz_visualization.publish_lantern_marker(marker_publisher, lantern_x, lantern_y)
            # Trajectory
            global path
            global path_publisher
            rviz_visualization.publish_trajectory_visualization(path, path_publisher, x, y)

        # Deciding angular speed to achieve the subgoal
        angular_speed = angular_speed_arch(velopt, subgoal_coord[0], subgoal_coord[1])
        vel_msg.linear.x = velopt
        vel_msg.angular.z = angular_speed

        # Distance to the subgoal (sample point)
        subgoal_dist = euclidianDistance(x,y, subgoal_coord[0], subgoal_coord[1])
        fraction_dist = subgoal_dist - subgoal_dist / 100

        # What would happen if this while loop is deleted? - probably smoother path, but potentionaly more complex calculation time - might not be the problem( solved by fewer sample points- triple method/ fewer range points)
        # It could be speed up by receiving already filtered range data - could be a separate node for it
        # Here watch out - it says distance to subgoal is bigger then... that distance always start at the distance of a 
        # sample point -> if we want to recalculate more often we need to put there largere distance to be smaller...
        # place here fraction of the sample_distance not 0.2 - neco jak  (subgoal_dist - subgoal_dist/100)
        while (subgoal_dist > fraction_dist):
            subgoal_dist = euclidianDistance(x,y, subgoal_coord[0], subgoal_coord[1])
            vel_publisher.publish(vel_msg)
            
            # Checks if it is in the ultimate goal
            if (goal_dist < to_goal_tolerance):
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                vel_publisher.publish(vel_msg)
                break

# # # # # # # # # # * * * * * * * * * Callback Functions * * * * * * * * * # # # # # # # # # #
# Callback functions for receiving information from the sensor and robot
def laserCallback(laser_msg):
    # laser_ranges is the list that contains information of range of each laser beam
    global laser_ranges
    # Angle increment between laser beams
    global angle_increment
    # Robot's one side maximum angle span - in the case of the turtlebot using Kinect sensor this angle is 30 degrees (from overall span of 60 degrees)
    global sensor_half_angle_span
    sensor_half_angle_span = laser_msg.angle_max

    # Accepts the information from the sensor's topic
    angle_increment = laser_msg.angle_increment
    laser_ranges = laser_msg.ranges

    #* Used sensor information - Kinect v1*#
    # In this case - the number of the range beams is 640 -> the lowest index = 0 and the highest = 639
    # The view of the sensor is 60 degrees (robot facing forward is 0 degrees and then 30 degrees on each side with negative sign on the left side)
    # Minimal distance it can see is 0.449 meters and the maximal is 10 meters, everything else out of this range is nan value

# Getting position of the robot with respect to the odometry
def odomPoseCallback(odom_message):
    global x
    global y, yaw

    # X, Y coordinate position of the robot (translation)
    x = odom_message.pose.pose.position.x
    y = odom_message.pose.pose.position.y

    # Quaternion orientation of the robot (orientation)
    qx = odom_message.pose.pose.orientation.x
    qy = odom_message.pose.pose.orientation.y
    qz = odom_message.pose.pose.orientation.z
    qw = odom_message.pose.pose.orientation.w

    # Quaternion as a list
    quaternion = (qx,qy,qz,qw)

    # Converting quaternion to roll, pitch and yaw - RPY
    rpy = tf.transformations.euler_from_quaternion(quaternion)

    # yaw describes the rotation of the robot with respect to odometry around z axis
    yaw = rpy[2]

# # # # # # # # # # * * * * * * * * * RUN Function * * * * * * * * * # # # # # # # # # #
# This function runs the whole program - initializes the ROS nodes, publishers, subscribers and runs move to goal function
def run(control_dist, linear_speed, chosen_control_span, num_ranges, goal_x, goal_y, in_time, vis_activation):
    #if __name__ == '__main__':
    try:
        # Initialisation of the node
        rospy.init_node('potential_fields_controller')
        # Creating publisher that talks to TurtleBot. cmd_vel = velocity command
        cmd_vel_topic = 'cmd_vel_mux/input/navi'
        cmd_vel = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)
        # Path publisher
        path_topic = '/path'
        global path_publisher
        global path
        path = Path()
        path.header.frame_id = '/odom'
        path.header.stamp = rospy.get_rostime()
        path_publisher = rospy.Publisher(path_topic, Path, queue_size=10)
        # Progress Contour publisher
        progress_contour_topic = '/progress_contour'
        global progress_contour_publisher
        global progress_path
        progress_path = Path()
        progress_path.header.frame_id = '/odom'
        progress_path.header.stamp = rospy.get_rostime()
        progress_contour_publisher = rospy.Publisher(progress_contour_topic, Path, queue_size=10)
        # Creating subscriber for the pose topic, every time a new pose info is registered it calls Callback function
        pose_topic = '/odom'
        pose_subscriber = rospy.Subscriber(pose_topic, Odometry, odomPoseCallback)
        # Scan topic for receiving distance information
        laser_topic = '/scan'
        laser_subscriber = rospy.Subscriber(laser_topic, LaserScan, laserCallback)
        # Sleep at the beginning in order to give it some time to receive global variables
        time.sleep(2)
        #MARKER publisher
        marker_topic = '/marker_topic'
        marker_publisher = rospy.Publisher(marker_topic, Marker, queue_size=1)
        # MAIN function that runs all other functions
        #move_to_goal(cmd_vel, marker_publisher, control_dist, linear_speed, chosen_control_span, num_ranges, goal_x, goal_y, vis_activation)

        # Run with threading - to be able to stop the robot by using GUI
        run_thread = threading.Thread(target=move_to_goal, args=(cmd_vel, marker_publisher, control_dist, linear_speed, chosen_control_span, num_ranges, goal_x, goal_y, in_time, vis_activation))
        run_thread.start()
    except rospy.ROSInterruptException:
        pass
# Function used by the Navigation GUI in order to stop the robot by setting the goal to the current position
def stop():
    global g_x, g_y
    global x, y
    g_x = x
    g_y = y
    # Restarts the program
    os.execl(sys.executable, sys.executable, *sys.argv)

run(0.5, 0.3, math.radians(60), 72, 10, 0, 20, True)
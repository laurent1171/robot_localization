#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud, Particle
from nav2_msgs.msg import Particle as Nav2Particle
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler
from angle_helpers import euler_from_quaternion
from numpy import array
import copy

#Class for defining particles with attributes x, y, theta, and their weighting w
class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    #helper function that converts a particles location and angle to a Neato pose (I think)
    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        # Odometry gives us the robot's orientation as a quaternion - we want theta, which is the yaw component 
        # Functions are provited for converting from quaternion to roll, pitch, yaw (we only care about yaw)
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0.0),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

    # TODO: define additional helper functions if needed
    
    # compute closest distance? using occupancy_field


class ParticleFilter(Node):
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            last_scan_timestamp: this is used to keep track of the clock when using bags
            scan_to_process: the scan that our run_loop should process next
            occupancy_field: this helper class allows you to query the map for distance to closest obstacle
            transform_helper: this helps with various transform operations (abstracting away the tf2 module)
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            thread: this thread runs your main loop
    """
    def __init__(self):
        super().__init__('pf')
        self.base_frame = "base_footprint"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 300          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update

        self.robot_pose = Pose()   #make sure this works/is needed?

        #self.occ_f = OccupancyField(Node) # how to do this?
        # TODO: define additional constants if needed

        #What are these two?
        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.update_initial_pose, 10)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(ParticleCloud, "particle_cloud", qos_profile_sensor_data)

        #subscription to laser scan on neato
        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """ This function takes care of sending out the map to odom transform """
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(seconds=0.1)
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, postdated_timestamp)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)


    def run_loop(self):
        """ This is the main run_loop of our particle filter.  It checks to see if
            any scans are ready and to be processed and will call several helper
            functions to complete the processing.
            
            You do not need to modify this function, but it is helpful to understand it.
        """
        #check if theres a new scan
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        #translate from particle pose to neato pose? or something like that?: we may need to code
        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           msg.header.stamp)
        #check if we got a new_pose
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0): #make sure 
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return
        
        #take turtlebot formatted scan and convert to polar neato lidar scan format?
        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(msg, self.base_frame)
        print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        #update odom_pose - estimation of where the robot is compared to where it started
        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()    # update based on odometry
            self.update_particles_with_laser(r, theta)   # update based on laser scan
            self.update_robot_pose()                # update robot's pose based on particles
            self.resample_particles()               # resample particles to focus on areas of high density
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    #Function for checking if the robot has moved far enough to update the particle cloud, current pose, etc.
    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh

    #function for estimating robot's pose and updating it
    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
        This method computes the mean pose from the particles.
        """
        # First, make sure the particle weights are normalized
        self.normalize_particles()

        # Initialize variables for mean position and orientation
        mean_x, mean_y, mean_theta = 0, 0, 0

        # Compute the weighted mean of the particles' states
        for particle in self.particle_cloud:
            mean_x += particle.x * particle.w
            mean_y += particle.y * particle.w
            mean_theta += particle.theta * particle.w

        # Convert mean_theta (yaw) to a quaternion for the Pose message
        orientation = quaternion_from_euler(0, 0, mean_theta)

        # Construct the robot's pose using the calculated mean values
        self.robot_pose = Pose(position=Point(x=mean_x, y=mean_y, z=0.0), 
                            orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]))

        # Check for the odom_pose attribute and handle the transformation
        if hasattr(self, 'odom_pose'):
            self.transform_helper.fix_map_to_odom_transform(self.robot_pose, self.odom_pose)
        else:
            self.get_logger().warn("Can't set map->odom transform since no odom data received")

    def update_particles_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        print("updating particles with odom")
        for i in range(self.n_particles):
            self.particle_cloud[i].x += delta[0]
            self.particle_cloud[i].y += delta[1]
            # Ensure the theta remains in [-pi, pi]
            self.particle_cloud[i].theta = (self.particle_cloud[i].theta + delta[2]) % (2 * np.pi)
            if self.particle_cloud[i].theta > np.pi:
                self.particle_cloud[i].theta -= 2 * np.pi

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample in helper_functions.py.
        """
        # make sure the distribution is normalized
        self.normalize_particles()
        
        # Extract the weights from the particle cloud
        weights = [particle.w for particle in self.particle_cloud]
        
        # Resample particles based on their weights
        # Using numpy's random.choice for this purpose
        indices = np.random.choice(len(self.particle_cloud), self.n_particles, p=weights)

        # Create a new particle cloud based on the sampled indices
        resampled_particles = [copy.deepcopy(self.particle_cloud[i]) for i in indices]

        self.particle_cloud = resampled_particles

    def update_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data
            r: the distance readings to obstacles
            theta: the angle relative to the robot frame for each corresponding reading 
        """
        # TODO: implement this - in progress, can't figure out how to use OccupancyField

        #double check syntax
        distances = [0.0]*self.n_particles
        closest_laser_dist = min(r)

        for i in range (self.n_particles):
            dist = OccupancyField.get_closest_obstacle_distance(self, self.particle_cloud[i].x, self.particle_cloud[i].y) #having trouble using occupancy field - need to load map?
            w = dist - closest_laser_dist
            self.particle_cloud[i].w = w
        
        self.normalize_particles()

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose) #msg.pose.pose is a position and an orientation, xyzw location and quaternion orientation
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """
        if xy_theta is None:  
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        self.particle_cloud = [None]*self.n_particles
        # TODO create particles - (done)
        #create 100 Particle instances with x, y, theta, w = 1.0

        #use numpy to create normalized distributions for x, y, theta
        #s = np.random.normal(mu, sigma, 1000)
        x_robot = self.robot_pose.position.x
        y_robot = self.robot_pose.position.y
        #roll_x, pitch_y, yaw_z = euler_from_quaternion(self.robot_pose)
        #theta_robot = yaw_z
        yaw_z = 0
        theta = np.random.normal(yaw_z, 0.17,self.n_particles)  #we think theta is in radians?
        x = np.random.normal(x_robot, 0.5,self.n_particles) #change 0 to the current pose of the robot (done)
        y = np.random.normal(y_robot, 0.5,self.n_particles)   #here as well

        #generate list of n_particles particles
        for i in range (self.n_particles):
            self.particle_cloud[i]= Particle(x[i],y[i], theta[i], 1.0)

        print("self.particle_cloud[2].w is", self.particle_cloud[2].w)

        self.normalize_particles()
        self.update_robot_pose()

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        # TODO: implement this (done) 

        particle_cloud_np = self.particle_cloud
        #print ("test,", self.particle_cloud[1].w)
        weights = [0.0]*self.n_particles
        for i in range (self.n_particles):
            #print("self.particle_cloud[1].w is", self.particle_cloud[1].w)
            weights[i] = (self.particle_cloud[i].w)

        #take sum of all weights
        total = sum(weights)
        print("total is ", total)

        #divide each weight by the total sum
        division_array =array([total]*self.n_particles) #create an array for matrix division
        #print("division array, ", division_array)
        #print("weights, ", weights)
        normalized_weights = np.divide(weights,division_array) #normalize by performing matrice division
        #print("normalized weights, ", normalized_weights)

        #reassign to each particle
        for i in range (self.n_particles):
            self.particle_cloud[i].w = normalized_weights[i] #assign normalize values back to particle_cloud
        
        #print("self.particle_cloud, ", self.particle_cloud)

    def publish_particles(self, timestamp):
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        for p in self.particle_cloud:
            msg.particles.append(Nav2Particle(pose=p.as_pose(), weight=p.w))
        self.particle_pub.publish(msg)

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop 
        if self.scan_to_process is None:
            self.scan_to_process = msg

def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

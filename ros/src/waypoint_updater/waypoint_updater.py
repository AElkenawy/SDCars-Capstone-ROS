#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''
# Number of waypoints to be published
LOOKAHEAD_WPS = 75 

# Maximum deceleration
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        
        rospy.init_node('waypoint_updater')
        
        ###* Subscribers declaration *###
        
        # /current_pose subscriber 
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        # Latched subscriber to store waypoints once only
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        # /traffic_waypoint subscriber
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Member variables declaration
        self.pose = None
        self.base_lane = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        
        # final_waypoints publisher
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        self.loop()
    
    # Loop function to target publishing frequency of around 50 hertz 
    def loop (self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()
                
    # Function to get closest waypoint ahead of vehicle
    def get_closest_waypoint_idx (self):
        
        # Vehicle coordinates
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        
        # Position and index of the closest point in the KDTree
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check whether closest point is ahead/front vehicle (dot product)
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]
        
        # Equation for hyperplane through closest_coord
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        
        # Car pose hyperplane
        pos_vect = np.array([x,y])
        
        # Preforming dot product
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        
        # If product is positive >> point/cl_vect is behind of vehicle/pos_vect) 
        if val > 0:
            # Take the next point (idx + 1)
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx
    
    # Function to publish waypoints
    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
   
    # Update waypoints based on desired behavior (stop/move) and current status
    def generate_lane(self):
        
        # Create Lane object
        lane = Lane()
        
        # Get closest waypoint (get_closest_waypoint_idx function)
        closest_idx = self.get_closest_waypoint_idx()
        
        # Waypoints starting from closest point till desired number (LOOKAHEAD_WPS)
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx : farthest_idx]
        
        # No traffic light is detected/not reached yet
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        
        # traffic light is detected, start the decelerate_waypoints routine
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        
        return lane
        
    # Function to clone base waypoints then modify necesary waypoints
    # to perform deceleration (in case of a traffic light detected)
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        
        # Enumerate through base waypoints
        for i, wp in enumerate (waypoints):
            
            # Create new waypoint message
            p = Waypoint()
            
            # Store current waypoints pose
            p.pose = wp.pose
            
            # Two waypoints back from traffic stop line to 
            # keep car front stops behind stop line
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            
            # Distance between current waypoint index and stop index 
            # using linear piecewise distance
            dist = self.distance(waypoints, i, stop_idx)
            
            # Mapping velocity value to current distance till stop index
            vel = math.sqrt(2 * MAX_DECEL * dist)
            
            # Return zero velocity when vel is small enough
            if vel < 1.0:
                vel = 0.0
            
            # Keep the speed limit in case of large distance between 
            # current waypoint and stop index
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        
        return temp
            
    
    # cb function to store the car's pose (around 50 hertz)
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        
    # cb function to store waypoints
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # Store waypoints in self.base_waypoints object
        self.base_lane = waypoints
        
        # Make sure to intialize self.waypoints2D before the subscriber
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            ''' Efficient spatial binary search to find points near the car by
            partitioning total points into k-dimentional sections '''
            self.waypoint_tree = KDTree(self.waypoints_2d)
    
    # cb function to stop waypoint index
    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

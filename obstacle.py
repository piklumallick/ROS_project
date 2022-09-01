#!/usr/bin/env python
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan #import library for lidar sensor
from nav_msgs.msg import Odometry #import library for position and orientation data
from geometry_msgs.msg import Twist

class Circling(): #main class
   
    def __init__(self): #main function
        global circle
        circle = Twist() #create object of twist type  
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) #publish message
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback) #subscribe message 
        self.sub = rospy.Subscriber("/odom", Odometry, self.odometry) #subscribe message

    def callback(self, msg): #function for obstacle avoidance
        print ('-------RECEIVING LIDAR SENSOR DATA-------')
        print ('Front: {}'.format(msg.ranges[0])) #lidar data for front side
        print ('Left:  {}'.format(msg.ranges[90])) #lidar data for left side
        print ('Right: {}'.format(msg.ranges[270])) #lidar data for right side
        print ('Back: {}'.format(msg.ranges[180])) #lidar data for back side
      
      	#Obstacle Avoidance
        self.distance = 0.7
        if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance: 
        #when no any obstacle near detected
            circle.linear.x = 0.5 # go (linear velocity)
            circle.angular.z = 0.1 # rotate (angular velocity)
            rospy.loginfo("Circling") #state situation constantly
        else: #when an obstacle near detected
            rospy.loginfo("An Obstacle Near Detected") #state case of detection
            circle.linear.x = 0.0 # stop
            circle.angular.z = 0.5 # rotate counter-clockwise
            if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
                #when no any obstacle near detected after rotation
                circle.linear.x = 0.5 #go
                circle.angular.z = 0.1 #rotate
        self.pub.publish(circle) # publish the move object

    def odometry(self, msg): #function for odometry
        print (msg.pose.pose) #print position and orientation of turtlebot a quaternion is a complex number with w as the real part and x, y, z as imaginary parts.

   #############################################Regarding W in orientation######################
#If a quaternion represents a rotation then w = cos(theta / 2), where theta is the rotation angle around the axis of the quaternion.
#The axis v(v1, v2, v3) of a rotation is encoded in a quaternion: **x = v1 sin (theta / 2), y = v2 sin (theta / 2), z = v3 sin (theta / 2)*.
#If w is 1 then the quaternion defines 0 rotation angle around an undefined axis v = (0,0,0).
#If w is 0 the quaternion defines a half circle rotation since theta then could be +/- pi.
#If w is -1 the quaternion defines +/-2pi rotation angle around an undefined axis v = (0,0,0).
#A quater circle rotation around a single axis causes w to be +/- 0.5 and x/y/z to be +/- 0.5.
######################################################################################################

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node') #initilize node
    Circling() #run class
    rospy.spin() #loop it

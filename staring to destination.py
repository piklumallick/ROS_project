import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
import math
PI = math.pi

class ROSEnter:
    def __init__(self):
        # Starts a new node
        rospy.init_node('speed_controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber("/odom", Odometry, self.get_orientation)
        self.x = 0
        self.y = 0
        self.theta = 0

    def get_orientation(self, msg):
        # Update position and orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation

        # print("theta: ", self.theta)
        _, _, self.theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        if(self.theta < 0):
            self.theta += 2*PI
        

    def move_to(self,x,y):
        #Starts a new node
        vel_msg = Twist()
        # print("vel_msg ", vel_msg)
        rate = rospy.Rate(100)

        goal = Point()
        goal.x = x
        goal.y = y

        # print("Starting Coordinate of Robot:")
        # print("X : ", self.x)
        # print("Y : ", self.y)

        # print("Expected final destination Coordinate:")
        # print("X : ", goal.x)
        # print("Y : ", goal.y)

	# calculating angle between current point and target point
        inc_x = goal.x - self.x
        inc_y = goal.y - self.y
        relative_angle = math.atan2(inc_y, inc_x)
        # print(" **************** Rotating robot at a position *****************")
        # print()
        if relative_angle < 0:
            relative_angle = 2*PI + relative_angle
        if relative_angle > 2*PI:
            relative_angle -= 2*PI
        print("Expected angle rotation for Robot : ",relative_angle)

        # speed = 40
        ang_vel = 0.25
        vel_msg.angular.z = ang_vel
        current_angle = 0
        while(current_angle < relative_angle):
            self.pub.publish(vel_msg)
            rate.sleep()
            current_angle = self.theta

        #Forcing our robot to stop
        print("Actual angle rotation of Robot : ", self.theta)
        vel_msg.angular.z = 0
        self.pub.publish(vel_msg)

        print("Angular Velocity : ",ang_vel)
        
        # Move now in that direction
        # rate = rospy.Rate(10)
        print()
        print("Moving robot in straight Line at that Direction")
        speed = 0.04
        dis = math.sqrt((inc_x**2) + (inc_y**2))
        vel_msg.linear.x=abs(speed)
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        
        current_dis = 0
        print("Expected Distance covered by Robot: ", dis)
        while(current_dis < dis):
            self.pub.publish(vel_msg)
            rate.sleep()
            current_dis = math.sqrt((self.x**2) + (self.y**2))
    
        print("Actual Distance covered by Robot : ",current_dis)

        print()
        print("************ Robot reach at his destination ****************")
        vel_msg.linear.x = 0
        self.pub.publish(vel_msg)

        print()
        print("Actual final destination Coordinate:")
        print("X : ", self.x)
        print("Y : ", self.y)

        return



if __name__ == '__main__':
    ros_inter = ROSEnter()
    x = 2
    y = 1
    ros_inter.move_to(x, y)



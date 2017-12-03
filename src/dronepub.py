#!/usr/bin/env python
# license removed for brevity
import roslib
import rospy
import math
import tf
from std_msgs.msg import String
from std_msgs.msg import Char
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from ardrone_autonomy.msg import Navdata

#Variables initialized
cmd_vel = Twist()
nav_data = Navdata()
imu_data = Imu()
empty=Empty()

def main():
    #Intialize the ROS Node
    rospy.init_node('drone_control', anonymous=True)

    #Publishers Initialized
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10, latch=True)
    takeoff_pub = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1, latch=True)
    land_pub = rospy.Publisher('ardrone/land', Empty, queue_size=1, latch=True)
    reset_pub = rospy.Publisher('ardrone/reset', Empty, queue_size=1, latch=True)

    
    #TF listener initialized
    listener = tf.TransformListener()

    #Reset the drone 
    rospy.loginfo("Drone Resetting: Please step away")   
    reset_pub.publish(empty)
    rospy.sleep(5.0)
    
    #Takeoff the drone first
    rospy.loginfo("Drone taking off")   
    takeoff_pub.publish(empty)
    rospy.sleep(5.0)


    #The Control loop for navigation
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('ardrone/base_link', 'ardrone/front', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #Subscribers Initialized
        rospy.Subscriber("ardrone/navdata", Navdata, nav_callback)
        rospy.Subscriber("ardrone/imu", Imu, imu_callback)

        ## hello_str = "hello world %s" % rospy.get_time()
      #  cmd_vel.linear.x=0.0    
      #  cmd_vel.linear.y=0.0    
      #  cmd_vel.linear.z=0.0    
      #  cmd_vel.angular.x=0.0    
      #  cmd_vel.angular.y=0.0    
      #  cmd_vel.angular.z=0.0    
      #  rospy.loginfo(cmd_vel)
      #  vel_pub.publish(cmd_vel)
      #  rate.sleep(2.0)

      #  cmd_vel.linear.x=0.0    
      #  cmd_vel.linear.y=0.0    
      #  cmd_vel.linear.z=0.0    
      #  cmd_vel.angular.x=0.0    
      #  cmd_vel.angular.y=0.0    
      #  cmd_vel.angular.z=0.0    
      #  rospy.loginfo(cmd_vel)
      #  vel_pub.publish(cmd_vel)
       # rate.sleep(2.0)
      #  takeoff_pub.publish(takeoff)
       # rate.sleep(5.0)
       # land_pub.publish(land)
       # rate.sleep(5.0)
       # rospy.loginfo(rospy.get_caller_id())
         

def nav_callback(data):
    nav_data = data

def imu_callback(data):
    imu_data = data

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

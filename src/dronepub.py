#!/usr/bin/env python
# license removed for brevity
import roslib
import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Char
from std_msgs.msg import Empty
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from ardrone_autonomy.msg import Navdata

rospy.init_node('talker', anonymous=True)
#vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1, latch=true)
takeoff_pub = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1, latch=True)
land_pub = rospy.Publisher('ardrone/land', Empty, queue_size=10)
cmd_vel = Twist()
nav_data = Navdata()
imu_data = Imu()
takeoff=Empty()
land=Empty()
takeoff_pub.publish(takeoff)
def talker():
   # rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("ardrone/navdata", Navdata, nav_callback)
    rospy.Subscriber("ardrone/imu", Imu, imu_callback)
    listener = tf.TransformListener()
    #rospy.loginfo(rospy.get_caller_id())
    rate = rospy.Rate(1) # 10hz
   # takeoff_pub.publish(takeoff)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('ardrone/base_link', 'ardrone/front', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
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
   # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.temp)
    nav_data = data

def imu_callback(data):
   # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    imu_data = data

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

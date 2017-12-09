#!/usr/bin/env python
# license removed for brevity
import roslib
import rospy
import math
import tf
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Char
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from ardrone_autonomy.msg import Navdata
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker


#Variables initialized
cmd_vel = Twist()
nav_data = Navdata()
imu_data = Imu()
odom = Odometry()
empty=Empty()
ar_data = AlvarMarkers()

testing = True

cx = 0.0
cy = 0.0
cz = 0.0

markerFlag = 0
statusFlag = 1
adjHeight = 0
adjX = 0
adjY = 0
init_goalx = 0
init_goaly = 0
init_goalz = 0
fist_sighting = True


# quoternion to rotation matrix
def quatToRot(x, y, z, w):
    R = np.array([[1-2*y**2-2*z**2, 2*x*y-2*z*w, 2*x*z+2*y*w],
                  [2*x*y+2*z*w, 1-2*x**2-2*z**2, 2*y*z-2*x*w],
                  [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x**2-2*y**2]])
    return R

def x_y_adjustment():


def main():
    #Intialize the ROS Node
    rospy.init_node('drone_control', anonymous=True)

    #Publishers Initialized
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10, latch=True)
    takeoff_pub = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1, latch=True)
    land_pub = rospy.Publisher('ardrone/land', Empty, queue_size=1, latch=True)
    reset_pub = rospy.Publisher('ardrone/reset', Empty, queue_size=1, latch=True)



    # ar subscribe
 #   rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback )

    #TF listener initialized
    listener = tf.TransformListener()

    #Reset the drone
    rospy.loginfo("Drone Resetting: Please step away")
    if not testing:
        reset_pub.publish(empty)
    rospy.sleep(1.0)

    #Takeoff the drone first
    rospy.loginfo("Drone taking off")
    if not testing:
        takeoff_pub.publish(empty)
    rospy.sleep(1.0)


    #The Control loop for navigation
    while not rospy.is_shutdown():
#       # try:
        #    (trans,rot) = listener.lookupTransform('ardrone/base_link', 'ardrone/base_frontcam', rospy.Time(0))
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #    rospy.loginfo("exception")
        #    continue

    #    (trans,rot) = listener.lookupTransform('ardrone/base_link', 'ardrone/base_frontcam', rospy.Time(0))
        #Subscribers Initialized
        rospy.sleep(1.0)
        rospy.Subscriber("ardrone/navdata", Navdata, nav_callback)
        rospy.Subscriber("ardrone/odometry", Odometry, odom_callback)
        rospy.Subscriber("ardrone/imu", Imu, imu_callback)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback )

        #rospy.loginfo(['in main X', adjX, ' Y', adjY, ' Z', adjHeight, ' Marker', markerFlag])


        if markerFlag == 0:
            rospy.loginfo("Searching")
            cmd_vel.angular.z  = 0.1
            rospy.loginfo("Rotating")
            if not testing:
                vel_pub.publish(cmd_vel)
        if adjX == 1:
            rospy.loginfo("adjusting X")
            if centerx > 0:
                cmd_vel.linear.x = -0.1
            else:
                cmd_vel.linear.x = 0.1

            if not testing:
                vel_pub.publish(cmd_vel)

        if adjY == 1:
            rospy.loginfo("adjusting Y")
            if centery > 3.5:
                cmd_vel.linear.y = 0.1
            else:
                cmd_vel.linear.y = -0.1

            if not testing:
                vel_pub.publish(cmd_vel)

        if adjustHeight == 1:
            pass





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
def ar_callback(data):
    global markerFlag
    global statusFlag
    global adjHeight
    global adjX
    global adjY
    global centerx
    global centery
    global centerz
    global init_goalx
    global init_goaly
    global init_goalz
    global first_sighting

    ar_data = data
    rot = np.array([0.0150814330344, -0.020624200866, -0.544323169626, 0.838486421908])
    trans = np.array([-0.0841731084939, 0.133747005225, 0.0])

    #rospy.loginfo(ar_data.markers)
    #rospy.loginfo(ar_data)
    #rospy.loginfo(len(ar_data.markers))

    #rospy.loginfo(['in callback X', adjX, ' Y', adjY, ' Z', adjHeight, ' Marker', markerFlag])


    if len(ar_data.markers) == 4:
        t0 = ar_data.markers[0].pose.pose.position
        t1 = ar_data.markers[1].pose.pose.position
        t2 = ar_data.markers[2].pose.pose.position
        t3 = ar_data.markers[3].pose.pose.position

        cx = -1*(t0.x + t1.x + t2.x + t3.x)/4
        cz = (t0.y + t1.y + t2.y + t3.y)/4
        cy = (t0.z + t1.z + t2.z + t3.z)/4

        c = np.array([cx, cy, cz])
    # if rotation is quoternion, convert to matrix
        if len(rot) == 4:
            rot = quatToRot(rot[0], rot[1], rot[2], rot[3])
            center = np.dot(rot, c) + trans #might need to adjust

        centerx = c[0]
        centery = c[1]
        centerz = c[2]
        rospy.loginfo(cz)


        if first_sighting:
            init_goalx = 0
            init_goaly = centery /2
            init_goalz = centerz
            statusFlag = 1
            first_sighting = False


        markerFlag = 1

        if abs()

        # if abs(centerz) > .2:
        #     adjHeight = 1
        # else:
        #     adjHeight = 0
        #     cmd_vel.linear.z = 0
        #     if not testing:
        #         vel_pub.publish(cmd_vel)

        if statusFlag == 1 and not first_sighting:

        if adjX == 0 and abs(centerx) > .3: #might have to adjust.
            adjX = 1
        else:
            adjX = 0
            cmd_vel.linear.x = 0
            if not testing:
                vel_pub.publish(cmd_vel)

        if adjY == 0 and abs(centery) > 3.5:
            adjY = 1
        else:
            adjY = 0
            cmd_vel.linear.y = 0
            if not testing:
                vel_pub.publish(cmd_vel)



        if abs(centerz) < .2 and abs(centerx) < .2 and abs(centery) < 1.5:
            # fix orientation

            rospy.loginfo('close to window fix orientation')



    else:
        markerFlag= 0

def nav_callback(data):
    nav_data = data

def imu_callback(data):
    imu_data = data

def odom_callback(data):
    odom = data

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import LaserScan
import time
from itertools import count
import tf
from tf.transformations import quaternion_from_euler
import numpy as np

index = count(step=1)  #### time counter 

pub_laser = rospy.Publisher("sonar_laser", LaserScan, queue_size=100)
pub_servo = rospy.Publisher("servo", UInt16, queue_size=10)

a = [0] * 180  # Initialize an array of 180 distances

def callback1(data):
    global w1
    q = data.data
    w = q / 100
    if w > 4:
        w = 4
    elif w == 0:
        w = 0.01
    w1 = round(w, 3)
    print(w1)

scan_time = 0.1
def laser():
    broadcaster = tf.TransformBroadcaster()  # Initialize transform broadcaster
    time_begin = rospy.Time.now()
    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        time_end = rospy.Time.now()

        for i in range(180):
            pub_servo.publish(i)  # Rotate servo from 0 to 179 degrees
            time.sleep(scan_time)

            a[i] = w1  # Update the distance readings array
            laser = LaserScan()
            Time = next(index)
            d = time_end - time_begin
            laser.header.seq = d
            laser.header.stamp = time_end
            laser.header.frame_id = 'sonar'
            laser.angle_min = 0 * np.pi / 180
            laser.angle_max = 180 * np.pi / 180
            laser.angle_increment = np.pi / 180
            laser.time_increment = 0.001
            laser.scan_time = 0.1
            laser.range_min = 0.01
            laser.range_max = 4
            laser.ranges = a  # Assign the updated distance readings
            laser.intensities = []

            pub_laser.publish(laser)

            # Publish the transform from gripper to sonar frame
            broadcaster.sendTransform(
                (0, 0, 0),  # Position of the sonar sensor
                quaternion_from_euler(0, 0, i * np.pi / 180),  # Rotation for the servo
                rospy.Time.now(),
                "sonar",  # The frame of the sensor
                "grip"    # The parent link/frame
            )

        r.sleep()

rospy.Subscriber('/sonar', Float64, callback1)
rospy.init_node('laser', anonymous=True)

if __name__ == '__main__':
    try:
        laser()
    except rospy.ROSInterruptException:
        pass


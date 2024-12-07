#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, UInt16, Bool ,String
from sensor_msgs.msg import LaserScan
import time
from itertools import count
import tf
from tf.transformations import quaternion_from_euler
import numpy as np

# Initialize the ROS node 
rospy.init_node('laser', anonymous=True)

# Initialize global variables
index = count(step=1)  # Time counter
w1 = 0  # Distance from sonar
a = [0] * 180  # Array of 180 distances
is_target = False  # Flag for YOLO target detection

# Publishers
pub_laser = rospy.Publisher("sonar_laser", LaserScan, queue_size=100)
pub_servo = rospy.Publisher("servo", UInt16, queue_size=10)
pub_stop = rospy.Publisher("stop_robot", String, queue_size=10)

def callback1(data):
    """Callback for sonar distance data."""
    global w1
    q = data.data
    w = q / 100  # Convert distance to meters
    if w > 4:
        w = 4
    elif w == 0:
        w = 0.01
    w1 = round(w, 3)

def yolo_callback(data):
    """Callback for YOLO model detection."""
    global is_target
    is_target = data.data  # Boolean indicating if the target is detected

scan_time = 0.1

def laser():
    broadcaster = tf.TransformBroadcaster()  # Initialize transform broadcaster
    time_begin = rospy.Time.now()  # Now that init_node is called, we can use ROS time
    r = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        time_end = rospy.Time.now()

        for i in range(180):
            # Rotate servo to the current angle
            pub_servo.publish(i)
            time.sleep(scan_time)

            # Update distance array and check for obstacle
            a[i] = w1  # Update the distance reading
            if w1 <= 0.2:  # Distance less than or equal to 20 cm
                rospy.loginfo(f"Object detected at {w1 * 100} cm, angle: {i}°")
                pub_stop.publish("STOP")  # Publish stop signal
                
                # Wait for YOLO detection
                rospy.sleep(1)  # Give some time to process the YOLO data
                if is_target:
                    rospy.loginfo("Target detected! Proceeding with target handling.")
                else:
                    rospy.loginfo("Obstacle detected! Avoiding obstacle.")

            # Create LaserScan message
            laser = LaserScan()
            d = time_end - time_begin
            laser.header.seq = next(index)
            laser.header.stamp = time_end
            laser.header.frame_id = 'sonar'
            laser.angle_min = 0 * np.pi / 180
            laser.angle_max = 180 * np.pi / 180
            laser.angle_increment = np.pi / 180
            laser.time_increment = 0.001
            laser.scan_time = 0.1
            laser.range_min = 0.01
            laser.range_max = 4
            laser.ranges = a  # Assign updated distances
            laser.intensities = []

            # Publish the LaserScan data
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

# Subscribers
rospy.Subscriber('/sonar', Float64, callback1)
rospy.Subscriber('/yolo_detection', Bool, yolo_callback)  # Listen to YOLO topic for boolean

# Initialize the ROS node
rospy.init_node('laser', anonymous=True)

if __name__ == '__main__':
    try:
        laser()
    except rospy.ROSInterruptException:
        pass

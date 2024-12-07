## arduino setup for rosserial
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

## terminal command to open rosserial
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600

## transformation package basics
https://articulatedrobotics.xyz/tutorials/ready-for-ros/tf/

## `sequence.py`
1- ROS Nodes:

    laser Node:
        This is the main node that you have created, named laser.
        It is responsible for rotating a servo, reading distance data from a sonar sensor, publishing laser scan data, and broadcasting transforms.
        The laser node also listens to the /sonar and /yolo_detection topics, processes the data, and sends control messages like stopping the robot if an obstacle is detected.

2- Topics:

    /sonar:
        Type: Float64
        Purpose: This topic receives the sonar sensor's distance readings (in centimeters), which are converted to meters in the callback1 function. The distance data is processed and used to detect obstacles.
        Subscriber: callback1(data) subscribes to this topic, processes the distance data, and stores it in the global variable w1.

    /yolo_detection:
        Type: Bool
        Purpose: This topic receives a Boolean value indicating whether the YOLO object detection model has detected a target or not.
        Subscriber: yolo_callback(data) subscribes to this topic and updates the is_target flag based on the detection status.

    sonar_laser:
        Type: LaserScan
        Purpose: This topic publishes LaserScan data, which represents the sensor's distance readings at different angles. It includes the sonar data processed into a LaserScan message.
        Publisher: pub_laser publishes the LaserScan data, which is used to visualize the sonar sensor's range in a ROS-based visualization tool (e.g., RViz).

    servo:
        Type: UInt16
        Purpose: This topic publishes the servo's angle to rotate the servo motor. The angle is published in steps, and the servo rotates accordingly.
        Publisher: pub_servo publishes an angle to this topic to control the servo.

    stop_robot:
        Type: String
        Purpose: This topic sends a command (such as "STOP") to stop the robot when an obstacle is detected.
        Publisher: pub_stop publishes a stop signal to this topic.


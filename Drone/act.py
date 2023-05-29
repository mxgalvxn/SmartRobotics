#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform, Pose

odometry = Pose()

def odometry_callback(data):
    global odometry
    odometry = data.data

def main():
    rospy.init_node('node_example')
    waypoint_publisher = rospy.Publisher('/hummingbird/command/trajectory',  MultiDOFJointTrajectory, queue_size= 10)
    sub_odometry = rospy.Subscriber('hummingbird/odometry_sensor1/pose', Pose , odometry_callback)
    #generar sussribver que escuche odometry
    #encuentre la posiscion actual y ese sea el punto inicial de la trayectoria
    #odometru sensor
    #odometry ground truth 


    drone_msg = MultiDOFJointTrajectory()
    drone_msg.header.stamp = rospy.Time.now()
    drone_msg.header.frame_id = 'base link'


    drone_msg.joint_names.append('base link')

    #creat starting point
    transforms =Transform()
    velocities = Twist()
    acceleration = Twist()

    transforms.translation.x = odometry.position.x
    transforms.translation.y = odometry.position.y
    transforms.translation.z = 1.0   

    point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [acceleration], rospy.Time(1))
    drone_msg.points.append(point)

    #create end point
    transforms = Transform()
    transforms.translation.x = 1.0
    transforms.translation.y = 1.0
    transforms.translation.z = 1.0

    transforms.translation.w = 1.0
    transforms.translation.x = 0.0
    transforms.translation.y = 0.0
    transforms.translation.z = 0.0

    velocities = Twist()
    acceleration = Twist()

    point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [acceleration], rospy.Time(2))
    waypoint_publisher.publish(drone_msg)




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

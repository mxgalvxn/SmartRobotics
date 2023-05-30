#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform, Pose
import math as m
import time

odometry = Pose()
xrd=[]
yrd=[]
zrd=[]

def points_generator():
    for i in range(1800):
        #xrd.append(2*i)
        xrd.append(2.0*m.cos(2.0 * m.pi * i / 180.0))
        yrd.append(2.0*m.sin(2.0 * m.pi * i / 180.0))
        zrd.append(i/180.0 + 1.0)
    

# def odometry_callback(data):
#     global odometry
#     odometry = data.position

def main():
    rospy.init_node('node_example')
    nodeRate = 8
    rate = rospy.Rate(nodeRate)
    waypoint_publisher = rospy.Publisher('/hummingbird/command/trajectory',  MultiDOFJointTrajectory, queue_size= 10)
    odometry = rospy.wait_for_message('/hummingbird/odometry_sensor1/pose', Pose, timeout=1)
    #sub_odometry = rospy.Subscriber('/hummingbird/odometry_sensor1/pose', Pose , odometry_callback)
    #generar sussribver que escuche odometry
    #encuentre la posiscion actual y ese sea el punto inicial de la trayectoria
    #odometru sensor
    #odometry ground truth 


    drone_msg = MultiDOFJointTrajectory()
    drone_msg.header.stamp = rospy.Time.now()
    drone_msg.header.frame_id = 'base link'
    drone_msg.joint_names.append('base link')

   
    points_generator()


    print()
    #print(len(yrd))
    #cimport timereate end point
    tiempo = .5
    
    while not rospy.is_shutdown():
         #creat starting point
        transforms =Transform()
        velocities = Twist()
        acceleration = Twist()


        transforms.translation.x = odometry.position.x
        #print(odometry.position.x)  
        transforms.translation.y = odometry.position.y
        #print(odometry.position.y)
        transforms.translation.z = 1.0  
        #print(odometry.position.z)
        point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [acceleration], rospy.Time(tiempo))
        drone_msg.points.append(point)

        transforms = Transform()
        i = 0
        while i < (len(xrd)):
            transforms.translation.x = xrd[i]
            transforms.translation.y = yrd[i]
            transforms.translation.z = zrd[i]
            # print('/////////////////////////////////')
            # print('')
            # print('x actual', ( odometry.position.x), 'x deseada', xrd[i])
            # print('')
            # print('y actual', ( odometry.position.y), 'y deseada', yrd[i])

            transforms.rotation.w = 1.0
            transforms.rotation.x = 0.0
            transforms.rotation.y = 0.0
            transforms.rotation.z = 0.0

            velocities = Twist()
            acceleration = Twist()
            point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [acceleration], rospy.Time(tiempo))
            drone_msg.points.append(point)
            waypoint_publisher.publish(drone_msg)
            odometry = rospy.wait_for_message('/hummingbird/odometry_sensor1/pose', Pose, timeout=1)
            
            distance = m.sqrt((xrd[i] - odometry.position.x)**2 + ( yrd[i] - odometry.position.y)**2)
            print(distance)
            if (distance) < 0.3:
                i+=1
                print(i)
            rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

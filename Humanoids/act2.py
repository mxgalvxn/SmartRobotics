#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import pinocchio

floating_base_pose = None
joint_states = None

def floating_base_pose_callback(msg):
    global floating_base_pose
    floating_base_pose = msg.pose

def joint_states_callback(msg):
    global joint_states
    joint_states = msg.position

def run_node():
    rospy.init_node('pinocchio_example_node')

    urdf_filename = '/home/niger/reemc_public_ws/src/whole_body_state_msgs/urdf/reemc_full_ft_hey5.urdf'
    free_flyer = pinocchio.JointModelFreeFlyer()
    model = pinocchio.buildModelFromUrdf(urdf_filename, free_flyer)
    rospy.loginfo('Model name: ' + model.name)

    data = model.createData()

    def update_forward_kinematics(q):
        pinocchio.forwardKinematics(model, data, q)

        for name, oMi in zip(model.names, data.oMi):
            rospy.loginfo(("{:<24} : {: .3f} {: .3f} {: .3f}"
                          .format(name, *oMi.translation.T)))

    rospy.Subscriber('/floating_base_pose_simulated', PoseStamped, floating_base_pose_callback)
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if floating_base_pose is not None and joint_states is not None:
            q = pinocchio.Quaternion(floating_base_pose.orientation.w,
                                     floating_base_pose.orientation.x,
                                     floating_base_pose.orientation.y,
                                     floating_base_pose.orientation.z).matrix()
            q = q.flatten()
            q = list(joint_states) + list(q)

            update_forward_kinematics(q)

            floating_base_pose = None
            joint_states = None

        rate.sleep()

if __name__ == '__main__':
    try:
        run_node()
    except rospy.ROSInterruptException:
        pass

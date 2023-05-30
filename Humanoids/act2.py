#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import pinocchio

# Variables globales para almacenar el estado de configurac
joint_states = JointState()
floating_base_pose = Odometry()
#global joint_states

def floating_base_pose_callback(msg):
    global floating_base_pose
    floating_base_pose = msg



def joint_states_callback(msg):
    global joint_states
    joint_states = msg
    
def jointQ(q):
    q[7:7 +len(joint_states.position)] = joint_states.position

def floatingPoseQ(q):
    position = floating_base_pose.pose.pose.position
    orientation = floating_base_pose.pose.pose.orientation
    
    q[0] = position.x
    q[1] = position.y
    q[2] = position.z
    q[3] = orientation.x
    q[4] = orientation.y
    q[5] = orientation.z
    q[6] = orientation.w

def run_node():
    rospy.init_node('pinocchio_example_node')

    urdf_filename = '/home/humanoid/reemc_public_ws/src/whole_body_state_msgs/urdf/reemc_full_ft_hey5.urdf'
    free_flyer = pinocchio.JointModelFreeFlyer()
    model = pinocchio.buildModelFromUrdf(urdf_filename, free_flyer)
    rospy.loginfo('Model name: ' + model.name)

    pose = rospy.Subscriber('/floating_base_pose_simulated', Odometry, floating_base_pose_callback)
    joint = rospy.Subscriber('/joint_states', JointState, joint_states_callback)


    # Create data required by the algorithms
    data = model.createData()

    # Home configuration
    q = pinocchio.neutral(model)

    jointQ(q)
    floatingPoseQ(q)
 
    rospy.loginfo('q: %s' % q.T)
    
    
     

    # Perform the forward kinematics over the kinematic tree
    pinocchio.forwardKinematics(model, data, q)
    floating_base_pose = None
    joint_states = None

    # Print out the placement of each joint of the kinematic tree
    for name, oMi in zip(model.names, data.oMi):
        rospy.loginfo(("{:<24} : {: .3f} {: .3f} {: .3f}"
                      .format(name, *oMi.translation.T)))

    
    rate = rospy.Rate(10)  # Frecuencia de actualizacin del estado de configuracin

    while not rospy.is_shutdown():
        # Verifica si los tpicos han recibido datos
        if floating_base_pose is not None and joint_states is not None:
            # Actualiza el estado de configuracin con los valores de los tpicos
            # Puedes realizar las acciones que desees con los valores, como imprimirlos en el terminal
            rospy.loginfo("Floating base pose: %s", floating_base_pose)
            rospy.loginfo("Joint states: %s", joint_states)

            # Reinicia las variables para esperar nuevos datos
            floating_base_pose = None
            joint_states = None

        rate.sleep()

if __name__ == '__main__':
    try:
        run_node()
    except rospy.ROSInterruptException:
        pass

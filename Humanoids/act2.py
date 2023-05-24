#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import pinocchio

# Variables globales para almacenar el estado de configuración
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

    # Create data required by the algorithms
    data = model.createData()

    # Home configuration
    q = pinocchio.neutral(model)
    rospy.loginfo('q: %s' % q.T)

    # Perform the forward kinematics over the kinematic tree
    pinocchio.forwardKinematics(model, data, q)

    # Print out the placement of each joint of the kinematic tree
    for name, oMi in zip(model.names, data.oMi):
        rospy.loginfo(("{:<24} : {: .3f} {: .3f} {: .3f}"
                      .format(name, *oMi.translation.T)))

    rospy.Subscriber('/floating_base_pose_simulated', PoseStamped, floating_base_pose_callback)
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    rate = rospy.Rate(10)  # Frecuencia de actualización del estado de configuración

    while not rospy.is_shutdown():
        # Verifica si los tópicos han recibido datos
        if floating_base_pose is not None and joint_states is not None:
            # Actualiza el estado de configuración con los valores de los tópicos
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

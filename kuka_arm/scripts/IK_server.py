#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# Rotation Matrix about X
def rot_x(q):
    R_x = Matrix([[       1,       0,       0 ],
                  [       0,  cos(q), -sin(q) ],
                  [       0,  sin(q),  cos(q) ]])
    return R_x

# Rotation Matrix about Y
def rot_y(q):
    R_y = Matrix([[  cos(q),       0,  sin(q) ],
                  [       0,       1,       0 ],
                  [ -sin(q),       0,  cos(q) ]])
    return R_y

# Rotation Matrix about Z
def rot_z(q):
    R_z = Matrix([[  cos(q), -sin(q),       0 ],
                  [  sin(q),  cos(q),       0 ],
                  [       0,       0,       1 ]])
    return R_z

# Homogenious Transforms Matrix given DH parameters
def tf_matrix(alpha, a, d, q):
    mat =  Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])

    return mat

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

    ### Your FK code here
    # Create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

	# Create Modified DH parameters
    s = {alpha0:     0, a0:      0, d1:  0.75,
         alpha1: -pi/2, a1:   0.35, d2:     0,  q2: q2-pi/2,
         alpha2:     0, a2:   1.25, d3:     0,
         alpha3: -pi/2, a3: -0.054, d4:  1.50,
         alpha4:  pi/2, a4:      0, d5:     0,
         alpha5: -pi/2, a5:      0, d6:     0,
         alpha6:     0, a6:      0, d7: 0.303,  q7: 0 }

	# Define Modified DH Transformation matrix
    T0_1 = tf_matrix(alpha0, a0, d1, q1).subs(s)
    T1_2 = tf_matrix(alpha1, a1, d2, q2).subs(s)
    T2_3 = tf_matrix(alpha2, a2, d3, q3).subs(s)
    T3_4 = tf_matrix(alpha3, a3, d4, q4).subs(s)
    T4_5 = tf_matrix(alpha4, a4, d5, q5).subs(s)
    T5_6 = tf_matrix(alpha5, a5, d6, q6).subs(s)
    T6_EE = tf_matrix(alpha6, a6, d7, q7).subs(s)
    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

	# Create individual transformation matrices
    T0_2 = T0_1 * T1_2
    T0_3 = T0_2 * T2_3
    T0_4 = T0_3 * T3_4
    T0_5 = T0_4 * T4_5
    T0_6 = T0_5 * T5_6

	# Extract rotation matrices from the transformation matrices
    R0_1 = T0_1[0:3,0:3]
    R0_2 = T0_2[0:3,0:3]
    R0_3 = T0_3[0:3,0:3]
    R0_4 = T0_4[0:3,0:3]
    R0_5 = T0_5[0:3,0:3]
    R0_6 = T0_6[0:3,0:3]
    R0_EE = T0_EE[0:3,0:3]

    # Initialize service response
    joint_trajectory_list = []
    for x in xrange(0, len(req.poses)):
        # IK code starts here
        joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
        px = req.poses[x].position.x
        py = req.poses[x].position.y
        pz = req.poses[x].position.z

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [req.poses[x].orientation.x, req.poses[x].orientation.y,
                     req.poses[x].orientation.z, req.poses[x].orientation.w])

        ### Your IK code here
    	# Compensate for rotation discrepancy between DH parameters and Gazebo
        rot_URDF2DH = rot_y(90) * rot_z(180)
        Rrpy = (rot_x(roll) * rot_y(pitch) * rot_z(yaw) * rot_URDF2DH).evalf()

        # Calculate joint angles using Geometric IK method
        wx = (px - d7 * Rrpy[0,2]).subs(s)
        wy = (py - d7 * Rrpy[1,2]).subs(s)
        wz = (pz - d7 * Rrpy[2,2]).subs(s)

        # triangle sides
        A = a2
        B = sqrt(a3**2 + d4**2)
        C = sqrt((sqrt(wx**2 + wy**2)-a1)**2 + (wz - d1)**2)

        # apply law of cosines
        cos_gamma = (A**2 + C**2 - B**2) / (2 * A * C)
        gamma = atan2(sqrt(1-cos_gamma**2), cos_gamma)
        cos_beta = (A**2 + B**2 - C**2) / (2 * A * B)
        beta = atan2(sqrt(1-cos_beta**2), cos_beta)

        # calculate theta 1-3
        theta1 = atan2(wy, wx).evalf(subs=s)
        theta2 = (pi/2 - gamma - atan2(wz-d1, sqrt(wx**2 + wy**2) - a1)).subs(s)
        theta3 = (pi/2 - beta - atan2(abs(a3), d4)).subs(s)

        # rotation matrix from joint 3 to joint 6,
        R0_3_inv = Transpose(R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3}))
        R3_6 = (R0_3_inv * Rrpy).evalf()

        # Euler angle decomposition
        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta5 = atan2(sqrt(R3_6[1,0]**2 + R3_6[1,1]**2), R3_6[1,2])
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])

        # Populate response for the IK request
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

    rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()

#!/usr/bin/env python3

import math
import rospy
from planar_3dof_control.msg import EndEffector
from geometry_msgs.msg import Vector3
from move_joints import JointMover  # Replace with the actual package and module
from rviz_marker import MarkerBasics  # Replace with the actual package and module


class EndEffectorMover:
    def __init__(self):
        rospy.init_node('antropomorphic_end_effector_mover', anonymous=True)

        # Subscribe to the elipsoidal motion commands
        rospy.Subscriber('/ee_pose_commands', EndEffector, self.ee_pose_callback)

        # Subscribe to the real end effector pose
        rospy.Subscriber('/end_effector_real_pose', Vector3, self.real_pose_callback)

        # Initialize JointMover and MarkerBasics
        self.joint_mover = JointMover() 
        
        # Set up the marker for visualization
        self.marker = MarkerBasics()
        self.index_ = 0


    def ee_pose_callback(self, msg):
        # Callback for receiving elipsoidal motion commands
        solutions = self.inverse_kinematics((msg.ee_xy_theta.x, msg.ee_xy_theta.y, msg.ee_xy_theta.z))

        # Send the command to the joints and marker
        for sol in solutions:
            if sol[1] > 0 and sol[2] < 0:
                self.joint_mover.move_all_joints(sol[0], sol[1], sol[2])
                break

    def real_pose_callback(self, msg):
        # Callback for receiving the real end effector pose
        self.marker.publish_point(msg.x, msg.y, msg.z, self.index_)
        self.index_ = self.index_ + 1

    def inverse_kinematics(self, position):
        Px, Py, Pz = position

        # Solutions
        solutions = list()
        theta_1_sols = list()
        theta_3_sols = list()

        # Denavit-Hartenberg parameters
        r1 = 0.0
        r2 = 1.0
        r3 = 1.0

        # Joint limitations
        theta2_limit = (-math.pi/4, 3*math.pi/4)
        theta3_limit = (-3*math.pi/4, 3*math.pi/4)

        # Theta 1
        theta_1 =  math.atan2(Py, Px)
        theta_1_neg =  math.atan2(-Py, -Px)

        theta_1_sols.append(theta_1)
        theta_1_sols.append(theta_1_neg)

        for sol in theta_1_sols:
            if math.cos(sol) != 0:
                alpha = Px/math.cos(sol) - r1
            else:
                alpha = Py/math.sin(sol) - r1

            # Theta 3
            c3 = (alpha**2 + Pz**2 - r2**2 - r3**2)/(2* r2 * r3)
            theta_3 = math.acos(c3) 
            theta_3_neg = -math.acos(c3)

            theta_3_sols.append(theta_3)
            theta_3_sols.append(theta_3_neg)

            # Theta 2
            for theta3_sol in theta_3_sols:
                theta_2 = math.atan2(Pz, alpha) - math.atan2(r3*math.sin(theta3_sol), r2 + r3 * math.cos(theta3_sol))

                if theta2_limit[0] <= theta_2 <= theta2_limit[1] and theta3_limit[0] <= theta3_sol <= theta3_limit[1]:
                    solutions.append((sol, theta_2, theta3_sol))

            theta_3_sols.clear()        

        return solutions
            
   
if __name__ == '__main__':
    try:
        end_effector_mover = EndEffectorMover()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
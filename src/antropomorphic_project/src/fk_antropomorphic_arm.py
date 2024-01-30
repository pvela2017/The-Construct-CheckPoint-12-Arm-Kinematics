#!/usr/bin/env python3

import math
from sympy import *

def forward_kinematics(theta1, theta2, theta3):
    r1 = 0.0
    r2 = 1.0
    r3 = 1.0

    p1 = 6.12323399573677e-17

    A0_1 = Matrix([[cos(theta1), -p1*sin(theta1), sin(theta1), r1*cos(theta1)],
                [sin(theta1), p1*cos(theta1), -cos(theta1), r1*sin(theta1)],
                [0, 1.0, p1, 0],
                [0, 0, 0, 1]])

    A1_2 = Matrix([[cos(theta2), -sin(theta2), 0, r2*cos(theta2)],
                [sin(theta2),  cos(theta2), 0, r2*sin(theta2)],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

    A2_3 = Matrix([[cos(theta3), -sin(theta3), 0, r3*cos(theta3)],
                [sin(theta3),  cos(theta3), 0, r3*sin(theta3)],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])


    A03 = A0_1 * A1_2 * A2_3

    # Extract position and orientation from the transformation matrix
    position = A03[:3, 3]
    orientation = A03[:3, :3]

    return position, orientation




if __name__ == "__main__":
    # Get user input for theta1, theta2, and theta3
    theta1 = float(input("Enter the value for theta1 in radians: "))
    theta2 = float(input("Enter the value for theta2 in radians: "))
    theta3 = float(input("Enter the value for theta3 in radians: "))

    # Calculate position and orientation of Frame 3
    position, orientation = forward_kinematics(theta1, theta2, theta3)

    # Display the result
    print("\nPosition of Frame 3:", position)
    print("Orientation matrix of Frame 3:")
    print(orientation)
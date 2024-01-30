#!/usr/bin/env python3

import math
from sympy import *

theta1 = Symbol("theta1")
theta2 = Symbol("theta2")
theta3 = Symbol("theta3")

r1 = Symbol("r1")
r2 = Symbol("r2")
r3 = Symbol("r3")

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
A0_3 = trigsimp(A03)

A13 = A1_2 * A2_3
A1_3 = trigsimp(A13)


preview(A0_1, viewer='file', filename="../matrixes/A01.png", dvioptions=['-D','300'])
preview(A1_2, viewer='file', filename="../matrixes/A12.png", dvioptions=['-D','300'])
preview(A2_3, viewer='file', filename="../matrixes/A23.png", dvioptions=['-D','300'])

preview(A0_3, viewer='file', filename="../matrixes/A03.png", dvioptions=['-D','300'])
preview(A1_3, viewer='file', filename="../matrixes/A13.png", dvioptions=['-D','300'])

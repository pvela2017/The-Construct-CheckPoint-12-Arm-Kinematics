#!/usr/bin/env python3

import math

def inverse_kinematics(position):
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
                solutions.append((sol, theta_2, theta3_sol, True))
            else:
                solutions.append((sol, theta_2, theta3_sol, False))

        theta_3_sols.clear()        

    for sol in solutions:
        print(f"Theta1: {sol[0]}, Theta2: {sol[1]}, Theta3: {sol[2]}, {sol[3]}")
        
   
if __name__ == "__main__":
    # Test the script with the given position P3=[0.5, 0.6, 0.7]
    position_test = [0.5, 0.6, 0.7]
    inverse_kinematics(position_test)

    
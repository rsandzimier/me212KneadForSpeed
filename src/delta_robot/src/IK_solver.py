#!/usr/bin/python

# Reference this paper https://drive.google.com/open?id=0Bx9M_Avapgn-RV9RTVQwMGZDeVpiSDJiTDJaeEpIYXIxVGIw

import math

class IKSolver():
    def __init__(self):
        # Dimensions are in mm
        self.L = 304.8 # Length of link 1 (attached to motor)
        self.l = 762.0 # Length of link 2
        self.sb = 220.0 # Length of side of base plate
        self.sp = 180.0 # Length of side of end effector plate
        self.gripper_center_x_offset = 0 # x position of the gripper center of rotation relative to the center of the end effector plate center
        self.gripper_center_y_offset = 0 # y position of the gripper center of rotation relative to the center of the end effector plate center
        # See paper for definitions of the following dimensions
        self.wb = math.sqrt(3)*self.sb/6
        self.ub = math.sqrt(3)*self.sb/3
        self.wp = math.sqrt(3)*self.sp/6
        self.up = math.sqrt(3)*self.sp/3
        self.a = self.wb - self.up
        self.b = self.sp/2 - math.sqrt(3)*self.wb/2
        self.c = self.wp - self.wb/2

    def solve(self,xyz_pos): 
    	# parameter xyz_pos is a list of floats in mm [x,y,z]
    	# returns corresponding joint angles as list if solution exists. returns empty list if no solution
        x = xyz_pos[0] - self.gripper_center_x_offset
        y = xyz_pos[1] - self.gripper_center_y_offset
        z = xyz_pos[2]

        L = self.L
        l = self.l
        a = self.a
        b = self.b
        c = self.c

        E1 = 2*L*(y+a)
        F1 = 2*z*L
        G1 = x**2 + y**2 + z**2 + a**2 + L**2 + 2*y*a - l**2

        E2 = -L*(math.sqrt(3)*(x+b)+y+c)
        F2 = 2*z*L
        G2 = x**2 + y**2 + z**2 + b**2 + c**2 + L**2 + 2*(x*b+y*c) - l**2

        E3 = L*(math.sqrt(3)*(x-b)-y-c)
        F3 = 2*z*L
        G3 = x**2 + y**2 + z**2 + b**2 + c**2 + L**2 + 2*(-x*b+y*c) - l**2

        E = [E1,E2,E3]
        F = [F1,F2,F3]
        G = [G1,G2,G3]

        solution = [0.0,0.0,0.0]

        for i in range(0,3):
            if E[i]**2 + F[i]**2 - G[i]**2 < 0:
                return [] # No solution
            t1 = (-F[i] + math.sqrt(E[i]**2 + F[i]**2 - G[i]**2))/(G[i]-E[i])
            t2 = (-F[i] - math.sqrt(E[i]**2 + F[i]**2 - G[i]**2))/(G[i]-E[i])
            sol1 = math.atan(t1)
            sol2 = math.atan(t2)
            solution[i] = sol1 if abs(sol1) < abs(sol2) else sol2 # Choose solution that is kinked "out"

        return solution # list of 3 motor joint angles 
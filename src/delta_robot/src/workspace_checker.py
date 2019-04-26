#!/usr/bin/python

import math
from IK_solver import IKSolver
class WorkspaceChecker():
    def __init__(self):
        # Dimensions are in mm
        self.l = 762.0
        self.xyz_min = [-330.2, -330.2, -635]
        self.xyz_max = [ 330.2,  330.2,  0.0]

        self.maxtheta = math.pi/2
        self.mintheta = -70*math.pi/180
        self.minphi = -18*math.pi/180
        self.maxphi = 18*math.pi/180
        self.gripper_center_x_offset = 0
        self.gripper_center_y_offset = 0

    def check(self, xyz_pos, printMessages=False):
        if not self.isBetween(xyz_pos, self.xyz_min, self.xyz_max):
            if printMessages: print("xyz out of range")
            return False
        iksolver = IKSolver()
        theta = iksolver.solve(xyz_pos)
        if len(theta)==0 or not self.isBetween(theta, [self.mintheta]*3, [self.maxtheta]*3):
            if printMessages: print("no IK solution or theta out of range")
            return False
        xend = xyz_pos[0] - self.gripper_center_x_offset
        yend = xyz_pos[1] - self.gripper_center_y_offset
        zend = xyz_pos[2]
        phi0 = math.asin(xend/self.l)
        phi1 = math.asin((-0.5*xend-math.sqrt(3)/2*yend)/self.l)
        phi2 = math.asin((-0.5*xend+math.sqrt(3)/2*yend)/self.l)
        if printMessages:
            print("phi values: " + str([phi0,phi1,phi2]))
            print("theta values: " + str(theta))
        if not self.isBetween([phi0,phi1,phi2], [self.minphi]*3, [self.maxphi]*3):
            if printMessages: print("phi out of range")
            return False
        if printMessages: print("in workspace")
        return True

    def isBetween(self, vals, min_vals, max_vals):
        if len(vals) != len(min_vals) or len(vals) != len(max_vals): return False 
        for i in range(0, len(vals)):
            if vals[i] > max_vals[i] or vals[i] < min_vals[i]:
                return False
        return True
        
    


#!/usr/bin/env python

"""
Benjmain Ziemann
benjamin.ziemann@students.olin.edu

Recieves cube struct from Kevin containing

"""

from __future__ import absolute_import
import numpy
import math
from irl.msg import Grid_Cube, Real_Cube, Real_Structure, Grid_Structure, Cube_Structures
import rospy


class CoordFrames(object):

    def __init__(self):
        """"
        Setup and variables associated with coordinate frames
        """

        #Cube for calibration  due to camera shifting
        f = open("previousOrigin.txt", "r")
        coords = f.readlines()
        f.close()
        self.origin = Grid_Cube()
        self.origin.x = float(coords[0].strip())
        self.origin.y = float(coords[1].strip())
        self.origin.z = float(coords[2].strip())

        #Physical offsets of global origin
        self.armOffSetY = -.550
        self.polluxOffSetX = -.0254
        self.polluxOffSetY = -.010

        self.cubeSize = .0889

        
        #Camera values
        self.pixelX = []
        self.pixelY = []
        self.pixelZ = []

        for i in range(5):
            self.pixelX.append(float('%.3f'%(self.origin.x+(i*0.04))))
        for i in range(5):
            self.pixelY.append(float('%.3f'%(self.origin.y-(i*0.04))))
        for i in range(5):
            self.pixelZ.append(float('%.3f'%(self.origin.z+(i*0.04))))


        #Real world values
        #Castor set
        self.realXC = [2.0*self.cubeSize, self.cubeSize, 0.0, -self.cubeSize, -2.0*self.cubeSize]
        self.realYC = [2.0*self.cubeSize+self.armOffSetY, self.cubeSize+self.armOffSetY, self.armOffSetY, 0.0, 0.0]

        #Pollux Set
        self.realXP = [-2.0*self.cubeSize+self.polluxOffSetX, -self.cubeSize+self.polluxOffSetX, 0.0, -self.cubeSize+self.polluxOffSetX, -2.0*self.cubeSize+self.polluxOffSetX]
        self.realYP = [0.0, 0.0, 0.0, self.armOffSetY+self.cubeSize+self.polluxOffSetY, self.armOffSetY+2.0*self.cubeSize+self.polluxOffSetY]

        #Shared
        self.realZ = [.047, .141, .237, .329, .423]



    def updateOrigin(self, originCube):
        """
        Updates the origin cube that is used to make the board

        Use for calibration purposes. 
        originCube is a Grid_Cube()
        """
        self.origin = originCube

        self.pixelX = []
        self.pixelY = []
        self.pixelZ = []

        for i in range(5):
            self.pixelX.append(float('%.3f'%(self.origin.x+(i*0.04))))
        for i in range(5):
            self.pixelY.append(float('%.3f'%(self.origin.y-(i*0.04))))
        for i in range(5):
            self.pixelZ.append(float('%.3f'%(self.origin.z+(i*0.04))))

        f = open("previousOrigin.txt", "w")
        f.truncate()
        f.write(str(self.origin.x)+"\n")
        f.write(str(self.origin.y)+"\n")
        f.write(str(self.origin.z)+"\n")
        f.close()


    def closest(self, values, val):
        """
        Returns the element from a list's index that is
        closest to the given value
        """
        mini = 999999
        min_index = None
        for i in range(5):
            if math.fabs(val-values[i]) < mini:
                mini = math.fabs(val-values[i])
                min_index = i
        return min_index


    def convertBoard(self, cubes):
        """
        Takes in a list of cubes structs and converts it to board model

        Called by Kevin's script for building planning
        """
        #Set Up board
        board = []

        #Add a new z layers
        for i in xrange(5):
            board.append([[None, None, None, None, None],
                            [None, None, None, None, None],
                            [None, None, None, None, None],
                            [None, None, None, None, None],
                            [None, None, None, None, None]])

        #Convert cube data to board dimensions
        #Fill out board according to cube data
        for cube in cubes.building:
            gcube = Grid_Cube()
            gcube.x = self.closest(self.pixelX, cube.x)
            gcube.y = self.closest(self.pixelY, cube.y)
            gcube.z = self.closest(self.pixelZ, cube.z)
            board[gcube.x][gcube.y][gcube.z] = gcube
        return board #3d array with x, y, and z of blocks


    def convertReal(self, cubes):
        """
        Takes in a list of cubes structs and converts it to a real board

        Called by Kevin's script for building planning
        """

        #Fill out board according to cube data
        real_cubes = Real_Structure()
        for cube in cubes.building:
            real_cube = Real_Cube()
            if(real_cube.y < 3):
                real_cube.x = self.realXC[cube.x]
                real_cube.y = self.realYC[cube.y]
                real_cube.z = self.realZ[cube.z]
            else:
                real_cube.x = self.realXP[cube.x]
                real_cube.y = self.realYP[cube.y]
                real_cube.z = self.realZ[cube.z]

            real_cubes.building.append(real_cube)
        return real_cubes

if __name__ == "__main__":
    cf = CoordFrames()

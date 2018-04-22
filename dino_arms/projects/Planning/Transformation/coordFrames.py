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

        #Physical offsets of global origin
        self.armOffSetY = -.550
        self.polluxOffSetX = -.0254
        self.polluxOffSetY = -.010

        self.cubeSize = .0889


        #TODO
        #Camera values. CONFIRM THESE WITH FIRST YEAR DATA
        self.pixelX = [-.065, -.025, .015, .055, .095]
        self.pixelY = [.580, .540, .500, .460, .420]
        self.pixelZ = [.0185, .0585, .0985, .139, .180]

        #Real world values
        #Castor set
        self.realXC = [2.0*self.cubeSize, self.cubeSize, 0.0, -self.cubeSize, -2.0*self.cubeSize]
        self.realYC = [2.0*self.cubeSize+self.armOffSetY, self.cubeSize+self.armOffSetY, self.armOffSetY, 0.0, 0.0]

        #Pollux Set
        self.realXP = [-2.0*self.cubeSize+self.polluxOffSetX, -self.cubeSize+self.polluxOffSetX, 0.0, -self.cubeSize+self.polluxOffSetX, -2.0*self.cubeSize+self.polluxOffSetX]
        self.realYP = [0.0, 0.0, 0.0, self.armOffSetY+self.cubeSize+self.polluxOffSetY, self.armOffSetY+2.0*self.cubeSize+self.polluxOffSetY]

        #Shared
        self.realZ = [.047, .141, .237, .329, .423]


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
            board[gcube.z][gcube.x][gcube.y] = gcube
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
            real_cube.x = self.realX[cube.x]+self.armOffSetX
            real_cube.y = self.realY[cube.y]
            real_cube.z = self.realZ[cube.z]
            real_cubes.building.append(real_cube)
        return real_cubes

if __name__ == "__main__":
    cf = CoordFrames()

"""
Benjmain Ziemann
benjamin.ziemann@students.olin.edu

Recieves cube struct from Kevin containing

"""

import numpy
import math
from irl.msg import Cube, Structure, Cube_Structures



class CoordFrames():

	#Dimensions of cubes in mm
	self.cubeHeight = 94

	#TODO
	#Camera values. GET THESE FROM FIRST YEAR DATA
	self.pixelX = [-81.28, -40.64, 0, 40.64, 81.28]
	self.pixelY = [538.48, 497.84, 457.2, 416.56, 375.92]
	self.pixelZ = [20.32, 60.96, 101.6, 142.24, 182.88]

	self.realX = [-177.8, -88.9, 0, 88.9, 177.8]
	self.realY = [-177.8, -88.9, 0, 88.9, 177.8]
	self.realZ = [47, 141, 237, 329, 423]

	def closest(self, values, val):
		min = 999999
		for i in values:
			if math.abs(val-values[i]) < min:
				min = i


	def convertBoard(self, cubes):
		"""
		Takes in a list of cubes structs and converts it to board model

		Called by Kevin's script for building planning
		"""

		#Set Up board
		board = []

		#Add a new z layers
		for i in range(5):
			board.append([[None, None, None, None, None], 
							[None, None, None, None, None], 
							[None, None, None, None, None],
							[None, None, None, None, None],
							[None, None, None, None, None]])
		
		#Convert cube data to board dimensions
		#Fill out board according to cube data
		for cube in cubes:
			cube.x = self.closest(self.pixelX, cube.x)
			cube.y = self.closest(self.pixely, cube.y)
			cube.z = self.closest(self.pixelz, cube.z)
			board[cube.z][cube.x][cube.y] = cube

		return board #3d array with x, y, and z of blocks


	def convertReal(self, cubes):
		"""
		Takes in a list of cubes structs and converts it to a real board

		Called by Kevin's script for building planning
		"""

		#Set Up board
		board = self.convertBoard(cubes)

		#Convert cube data to board dimensions
		#Fill out board according to cube data
		for z in range(5):
			for x in range(5):
				for y in range(5)
					if board[z][x][y] != None:
						cube = board[z][x][y]
						cube.x = self.realX[cube.x]
						cube.y = self.realY[cube.y]
						cube.z = self.realZ[cube.z]
						board[z][x][y] = cube

		return board #3d array with x, y, and z of blocks

if __name__ == "__main__":
	cf = CoordFrames()
	cf.convertBoard()
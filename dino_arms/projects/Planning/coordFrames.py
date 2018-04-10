"""
Benjmain Ziemann
benjamin.ziemann@students.olin.edu

Recieves cube struct from Kevin containing

"""

import numpy
import math
#from irl.msg import Cube, Structure, Cube_Structures



class CoordFrames():

	#Dimensions of cubes in mm
	self.cube_size = 92

	#TODO
	#Camera values. GET THESE FROM FIRST YEAR DATA
	self.pixelX = [.8, 2.4, 4.0, 5.6, 7.2]
	self.pixelY = [.8, 2.4, 4.0, 5.6, 7.2]
	self.pixelZ = [.8, 2.4, 4.0, 5.6, 7.2]

	def closest(self, values, val):
		min = 999999
		for i in values:
			if math.abs(val-values[i]) < min:
				min = i


	def convertBoard(self):
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
			cube.x = self.closest(self.pixelX, cube.x)]
			cube.y = self.closest(self.pixely, cube.y)]
			cube.z = self.closest(self.pixelz, cube.z)]
			board[cube.z][cube.x][cube.y] = cube

		return board #3d array with x, y, and z of blocks


	def convertReal(self, cubes):
		"""
		Takes in a list of cubes structs and converts it to a real board

		Called by Kevin's script for building planning
		"""

if __name__ == "__main__":
	cf = CoordFrames()
	cf.convertBoard()
#!/usr/bin/env python

import numpy as np
import time
import sys
import rospy
import Tkinter as tk


UNIT = 100   # pixels
MAZE_H = 5  # grid height
MAZE_W = 5  # grid width


class Cell(object):

    def __init__(self, canvas, x, y, size):
        """ Constructor of the object called by Cell(...) """
        self.canvas = canvas
        self.abs = x
        self.ord = y
        self.size= size
        self.fill= False
        self.rect = None

        self.xmin = self.abs * self.size
        self.xmax = self.xmin + self.size
        self.ymin = self.ord * self.size
        self.ymax = self.ymin + self.size

    def switch(self):
        """ Switch if the cell is filled or not. """
        self.fill= not self.fill

    def draw(self):
        """ order to the cell to draw its representation on the canvas """
        if self.canvas != None :
            fill = "#66d9ef" if self.fill else "#104494"

            self.rect = self.canvas.create_rectangle(self.xmin, self.ymin, self.xmax, self.ymax, fill = fill, activefill="#a1e9f7")
            self.canvas.tag_bind(self.rect, "<Button-1>", self.handleMouseClick)

    def handleMouseClick(self, event):
        self.switch()
        self.canvas.delete(self.rect)
        self.draw()



class UI(tk.Tk, object):
    def __init__(self):
        super(UI, self).__init__()
        self.title('Minecraft MVP')
        self.publisher = rospy.Publisher("minimap", String, queue_size=10)
        self.build_env()

    def build_env(self):
        self.canvas = tk.Canvas(self, bg='white',
                           height=MAZE_H * UNIT,
                           width=MAZE_W * UNIT)

        self.grid = []
        for row in range(5):

            line = []
            for column in range(5):
                line.append(Cell(self.canvas, column, row, UNIT))

            self.grid.append(line)

        self.label = tk.Label(self, text="Use the grid to plan your structure, then click BUILD when done!", width="50", height="5", bg='#1a0f35', fg="white")
        self.label.pack(side=tk.TOP, padx=0, pady=0, fill= tk.X)
        self.draw()
        self.b = tk.Button(self, text="Build", width="50", height="5", bg="#0483e9", activebackground="#a1e9f7")
        self.b.bind("<Button-1>", self.callback)
        self.b.pack(side=tk.BOTTOM, padx=0, pady=0, fill=tk.X)

        # pack all
        self.canvas.pack()

    def callback(self, event):
        print "Begin Processing"
        cubes = []
        for row in range(5):
            for column in range(5):
                if self.grid[row][column].fill:
                    cubes.append(self.grid[row][column])




    def draw(self):
        for row in self.grid:
            for cell in row:
                cell.draw()



if __name__ == '__main__':
    env = UI()
    env.mainloop()

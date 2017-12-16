"""
This script creates the GUI for tool picker game

Need to have roscore running
Initial Setup button setup all the dependencies automatically
Ready to Play button starts detection & simon says script
"""
import sys
from PySide.QtCore import *
from PySide.QtGui import *
import rospy
import cv2
import numpy as np
import random
import time
import math
import os

from std_msgs.msg import String
from sensor_msgs.msg import Image

qt_app = QApplication(sys.argv)

class ToolPickerGui(QWidget):

	def __init__(self):
		rospy.init_node("ToolPickerGui", anonymous=True)
		self.cmd_pub = rospy.Publisher("/tool_cmd", String, queue_size=10)
		self.prev_cmd = ""
		self.cmd = ""
		self.is_playing = False

		# Initialize the object as a QWidget and
		# set its title and minimum width
		QWidget.__init__(self)
		self.setWindowTitle('Tool Picker')
		self.setMinimumWidth(600)
		self.setMinimumHeight(400)
		font = QFont()
		font.setPointSize(30)
		self.setFont(font)

		#Set up timer to listen to command and player gesture
		self.timer = QTimer()

		# Create the QVBoxLayout that lays out the whole form
		self.layout = QVBoxLayout()

		# Create the form layout that manages the labeled controls
		self.form_layout = QFormLayout()
		self.tool_list = ['Clamp','Cutter', 'Screw Driver','Wrench','Scissors','Piler']

		# Create and fill the combo box to choose the simon_user
		self.tool_box = QComboBox(self)
		self.tool_box.addItems(self.tool_list)

		# Add it to the form layout with a label
		self.form_layout.addRow('&Welcome to Tool Picker game!', self.tool_box)
		self.form_layout.addRow('&Please pick the tool you want:', self.tool_box)

		# Create the entry control to specify a
		# recipient and set its placeholder text
		# Add the form layout to the main VBox layout
		self.layout.addLayout(self.form_layout)

		# Add stretch to separate the form layout from the button
		self.layout.addStretch(1)

		# Create a horizontal box layout to hold the button
		self.button_box = QHBoxLayout()

		# Add stretch to push the button to the far right
		self.button_box.addStretch(1)

		#creating buttons that will initialize everything and display commands
		self.setup_button = QPushButton('&Initial Setup',self)
		self.setup_button.clicked.connect(self.set_this_up)

		self.play_button = QPushButton('&Start Playing!',self)
		self.play_button.clicked.connect(self.lets_play)

		# Add it to the button box
		self.button_box.addWidget(self.setup_button)
		self.button_box.addWidget(self.play_button)

		# Add the button box to the bottom of the main VBox layout
		self.layout.addLayout(self.button_box)
		# Set the VBox layout as the window's main layout
		self.setLayout(self.layout)

	@Slot()
	def set_this_up(self):
		'''
		After the user presses the Initial Setup button
		'''
		# rosrun usb_cam usb_cam_node _video_device:='/dev/video1'
		self.cmd_pub.publish("")

	@Slot()
	def lets_play(self):
		'''
		After the user presses the Ready to Play button
		'''
		self.is_playing = True
		self.prev_cmd = self.cmd
		self.cmd = str(self.tool_box.currentText())

		if self.cmd != self.prev_cmd:
			self.cmd_pub.publish(self.cmd)

	def run(self):
		# Show the form
		self.show()

		# Run the qt application
		qt_app.exec_()

if __name__ == "__main__":
	# Create an instance of the application window and run it
	app = ToolPickerGui()
	app.run()

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
		self.cmd = ""
		self.res = ""
		self.is_playing = False

		# Initialize the object as a QWidget and
		# set its title and minimum width
		QWidget.__init__(self)
		self.setWindowTitle('SimonSays')
		self.setMinimumWidth(600)
		self.setMinimumHeight(400)
		font = QFont()
		font.setPointSize(15)

		#Set up timer to listen to command and player gesture
		self.timer = QTimer()

		# Create the QVBoxLayout that lays out the whole form
		self.layout = QVBoxLayout()

		# Create and fill the combo box to choose the simon_user
		self.simon_user = QComboBox(self)
		self.simon_user.addItems(self.user)

		# Add it to the form layout with a label
		self.form_layout.addRow('&Welcome to the ', self.simon_user)

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

		self.configured_button = QPushButton('&Ready to play?',self)
		self.configured_button.clicked.connect(self.lets_play)

		# Add it to the button box
		self.button_box.addWidget(self.setup_button)
		self.button_box.addWidget(self.configured_button)

		# Add the button box to the bottom of the main VBox layout
		self.layout.addLayout(self.button_box)
		# Set the VBox layout as the window's main layout
		self.setLayout(self.layout)

	def command_callback(self,data):
		self.cmd = data.data

	def result_callback(self,data):
		self.res = data.data

	@Slot()
	def update_text(self):
		'''
		Update the text message
		'''
		if self.is_playing == True:
			self.command.setText(self.cmd)
			self.result.setText(self.res)

	@Slot()
	def set_this_up(self):
		'''
		After the user presses the Initial Setup button
		'''
		os.system("gnome-terminal -e 'bash -c \"roslaunch skeleton_markers markers_from_tf.launch; exec bash\"'")
		time.sleep(1)
		os.system("gnome-terminal -e 'bash -c \"cd ../; cd skeleton_markers/;  rosrun rviz rviz -d markers_from_tf.rviz;  exec bash\"'")
		time.sleep(1)
		os.system("gnome-terminal -e 'bash -c \"rosrun edwin skeleton.py;  exec bash\"'")
		time.sleep(1)

	@Slot()
	def lets_play(self):
		'''
		After the user presses the Ready to Play button
		'''
		self.is_playing = True
		os.system("gnome-terminal -e 'bash -c \"cd scripts/; cd sight/; python3 skeleton_characterization.py; exec bash\"'")
		time.sleep(1)
		os.system("gnome-terminal -e 'bash -c \"cd scripts/; cd Interactions/; python Simon_Says_practice.py; exec bash\"'")
		time.sleep(1)

	def run(self):
		# Show the form
		self.show()

		#Set Timer for update label
		self.connect(self.timer, SIGNAL("timeout()"), self.update_text)
		self.timer.start(300)

		# Run the qt application
		qt_app.exec_()

if __name__ == "main":
	# Create an instance of the application window and run it
	app = ToolPickerGui()
	app.run()

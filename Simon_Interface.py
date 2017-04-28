import sys
from PySide.QtCore import Slot
from PySide.QtGui import *
import rospy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time
import math
import os


from std_msgs.msg import String
from sensor_msgs.msg import Image
from edwin.msg import Edwin_Shape, Bones
from cv_bridge import CvBridge, CvBridgeError


# Every Qt application must have one and only one QApplication object;
# it receives the command line arguments passed to the script, as they
# can be used to customize the application's appearance and behavior
qt_app = QApplication(sys.argv)

class LayoutExample(QWidget):

	def __init__(self):
		rospy.Subscriber("/say_cmd", String, self.data_collect, queue_size = 10)
		# Initialize the object as a QWidget and
		# set its title and minimum width
		QWidget.__init__(self)
		self.setWindowTitle('SimonSays')
		self.setMinimumWidth(600)
		self.setMinimumHeight(400)

		# Create the QVBoxLayout that lays out the whole form
		self.layout = QVBoxLayout()

		# Create the form layout that manages the labeled controls
		self.form_layout = QFormLayout()
		self.user = ['Edwin is Simon',
					'User is Simon']

		# Create and fill the combo box to choose the simon_user
		self.simon_user = QComboBox(self)
		self.simon_user.addItems(self.user)

		# Add it to the form layout with a label
		self.form_layout.addRow('&Who is Simon?:', self.simon_user)

		self.command = QLabel('',self)
		self.form_layout.addRow('Commands:',self.command)

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

	def data_collect(self):
		''' Show the constructed greeting. '''
		self.data = data.data
		if self.user[self.simon_user.currentIndex()] == 'Edwin is Simon':
			self.command.setText('%s,%s' % (self.data))

	@Slot()
	def set_this_up(self):
		os.system("gnome-terminal -e 'bash -c \"roscore; exec bash\"'")
		time.sleep(1)
		os.system("gnome-terminal -e 'bash -c \"roslaunch skeleton_markers markers_from_tf.launch; exec bash\"'")
		time.sleep(1)
		os.system("gnome-terminal -e 'bash -c \"cd ../; cd ../;  cd skeleton_markers/;  rosrun rviz rviz -d markers_from_tf.rviz;  exec bash\"'")

		time.sleep(1)
		os.system("gnome-terminal -e 'bash -c \"rosrun edwin skeleton.py;  exec bash\"'")
		time.sleep(1)

	@Slot()
	def lets_play(self):
		#os.system("gnome-terminal -e 'bash -c \"roscd edwin; exec bash\"'")
		os.system("gnome-terminal -e 'bash -c \"cd scripts/; cd sight/; python3 skeleton_characterization.py; exec bash\"'")
		time.sleep(5)
		os.system("gnome-terminal -e 'bash -c \"cd Interactions/; python Simon_Says_practice.py; exec bash\"'")

	def run(self):
		# Show the form
		self.show()
		# Run the qt application
		qt_app.exec_()

# Create an instance of the application window and run it
app = LayoutExample()
app.run()

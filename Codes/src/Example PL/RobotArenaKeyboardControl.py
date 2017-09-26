'''
	The Hub : Central communication system of the without
	Robot Arena, controls information flow in the system

	Modified: 20.06.2015 by Mehmet Efe Tiryaki (m.efetiryaki@gmail.com)
	Created : 20.06.2015 by Mehmet Efe Tiryaki (m.efetiryaki@gmail.com)


'''

#!/usr/bin/env python

from without.srv import *
from without.msg import *
import rospy
import random
import numpy
import sys
import termios
import os	
import cv2
import time
from math import *


class playerClient():
	"""
	This code initializes player nodes, let them chose their robot and
	collect their sensor data from the Hub node.
	"""

	def __init__(self,sticker=3):
		rospy.wait_for_service('PlayerControlService')
		playerControlService = rospy.ServiceProxy('PlayerControlService', PlayerControl)
		# keyboard event variable
		self.TERMIOS=termios
		# init player
		self.CMD = 'Init'
		self.myPlayer = Player()
		self.myPlayer.ID = 'Efe'
		self.myPlayer.Sticker = sticker
		self.myVelocity = Vector4()
		HubRequest = PlayerControlRequest(self.CMD,self.myPlayer)
		HubResponse = playerControlService(HubRequest)
		
		# Wait for start
		while HubResponse.Status ==10:
			self.CMD = 'Start?'
			HubRequest = PlayerControlRequest(self.CMD,self.myPlayer)
			HubResponse =playerControlService(HubRequest)
			time.sleep(0.5)
		# play
		while HubResponse.Status == 0:
			self.CMD = 'Sensor'
			HubRequest = PlayerControlRequest(self.CMD,self.myPlayer)
			HubResponse = playerControlService(HubRequest)
			
			# send comment according to keyboard input
			input=self.getkey()
			if input=="w":
				self.myVelocity.x=1
				self.myVelocity.y=0
				self.myVelocity.z=0
			elif input=="s":
				self.myVelocity.x=-1
				self.myVelocity.y=0
				self.myVelocity.z=0
			elif input=="a":
				self.myVelocity.x=0
				self.myVelocity.y=0
				self.myVelocity.z=-1
			elif input=="d":
				self.myVelocity.x=0
				self.myVelocity.y=0
				self.myVelocity.z=1
			else:
				self.myVelocity.x=0
				self.myVelocity.y=0
				self.myVelocity.z=0
			input=""
			# a small sleep to not result queing in Hub 		
			time.sleep(0.05)			
			# send control command
			self.CMD = 'Control'
			self.myPlayer.controlCommand = self.myVelocity
			HubRequest = PlayerControlRequest(self.CMD,self.myPlayer)
			HubResponse = playerControlService(HubRequest)



	def getkey(self):
			# This part of the code is taken from internet in such a rush 
			# that we do not remember where we took, excuse us 
        	fd = sys.stdin.fileno()
       		old = termios.tcgetattr(fd)
        	new = termios.tcgetattr(fd)
        	new[3] = new[3] & ~self.TERMIOS.ICANON & ~self.TERMIOS.ECHO
        	new[6][self.TERMIOS.VMIN] = 1
        	new[6][self.TERMIOS.VTIME] = 0
        	termios.tcsetattr(fd, self.TERMIOS.TCSANOW, new)
        	c = None
        	try:
                	c = os.read(fd, 1)
        	finally:
               		termios.tcsetattr(fd,self.TERMIOS.TCSAFLUSH, old)
        	return c


if __name__=='__main__':
	#try:
	Sticker=int(sys.argv[1])
	if (type(Sticker)==int):
		print("Keyboard robot controller started for robot with sticker " +str(Sticker))
		user1=playerClient(Sticker)
	else:
		print("You should give a robot sticker as number")
	#except:
	print("You should give a robot sticker as number")	
		

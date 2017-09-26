'''

	Modified: 21.06.2015 by Mehmet Efe Tiryaki (m.efetiryaki@gmail.com)
	Created : 19.06.2015 by Cemre Goc (cemregoc@aol.com)


'''

#!/usr/bin/env python

from without.srv import *
from without.msg import *
import rospy
import random
import numpy
import sys
import cv2
import time
from math import *



class playerClient():
	"""
	This code initializes player nodes, let them chose their robot and
	collect their sensor data from the Hub node.
	"""

	def __init__(self):
		rospy.wait_for_service('PlayerControlService')
		playerControlService = rospy.ServiceProxy('PlayerControlService', PlayerControl)

		self.CMD = 'Init'
		self.myPlayer = Player()
		self.myPlayer.ID = 'ceroloy'
		self.myPlayer.Sticker = 3
		self.myVelocity = Vector4()
		HubRequest = PlayerControlRequest(self.CMD,self.myPlayer)
		HubResponse = playerControlService(HubRequest)
		#print HubResponse.Status
		while HubResponse.Status ==10:
			self.CMD = 'Start?'
			HubRequest = PlayerControlRequest(self.CMD,self.myPlayer)
			HubResponse =playerControlService(HubRequest)
			time.sleep(0.5)
		while HubResponse.Status == 0:
			self.CMD = 'Sensor'
			HubRequest = PlayerControlRequest(self.CMD,self.myPlayer)
			HubResponse = playerControlService(HubRequest)
			#print(HubResponse)
			#print("------------------")
			targetPos = HubResponse.yourPlayer.sensors[0].data
			myPos=HubResponse.yourPlayer.sensors[1].data
			angle= atan2(targetPos[1]-myPos[1],targetPos[0]-myPos[0])*180/pi
			if -angle<myPos[2]-20:
				self.myVelocity.x=0
				self.myVelocity.y=0
				self.myVelocity.z=1
			elif -angle>myPos[2]+20:
				self.myVelocity.x=0
				self.myVelocity.y=0
				self.myVelocity.z=-1
			else:
				self.myVelocity.x=1
				self.myVelocity.y=0
				self.myVelocity.z=0
	
			time.sleep(0.05)			
			self.CMD = 'Control'
			self.myPlayer.controlCommand = self.myVelocity
			#print(self.myPlayer.controlCommand)
			HubRequest = PlayerControlRequest(self.CMD,self.myPlayer)
			HubResponse = playerControlService(HubRequest)



if __name__ == '__main__':
	user1 = playerClient()


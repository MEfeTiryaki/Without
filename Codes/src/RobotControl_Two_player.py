'''


	Modified: 21.06.2015 by Mehmet Efe Tiryaki (m.efetiryaki@gmail.com)
	Created : 21.06.2015 by Mehmet Efe Tiryaki (m.efetiryaki@gmail.com)


'''
#!/usr/bin/env python
import rospy
from without.srv import * 
from without.msg import *
import sys
import arDriveBlue as adb
import glob
class robotcontrol:
	def __init__(self):
		"""
			This code is for driving two robot at a time
		"""
		rospy.init_node('Robot_Controller')
		self.robotvector=Vector4()

		# this list should contain bdaddr adresses according to the sticker numbers
		# self.robotconnections[0]=> controls sticker 3 ,[1]=>4 and so on...
		self.robotconnections=[ 	adb.ThreeOmniDriver(minVals=[1290,1270,1300,1300],\
							  	    midVals=[1495,1525,1485,1500],\
							   	    maxVals=[1760,1750,1710,1750],\
						             	    bdaddr="98:D3:31:60:3E:54"),	
						adb.ThreeOmniDriver(minVals=[1290,1270,1300,1300],\
							    	    midVals=[1495,1525,1485,1500],\
							     	    maxVals=[1760,1750,1710,1750],\
							     	    bdaddr="98:D3:31:40:42:5D")]
		
		# Ros inits
		self.controller_=rospy.Service('RobotControlService',RobotControl, self.disperse)
		self.controller_.spin()

	def disperse(self,req):
		# send control data to specified robots 
		for pl in list(filter(lambda x: True if x.playable=="yes" else False, req.players)):
			x= -pl.modifiedCommand.x # u 
			y= pl.modifiedCommand.y  # v
			z= pl.modifiedCommand.z  # omega
			try:
				self.robotconnections[pl.Sticker-3].drive(vel=(x,y,z))
			except:
				pass

		return RobotControlResponse()				

def main():

	rc=robotcontrol()
	
if __name__ == '__main__':
		main()

'''

	Modified: 21.06.2015 by Mehmet Efe Tiryaki (m.efetiryaki@gmail.com)
	Created : 17.06.2015 by Yunus Emre Badem (yebadem@gmail.con)


'''
#!/usr/bin/env python
import rospy
from without.srv import * 
from without.msg import *
import sys
import arDriveBlue as adb
import glob
class robotcontrol:
	def __init__(self,sticker=3,vel=[0,0,0]):
		rospy.init_node('Robot_Controller')
		self.robotvector=Vector4()
		self.robotconnections=[None,None]
		
		# One should add extra robots as below
		# and check the addresses 
		if sticker==3:
			self.robotconnections[sticker-3]= adb.ThreeOmniDriver(minVals=[1290,1270,1300,1300],\
							  	    midVals=[1495,1525,1485,1500],\
							   	    maxVals=[1760,1750,1710,1750],\
						             	    bdaddr="98:D3:31:60:3E:54")		
		elif sticker==4:
			self.robotconnections[sticker-3]= adb.ThreeOmniDriver(minVals=[1300,1300,1000,1300],\
							    	    midVals=[1500,1500,1650,1500],\
							     	    maxVals=[1980,1800,2050,1750],\
							     	    bdaddr="98:D3:31:40:42:5D")
		self.drive(sticker,vel)
		self.controller_=rospy.Service('RobotControlService',RobotControl, self.disperse)
		self.controller_.spin()

	def disperse(self,req):
		for pl in req.players:
			x= -pl.modifiedCommand.x # u
			y= pl.modifiedCommand.y  # v
			z= pl.modifiedCommand.z  # omega
			try:
				self.robotconnections[pl.Sticker-3].drive(vel=(x,y,z))
			except:
				pass

		return RobotControlResponse()				

	def drive(self,sticker,velo):
		# drive specified robot
		self.robotconnections[sticker-3].drive(vel=velo)

def main(sticker=3,velo=[0,0,0]):

	rc=robotcontrol(sticker,velo)
	
if __name__ == '__main__':
	try:
		# to test robots from command line 
		# python RobotControl.py Sticker Vx Vy Omega 
		vel=[ -float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]) ] 
		sticker=int(sys.argv[1])
		print("Sticker :"+str(sticker))
		main(sticker,vel)
	except:	
		# default usage
		main()

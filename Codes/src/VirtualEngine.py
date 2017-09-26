'''

	Modified: 21.06.2015 by Mehmet Efe Tiryaki (m.efetiryaki@gmail.com)
	Created : 14.06.2015 by Cemre Goc (cemregoc@aol.com)


'''

#!usr/bin/env python
#from virtual_env.msg import *
#from virtual_env.srv import *
from without.msg import *
from without.srv import *
import rospy
import random
import numpy
import sys
import cv2
from math import *
from copy import deepcopy

class VirtualSensor(): 
	def __init__(self,getMap):
		self.sceneMap = getMap
	def calculate(self,PL,allPL):
		"""
			while creating any sensor be careful about the opencv2 x,y coordinate
			system convention take a look at our Sharp sensor it is the best working 
			example so far.
			your custom sensor should modify s.data and should write a list containing 
			float32 values

			**A good sensor example can be a modified target sensor combined with Ultrasonic
			sensor it return the distance and the heading of the target in a range and a 
			line-of-sight
		"""
		for s in PL.sensors:
			if s.name == 'LiDAR':
				s.data = [s.range+1]*int(s.angularRange)
				myRange = int(s.range)
				for theta in range(0,len(s.data)):
					for radius in range(0,myRange):
						if self.sceneMap.item(-int(radius*sin((theta + PL.position.z)*pi/180)+\
								PL.position.y),int(radius*cos((theta + PL.position.z)*pi/180)+PL.position.x)) == 255:
							s.data[theta] = radius
							break

			elif s.name == 'Sharp':
				myRange =int(s.range)
				s.data=[s.range+1]
				for R in range(0, myRange):
					if self.sceneMap.item(int(-R*sin((PL.position.z + s.relativeHeading.z)*pi/180) +PL.position.y+s.relativeHeading.y),\
										int(R*cos((PL.position.z + s.relativeHeading.z)*pi/180) + PL.position.x + s.relativeHeading.x)) > 0:
						s.data[0] = R
						break
			
			elif s.name == 'Ultrasonic':
				R = [0]*int(s.angularRange)
				myRange = int(s.range)
				for theta in range(-int(len(R)/2),int(len(R)/2)):
					for radius in range(0,myRange):
						if self.sceneMap.item(int(radius*cos((theta+PL.position.z+s.relativeHeading.z)*pi/180)+\
						   PL.position.y),int(-radius*sin((theta+PL.position.z+s.relativeHeading.z)*pi/180)+PL.position.x)) == 255:
							R[theta] = radius
							break
					
				s.data=min(R)

			elif s.name == 'Target_Sensor':
				theTarget=list(filter(lambda x:True if s.targetSticker==x.Sticker else False,allPL))
				s.data=[theTarget[0].position.x,theTarget[0].position.y]

			elif s.name == 'GPS':
				s.data = [PL.position.x,PL.position.y,PL.position.z]
			

class myVelocity():
	def __init__(self, getMap):
		self.sceneMap = getMap

	def calculateVelocity(self, PL, allPL):
		# create a copy of map
		TheMap = deepcopy(self.sceneMap) 
		# add circles for robots but no non-playable stuff other than current one
		list(map(lambda p: cv2.circle(TheMap,(int(p.position.x),int(p.position.y)),100,(255,255,255),-1,8,0),\
			list(filter(lambda player : True if player.playable=="yes" and player.Sticker != PL.Sticker else False, allPL)) ))
		# get position of the current robot
		p = PL.position
		# number of slice to be check
		_slice=20.
		divider=int(360/_slice)
		# max radius and min radius of the area will cbe checked
		myRange=100
		minRange=50
		R=[myRange]*int(_slice)
		# Just a Lidar code
		for theta in range(0,int(_slice)):
			for radius in range(minRange,myRange):
				if TheMap.item(int(-radius*sin((theta*divider+PL.position.z)*pi/180)+\
					PL.position.y),int(radius*cos((theta*divider+PL.position.z)*pi/180)+PL.position.x))==255:
					R[theta]=radius
					break
		# if there is something in range 
		if min(R)<myRange:
			i=0
			w_collision=[0,0]
			count=0
			# sum the vectors from center to wall
			while i<len(R):
 				if R[i]<myRange:
					collisionAngle=i*divider
					w_collision=list(map(lambda a,b: a+b , w_collision,[R[i]*cos(collisionAngle*pi/180),R[i]*sin(collisionAngle*pi/180)]))
					count+=1
				i+=1
			# find the unit vector
			norm_w=(w_collision[0]**2+w_collision[1]**2)**0.5
			w_collision=list(map(lambda x: x/norm_w,w_collision))
			# get the command
			u_c = [PL.controlCommand.x,PL.controlCommand.y]
			# calculate the projection magnetitude
 			projection= numpy.dot(u_c,w_collision)
			if projection>0:
				# if it is positive
				projection=projection*numpy.array(w_collision)
				# substruct the vector toward the wall
				u_c=numpy.array(u_c)-projection
				# write the result in PL.modifiedCommand
				PL.modifiedCommand.x,PL.modifiedCommand.y = u_c.item(0),-u_c.item(1) 
		# multiply with speed coefficient		
		PL.modifiedCommand.x,PL.modifiedCommand.y,PL.modifiedCommand.z=PL.modifiedCommand.x*PL.speedCoefficient,\
																	   PL.modifiedCommand.y*PL.speedCoefficient,\
																	   PL.modifiedCommand.z*PL.speedCoefficient	

#Client of Hub
class VirtualEngine():
	def __init__(self):
		rospy.init_node('VirtualEngineService')
		service = rospy.Service('VirtualEngineService', VirtualEngCont, self.handle_VirtualEngine)
		rospy.spin()


	def handle_VirtualEngine(self,req):
		if req.CMD == 'Init':
			# get the wall map as flattened numpy array
			self.myMap = numpy.array(req.sceneMap.parallelMap[0].Array)
			y,x=req.sceneMap.size
			# reshape it 
			self.myMap = numpy.uint8(numpy.reshape(self.myMap, (x,y)))
			# add counter of the map to restrict area
			linethickness = 20
			a,b = self.myMap.shape
			cv2.rectangle(self.myMap, (0 + linethickness/2,0 +linethickness/2),\
					(b -linethickness/2,a-linethickness/2), (255,255,255), linethickness, 8, 0)
			# Initiate players in Virtual Engine
			self.VEplayers=req.players
			# Initiate sensors and velocity calculator
			self.mySensor = VirtualSensor(self.myMap)
			self.myVelocity = myVelocity(self.myMap)


		elif req.CMD == 'Physics':
			# get the players
			self.VEplayers=req.players
			# Calculate sensors and velocity commands for playable players
			list(map(lambda x: self.mySensor.calculate(x,req.players), list(filter(lambda x: True if x.playable=="yes" else False, req.players))))
			list(map(lambda x: self.myVelocity.calculateVelocity(x,req.players), list(filter(lambda x : True if x.playable=="yes" else False,req.players))))
		
		return VirtualEngContResponse(req.players)

if __name__ == "__main__":
	VirtualEngine()

	

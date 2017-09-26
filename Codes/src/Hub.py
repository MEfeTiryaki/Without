'''
	The Hub : Central communication system of the without
	Robot Arena, controls information flow in the system

	Modified: 21.06.2015 by Mehmet Efe Tiryaki (m.efetiryaki@gmail.com)
	Created : 06.06.2015 by Mehmet Efe Tiryaki (m.efetiryaki@gmail.com)


'''

#! usr/bin/env python
# -*- coding: utf-8 -*-
# system imports

import random
import numpy
import sys
import cv2
import time

# ros package imports
from without.srv import *
from without.msg import *
import rospy

class Hub():
	def __init__(self):
		self.counter = 0
		rospy.init_node('The_Hub')
		print("Hub node is initiated")
		# ---The Clients---


		# 	Services should be initiated formerly 
		# 	Eye Client
		self.Eye_Data = rospy.ServiceProxy('EyeService', EyeData)
		print("Eye client is initiated")

		# 	Virtual Engine Client
		#  	client for applying sensor data and
		#	virtually altered robot motion data
		self.Virtual_Engine=rospy.ServiceProxy('VirtualEngineService',\
		   					VirtualEngCont)
		# Game Variables
		self.Map=None
		self.GameMode="Blind"
		
		# 	Robot Control Client
		self.Robot_Control= rospy.ServiceProxy('RobotControlService',\
		   					RobotControl)


		# ---The Services---
		# Game Master
		self.MasterControlService=rospy.Service('MasterControlService',\
						 MasterControl, self.MasterControlCallback)
		# Player Service
		# 	It is a generic service from which any
		# 	player can join to the game and arrange 
		#	their robots specification 
		#	any player can request data and send 
		#	command over this servise
		self. PlayerService=rospy.Service('PlayerControlService',\
						 PlayerControl, self.PlayerControlCallback)

		# ---Master Data---
		self.MasterID="Sadetra"

		# ---Player Data---
		# 	Player List
		self.Players=[]
		#	Empty Player list for initiation purposes
		self.PlayablePlayers=[]

		# ---Game Flags---
		# 	These flags are used to control the flow control 
		# 	of the game
		self.IsGameLoaded=False
		self.IsGameStarted=False
		self.PlayersReady=False
		# 	GM Flags 
		self.GM_GameLoadComfirm=False
		self.GM_EndGame=False
		self.GM_PauseGame=False
		# ---The Game Flow ---
		self.MapInit()
		print("Hub : Background is taken")
		self.Flow()


	def Flow(self):
		while True:
			if self.IsGameStarted:
				# The Game loop is started here 
				self.The_Game()
				# After game ended by GM 
				self.GM_GameLoadComfirm=False
				time.sleep(0.1)

	def MapInit(self):
		# Apply Eye for background image
		ID = 'Hub'
		cmd = 1
		size=[1500,750]
		eyeResponse=self.Eye_Data(ID,cmd,size)
		# save image Map() formated background image 
		self.Map=eyeResponse.background
		print("Hub : scene size is " +str(self.Map.size))

# Game Loop Methods
	def The_Game(self):
		while not self.GM_EndGame:
			if not self.GM_PauseGame:
				# Request World data from Eye 
				ID = 'Hub'
				cmd = 2
				size=[1500,750]
				eyeResponse=self.Eye_Data(ID,cmd,size)
				for robot in eyeResponse.robots:
					# Match know players and the one eye identify
					thePlayer=list(filter(lambda x:True \
						if x.Sticker==robot.w else False,self.Players))
					# set the position of the Player
					if len(thePlayer)==1:
						thePlayer[0].position=Vector4(robot.x,robot.y,robot.z,0)
					else:
						pass

				# Send the data to the Phys. Engine using THE SERVICE 
				VERequest=VirtualEngContRequest()
				VERequest.CMD="Physics"
				VERequest.players=self.Players
				VEResponse=self.Virtual_Engine(VERequest)
				# Update Player 
				self.Players=VEResponse.players
				
				# Set the current Robot command to send the robot controller. 
				RCRequest=RobotControlRequest()
				RCRequest.players=self.Players
				error=self.Robot_Control(RCRequest)
				# End of a Game Loop.


# Callbacks
	def MasterControlCallback(self,req):
		# Check Master ID
		if req.ID==self.MasterID:
			if req.CMD=="Init":
				# PlayablePlayers list create a connectable player list for playable players
				self.PlayablePlayers=list(filter(lambda x: True if x.playable=="yes" else False,req.players))
				# Add non-playable players to Players directly
				self.Players=list(filter(lambda x: True if x.playable=="no" else False, req.players))
				# Set Game Mode
				self.GameMode=req.gameMode
				# Set related flag
				self.GM_GameLoadComfirm=True
				print("Hub : Master upload game settings")

			elif req.CMD=="Start":
				# Set related flag
				self.IsGameStarted=True
				self.GM_EndGame=False
			elif req.CMD=="Pause":
				# Set related flag
				self.GM_PauseGame=not self.GM_PauseGame
			elif req.CMD=="End":
				# Set related flag	
				self.GM_EndGame=True
				self.IsGameStarted=False
			elif req.CMD=="IsReady":
				# Check number of the remaining PlayablePlayer 
				if self.GM_GameLoadComfirm and len(self.PlayablePlayers)==0:
					# Initiate Virtual engine setups
					VERequest=VirtualEngContRequest()
					VERequest.CMD="Init"
					VERequest.players=self.Players 
					VERequest.sceneMap=self.Map
					self.Virtual_Engine(VERequest)
					print("Hub : Player data send to Virtual Engine")
					# return ready message
					return MasterControlResponse("Ready")
				else:
					pass
			# return not ready message
			return MasterControlResponse("not Ready")
		else:
			pass	

	def PlayerControlCallback(self,req):
		# initiate a response
		response=PlayerControlResponse()
		if req.CMD=="Init":
			# Check player list for number of available Players 
			if len(self.PlayablePlayers)>0:
				# Check requested ID
				if len(list(filter(lambda x:True if x.ID==req.myPlayer.ID else\
				 						False, self.Players)))==0:
					# get the requested sticker
					appliedPlayer=list(filter(lambda x:True \
							if x.Sticker==req.myPlayer.Sticker else False,self.PlayablePlayers))
					if len(appliedPlayer)==1:
						# if available assign 
						appliedPlayer[0].ID=req.myPlayer.ID
						self.Players.append(appliedPlayer[0])
						self.PlayablePlayers.remove(appliedPlayer[0])
						print("Hub : player is added \n"+ str(appliedPlayer[0])) 
						response.Status=10 # game is not started yet
						response.yourPlayer=appliedPlayer[0]
						# if game mode is Informed
						if self.GameMode=="Informed":
							response.map=self.Map
					else:
						response.Status=2 # there is no such sticker
				else:
					response.Status=1 # Id is taken
			else:
				response.Status=3 # no player available

		elif req.CMD=="Sensor":
			if self.IsGameStarted:
				# Send buffered sensor data 
				appliedPlayer=list(filter(lambda x:True \
					if x.ID==req.myPlayer.ID else False,self.Players))
				response.yourPlayer=appliedPlayer[0]
				response.yourPlayer.position=Vector4()
				response.Status=0 # no problem
			else:
				response.Status=10 # game is not started yet
		elif req.CMD=="Control":
			if self.IsGameStarted:
				# Add Control command to Buffer 
				appliedPlayer=list(filter(lambda x:True \
					if x.ID==req.myPlayer.ID else False,self.Players))
				
				response.Status=0 # no problem
				appliedPlayer[0].controlCommand=req.myPlayer.controlCommand
			else:
				response.Status=10 # game is not started yet
		elif req.CMD=="Start?":
			# to check if game start or not 
			if self.IsGameStarted:
				response.Status=0
			else:
				response.Status=10
		else:
			response.Status=5 # wtf
			
		return response
	


if __name__ == '__main__':
	h=Hub()









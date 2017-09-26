'''
	

	Modified: 18.06.2015 by Mehmet Efe Tiryaki (m.efetiryaki@gmail.com)
	Created : 06.06.2015 by Utku Norman ( norman.utku@metu.edu.tr )


'''


#!/usr/bin/env python

import serial
import sys
import thread as _thread 
import bluetooth as BT
#import _thread
from time import sleep
from collections import deque
from copy import deepcopy
#from struct import *
import struct
import math
import numpy as np

#.encode(encoding='UTF-8')

# Arduino Serial Interface Handler
class BluetoothInterfaceHandler:
	"""
	Serial interfacing class for Serial Communication with Arduino
	Modified 06.06.2015 by Utku Norman (norman.utku@metu.edu.tr) 
	"""

	def __init__( self , bdaddr = '93:D3:31:60:6E:54' , debugMode = False ):
		self.sock=BT.BluetoothSocket(BT.RFCOMM)
		self.bdaddr=bdaddr
		self.connected = False
		self.debugMode = debugMode
		self.delimiters = {
			'BEGIN' : 91 , # ASCII for [
			'END'	: 93   # ASCII for ]
			#'BEGIN' : 49 , # ASCII for 1
			#'END'	: 50   # ASCII for 2
		}
		try:
			self.sock.connect((bdaddr,1))
			self.connected = True
			#print("Robot Control : directly connected")
		except:
			self.sock.close()
			self.sock.recv(1024)
			#print("Robot Control : closed")
			self.sock.connect((bdaddr,1))
			#print("Robot Control : connected")
			self.connected = True
			

	def send( self , data ):
		# Put in a datagram
		msgInt = [ ]
		msgInt.append( self.delimiters[ 'BEGIN' ] )
		for element in data:
			msgInt.append( element )
		msgInt.append( self.delimiters[ 'END'   ] )

		self.sock.send("".join(map(chr,msgInt)))
		self.sock.recv(1024)
		if self.debugMode:
			print( "SerialInterfaceHandler: Sending..." )
			print( "data:   " , data   ) 
			print( "msgInt: " , msgInt ) 
			print( "msg:    " , msg    )
		#self.arduPort.close( )

	def recon( self ):
		self.arduPort = serial.Serial( self.portName , self.baudRate , timeout = 0.1 )


# Arduino Serial Data Packaging Handler
class SerialDataPacker:
	"""
	Serial Data Packaging Class for Serial Communication with Arduino
	Modified 06.06.2015 by Utku Norman (norman.utku@metu.edu.tr) 
	"""

	names    = { } # Dict for names to data indexes  
	initVals = [ ] # List of middle uSec values
	curVals  = [ ] # List of current list of values
	data     = [ ]

	def __init__( self , initVals , debugMode = False , defaultNumBytes = 2 ):
		"""
		Initialize by setting names
		Local values are held by dict structs independent of the init. method
		Orders in params should match.

		@param num:			The number of elements in a packet
							e.g. 
								Four omni driver
									4
		@type  num:			int
		@param initVals:		Holds a list middle position values in sending-indexed order
							e.g.
								Passing pulse width values for PWM signals, in uSecs
									[ 1400 , 1500 ]  
		@type  initVals:		List of ints
		@param debugMode:	Switches debug feedback
		@type  debugMode: 	bool
		"""
		self.defaultNumBytes = defaultNumBytes
		self.debugMode = debugMode
		self.initVals = initVals

		self.num = len( self.initVals )

		# Set the current values to be the middle values
		self.curVals = deepcopy( self.initVals ) 

		# Prepare data package
		self.packToData( )

		#Debug block
		if self.debugMode:
			print( "SerialDataPacker init:"    )
			print( "num:       " , self.num    )
			#print( "names:       " , self.names    )
			print( "data:        " , self.data     )

	def setVal( self , val , idx ):
		self.curVals[ idx ] = val
		self.packToData( )
		if self.debugMode:
			print( "setVal: val " , name   )
			print( "curVals:    " , self.curVals )
			print( "data:       " , self.data    )

	def setVals( self , vals ):
		if self.debugMode:
			print( "setVals:" )
			print( "old curVals: " , self.curVals )
		# Update curVals
		if self.debugMode: 
			print( "setting vals from list: " , vals )
		self.curVals = vals
		if self.debugMode:
			print( "new curVals:" , self.curVals )
		# Repack data
		self.packToData( )

	def valToTwoBytes(  self , val , endian = "MSByte" ):
		# Parse int to two bytes
		dataHighByte = int( int( val ) / 256 )
		dataLowByte  =      int( val ) % 256
		
		# Return considering byte endianness
		if   endian == "MSByte": # Most  Significant Byte First
			return ( dataHighByte , dataLowByte  ) # tuple
		elif endian == "LSByte": # Least Significant Byte First
			return ( dataLowByte  , dataHighByte ) # tuple

	def getCurVal( identifier ):
			
			return self.curVals[ self.names [ identifier ] ]

	def packToData( self ):
		self.data = [ ]
		for val in self.curVals:
			if self.defaultNumBytes == 2:
				( firstByte , secondByte ) = \
					self.valToTwoBytes( val )
				self.data.append( firstByte  )
				self.data.append( secondByte )
			elif self.defaultNumBytes == 1:
				self.data.append( val  )
		if self.debugMode:
			print( "packToData: data " , self.data )


class ThreeOmniDriver:
	"""
	Driver class for high-level control of a three omni-wheel robot.
	Modified 06.06.2015 by Utku Norman (norman.utku@metu.edu.tr) 

	PW : Pulse Width
	"""

	#minVals = [ 1300 , 1300 , 1300 , 1500 ]
	#midVals = [ 1490 , 1480 , 1530 , 1500 ] # Left Back Right N/A
	#maxVals = [ 1700 , 1700 , 1700 , 1500 ]

	# 0 
	# 1 -> 4
	# 2 -> 3 nolu robot
	# 3 -> 2
	# 4 ->
	#minSpeed = 0
	#maxSpeed = 100

	d = 1 #0.1

	a = math.sin( math.pi / 3 )
	b = math.cos( math.pi / 3 )

	#	h = [ [ - a ,   b ,   d ] ,1
	#		  [   0 , - 1 ,   d ] ,
	#		  [   a ,   b ,   d ] ]

	# vel[0] [ - 1 , 1 ]
	# vel[1] [ - 1 , 1 ]
	# vel[2] [ - 1 , 1 ]

	#maxVel = [    b + d ,       d , a + b + d ]
	#minVel = [      - d , - 1 - d ,       - d ]

	maxVel = [   a + b + d ,   1 + d ,   a + b + d ]
	minVel = [ - a - b - d , - 1 - d , - a - b - d ]

	#midVel = np.subtract( math.fabs( maxVel ) math.fabs( minVel ) ) / 2
	#midVel 
	midVel = [ 0 , 0 , 0 ]

	def __init__( self , minVals = None , midVals = None , maxVals = None , \
		bdaddr = None , dataPacker = None ):

		if minVals == None:
			self.minVals = [ 1300 , 1300 , 1300 , 1500 ]
		else: 
			self.minVals = minVals

		if midVals == None:
			self.midVals = [ 1500 , 1500 , 1500 , 1500 ] # Left Back Right N/A
		else:
			self.midVals = midVals

		if maxVals == None:
			self.maxVals = [ 1700 , 1700 , 1700 , 1500 ]
		else:
			self.maxVals = maxVals

		if bdaddr   == None:
			self.arduFace   = BluetoothInterfaceHandler( )
		else:
			self.arduFace =  BluetoothInterfaceHandler( bdaddr = bdaddr )

		if dataPacker == None:
			self.dataPacker = SerialDataPacker( self.midVals )
		else: 
			self.dataPacker =  dataPacker

	def stop( self ):
		print( "stopping" , self.dataPacker.initVals )
		self.dataPacker.setVals( self.dataPacker.initVals )
		self.arduFace.send( self.dataPacker.data )

	def drive( self , vals = None , vel = None ):
		"""drive the robot. Push if no new value specified
		"""

		if vals != None:
			self.dataPacker.setVals( vals )

		elif vel != None:
			( linVelX , linVelY , angVelZ ) = vel 

			#angVelZ = math.radians( float( angVelZ ) )
			vals = self.velToSpeeds( vel )

			self.dataPacker.setVals( vals )

		self.arduFace.send( self.dataPacker.data )

	def speedToPW( self , speed , idx ): 
		"""convert by scaling a speed value to a pulse width value"""
		scaler =  float ( ( self.maxVals[ idx ] - self.midVals[ idx ] ) 
			/ ( self.maxVel[ idx ] - self.midVel[ idx ] ) )
		#print( "TOD: scaler: " , scaler , " idx:" , idx )
		#print( "TOD: scaler: " , scaler , " idx:" , idx )
		pw = self.midVals[ idx ] + speed * scaler
		#print( "pw " , pw )
		return pw;

	def velToSpeeds( self , vel ):
		"""
		Convert Velocity tuple ( vel ) to the Pulse Widths ( pws ) tuple

		using the control model in 
		http://www.inescporto.pt/~hfpo/papers/Oliveira_CRCS_INTECH.pdf p.211

		Velocity tuple:
			vel = ( linVelX , linVelY , angVelZ )
				linVelP : magnitude of the linear  velocity at the p direction
				angVelP : magnitude of the angular velocity at the p direction
 			Incoming drive command

		Pulse Widths list :
			pws = ( pw_0 , pw_1 , pw_2 , 1500 )
				pw_i : pulse width value for the ith motor
			Pulse width values in uSec for each
		"""
		STRAIGHT = True
		REVERSE  = False

		( linVelX , linVelY , angVelZ ) = vel

		a = self.a
		b = self.b
		d = self.d

		# The control model
		# h: the transfer function
		# d: distance between the wheels and the center of the robot [ m ]
		h = [ [ - a ,   b ,   d ] ,
			  [   0 , - 1 ,   d ] ,
			  [   a ,   b ,   d ] ]

		speeds = np.dot( h , vel ) 

		#print( "TOD: vel" , vel )

		#print( "TOD: speeds" , speeds )

		vals = [ math.floor ( self.speedToPW( speeds[ 0 ] , 0 ) ) , \
				 math.floor ( self.speedToPW( speeds[ 1 ] , 1 ) ) , \
				 math.floor ( self.speedToPW( speeds[ 2 ] , 2 ) ) , \
				 1500 ]

		return vals

rcvBuffer = deque( )

////////////////////////////////////////////////////////////////////////////////
//
// SerialDataHandler.h - Class Header File
//
// Description: A wrapper-class to frame packets in Serial Communication 
//    with Arduino
// Modified : 06.06.2015 by Utku Norman ( norman.utku@metu.edu.tr )
// Created  : 25.05.2015 by Utku Norman ( norman.utku@metu.edu.tr )
//
// Reference(s) :
// Template use :
//  http://www.codeproject.com/Articles/48575/How-to-define-a-template-class-in-a-h-file-and-imp
//
////////////////////////////////////////////////////////////////////////////////
//#include <String.h>
//#include <SerialDataHandler.h>
#include "Arduino.h"

#ifndef SerialDataHandler_h
#define SerialDataHandler_h

//#define SIZE 8

// Length of data in a wrapped / framed segment in bytes
// Timeout duration in miliseconds
template < int DATA_LEN >  
class SerialDataHandler
{
  public:
    SerialDataHandler( char , char , bool );
    void clearBuffer( void );
    bool   writeByte( unsigned char );
    int      readInt( char );
    unsigned char    readByte( unsigned char );
    bool isReceiving;
    bool  isBuffered;

  private:
    void       _saveByte( unsigned char );
    void _receiveSuccess( void );
    void    _receiveFail( void );
    void      _resetData( void );
    bool    _debugMode;
    char    _startByte;
    char      _endByte;
    int      _rcvIndex;
    unsigned char _data[ DATA_LEN ];

};

#endif

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// SerialDataHandler.cpp - Class Source File
//
// Description:  A wrapper-class to frame packets in Serial Communication 
//    with Arduino
// Modified : 06.06.2015 by Utku Norman ( norman.utku@metu.edu.tr )
// Created  : 25.05.2015 by Utku Norman ( norman.utku@metu.edu.tr )
//
////////////////////////////////////////////////////////////////////////////////
//#include <String.h>
//#include <SerialDataHandler.h>

// Initialize Class Variables //////////////////////////////////////////////////
/*
int SerialDataHandler::rcvIndex = 0;
String SerialDataHandler::_data = "";
bool SerialDataHandler::isReceiving = false;
bool SerialDataHandler::isBuffered = false;
*/

// Constructors ////////////////////////////////////////////////////////////////
template < int DATA_LEN > 
SerialDataHandler < DATA_LEN >::SerialDataHandler( 
  char startByte , char endByte , bool debugMode )
{
  //clearBuffer( );

  _debugMode   = debugMode ;
  _startByte   = startByte ; 
  _endByte     = endByte   ;
  
}

// Public Methods //////////////////////////////////////////////////////////////
template < int DATA_LEN >
void SerialDataHandler< DATA_LEN >::clearBuffer( void ) {

  isReceiving = false ;
  isBuffered  = false ;

  _rcvIndex = 0;
  _resetData( );

  if ( _debugMode ) {
        Serial.println( "SDH: Buffer cleared." );
  }

}

template < int DATA_LEN >
bool SerialDataHandler< DATA_LEN>::writeByte( unsigned char receivedByte ) {

  // Analyze the received byte
  if ( !isReceiving ) {
    if ( receivedByte == _startByte ) {
      isReceiving = true;
      _rcvIndex = 0;
      /*
      if ( _debugMode ) {
        Serial.println( );
        Serial.println( "SDH: Activated." );
        Serial.print( "SDH: DATA_LEN:" );
        Serial.println( DATA_LEN );
      }
      */
    }
  }
  else { // if receiving
    if( _rcvIndex == DATA_LEN ) {
      if( receivedByte == _endByte ) {
        _receiveSuccess( );
      }
      else {
        _receiveFail( );
      }
    }
    else if( _rcvIndex > DATA_LEN ) {
      _receiveFail( );
    }
    else {
        _saveByte( receivedByte ) ;
        return true;
    }
  }

  return false;
}

// int is 16-bit in arduino
// Ref:
//  https://learn.sparkfun.com/tutorials/data-types-in-arduino
//  int (16 bit) - signed number from -32768 to 32767. 
//  This is most commonly what you see used for general purpose variables in Arduino 
//  example code provided with the IDE
template < int DATA_LEN > 
int SerialDataHandler< DATA_LEN >::readInt( char dataStartIndex ) {
  // Most significant byte first encoding
  return _data[ dataStartIndex ] * 256 + _data[ dataStartIndex + 1 ];
}

// byte (8 bit) - unsigned number from 0-255
// char (8 bit) - signed number from -128 to 127. 
// The compiler will attempt to interpret this data type as a character 
// in some circumstances, which may yield unexpected results
template < int DATA_LEN >
unsigned char SerialDataHandler< DATA_LEN >::readByte( unsigned char dataStartIndex ) {
  return _data[ dataStartIndex ];
}

// Private Methods /////////////////////////////////////////////////////////////
template < int DATA_LEN  >
void SerialDataHandler< DATA_LEN >::_saveByte( unsigned char receivedByte ) {

  /*
 if ( _debugMode ) {
    Serial.print( "SDH: byte saved: " );
    Serial.println ( ( unsigned char ) receivedByte ); 
    Serial.print( "SDH: at index: " );
    Serial.println ( _rcvIndex ); 
  }
 */

  _data[ _rcvIndex ] = receivedByte;
  _rcvIndex = _rcvIndex + 1; 

}

template < int DATA_LEN >
void SerialDataHandler< DATA_LEN >::_receiveSuccess( ) {

  /*if ( _debugMode  ) {
    Serial.println( );
    Serial.print( "SDH: data received as: " );
    Serial.println ( ( char )_data ); 
  }*/

  if ( _debugMode ) {
  Serial.println( "SDH: Receive success." );
  }
  
  isReceiving = false ;
  isBuffered  = true  ;

}

template < int DATA_LEN >
void SerialDataHandler< DATA_LEN >::_receiveFail( ) {

  if ( _debugMode  ) {
    Serial.println( "SDH: receive failed." );
  }
  clearBuffer( );

}

template < int DATA_LEN >
void SerialDataHandler< DATA_LEN >::_resetData( ) {
  for( char i = 0 ; i < DATA_LEN ; i ++ ) {
    _data[ i ] = 0;        
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// arduDrive.ino - Arduino script file
//
// Description: A slave multiple servo driver code for Arduino
//
// Modified : 06.06.2015 by Utku Norman ( norman.utku@metu.edu.tr )
// Created  : 25.05.2015 by Utku Norman ( norman.utku@metu.edu.tr )
//
////////////////////////////////////////////////////////////////////////////////

// Libraries ///////////////////////////////////////////////////////////////////
#include <Servo.h>               // servo library to drive servos

// Obselete, since the software serial is tested to be not reliable enough.
//#include <SoftwareSerial.h>      // "virtual" serial port/UART library

//#include <SerialDataHandler.h> // data packaging library for framing a group of commands
// Uncomment this to import the data packager header if exists in a separate file

// Constructors ////////////////////////////////////////////////////////////////

// A "virtual" serial port/UART with < RX , TX > = < pin D10 , pin D11 >
// Obselete, since the software serial is tested to be not reliable enough.
// SoftwareSerial Serial( 10 , 11 ); 

// Construct the servo objects
Servo servo0; 
Servo servo1;
Servo servo2;
Servo servo3;
// A data frame of 8 bytes of data with '[' and ']' as start and end wrapper bytes.
// Notes: The frame is 10 bytes in total, the 3rd param. is to enable debug.
SerialDataHandler< 8 > serialHandler( ( unsigned char )'[' , ( unsigned char )']' , true );
// Temporary storage for an incoming byte
unsigned char receivedByte;
bool debugMode = true;

// Setup Commands //////////////////////////////////////////////////////////////
void setup( ) {

  // Turn the Serial Protocol ON at the specified baudRate
  Serial.begin( 115200 ); 

  // Send message to other device
  Serial.println( "Hello World! sayeth the triBot." );

  int servoPins[] = { 3 , 5 , 6 , 9 };

  // Map pin numbers to the servo objects
  // Note that e.g. 3 is D3, the digital pin number 3
  servo0.attach( servoPins[ 0 ] );
  servo1.attach( servoPins[ 1 ] );
  servo2.attach( servoPins[ 2 ] );
  servo3.attach( servoPins[ 3 ] );

}

// The Main Loop ///////////////////////////////////////////////////////////////
void loop( ) {

  // Save if an incoming serial byte exists
  if ( Serial.available( ) > 0 ) {  // Check buffer for incoming byte

    // Read incoming byte from Serial buffer into variable
    receivedByte = Serial.read( );  
    // Serial.write( receivedByte );   // Echo
    
    // Pass the byte read to the serial handler
    serialHandler.writeByte( receivedByte );

  }

  // If a whole frame is received
  if ( serialHandler.isBuffered ) {
   
      // Assign the integer values in the frame with the begining indices:
      servo0.writeMicroseconds( serialHandler.readInt( 0 ) );
      servo1.writeMicroseconds( serialHandler.readInt( 2 ) );
      servo2.writeMicroseconds( serialHandler.readInt( 4 ) );
      servo3.writeMicroseconds( serialHandler.readInt( 6 ) );

      // Feedback debug strings
      if ( debugMode ) {
          Serial.println( "arBot speaking:" );
          Serial.print( "Servo [ 0 1 2 3 ] pulse width values (uS) :[" );
          Serial.print( " " );
          Serial.print( serialHandler.readInt( 0 ) );
          Serial.print( " " );
          Serial.print( serialHandler.readInt( 2 ) );
          Serial.print( " " );
          Serial.print( serialHandler.readInt( 4 ) );
          Serial.print( " " );
          Serial.print( serialHandler.readInt( 6 ) );
          Serial.print( " " );
          Serial.println( "]" );
      }

      // Drop the package since it is processed
      serialHandler.clearBuffer( );
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////





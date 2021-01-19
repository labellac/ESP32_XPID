/*
	X-Sim PID
  This is a branch of the original program developed by Martin Wiedenbauer that replace the control of two motor H-Bridge with analogue feedback 
  with two open loop stepper motors. Target is an ESP32.

	Command input protocol for main serial
  'X''S'{16 bit left position value}{16 bit right position value}  Set Left and Right target position value
  'X''E'{bool}"        Enable left and right motor true=enabled, false=disabled
  'X''P'{value}"       Proportional value for left and right motor
  'X''I'{value}"       Integral value for left and right motor
  'X''D'{value}"       Derivative value for left and right motor
  'X''H'               Homing cycle

  Command input/outpu for debug serial
  TODO!
	'X' 200 0 0 C			Send back over serial port both setpper position raw values
	'X' 201 0 0 C			Send back over serial port the current pid count
	'X' 202 0 0 C			Send back over serial port the firmware version (used for x-sim autodetection)
	'X' 203 M V C			Write EEPROM on address M (only 0 to 255 of 1024 Bytes of the EEPROM) with new value V
	'X' 204 M 0 C			Read EEPROM on memory address M (only 0 to 255 of 1024 Bytes of the EEPROM), send back over serial the value
	'X' 205 0 0 C			Clear EEPROM
	'X' 206 0 0 C			Reread the whole EEPRom and store settings into fitting variables
	'X' 207 0 0	C			Disable power on motor 1
	'X' 208 0 0	C			Disable power on motor 2
	'X' 209 0 0	C			Enable power on motor 1
	'X' 210 0 0	C			Enable power on motor 2
	'X' 211 0 0	C			Send all debug values
	

	Pin out of ESP32 for steppers

	GPIO 21 - Step pulse for left stepper. 
	GPIO 19 - Direction signal for left stepper.
	GPIO 18 - Enable signal for left stepper 
	GPIO  4 - Stop switch for left stepper
  GPIO 32 - Step pulse for right stepper. 
  GPIO 25 - Direction signal for right stepper.
  GPIO 26 - Enable signal for right stepper 
  GPIO 27 - Stop switch for right stepper

	Auxiliar signals

	GPIO 14 - Emergency stop
	GPIO 13 - Main relay control
  GPIO 16  - Debug RX
  GPIO 17 - Debug TX

*/

#include <stdio.h>

//Defines
#define DEBUG_BUFFER_SIZE 64  //Debug buffer size in bytes
#define MAIN_BUFFER_SIZE  64  //Debug buffer size in bytes
#define TASK_DELAY_100MS  100 //Task delay in miliseconds
#define TASK_DELAY_10MS   10  //Task delay in miliseconds
#define DEBUG_TX_PIN      17  //Debug tx pin on ESP32
#define DEBUG_RX_PIN      16  //Debug rx pin on ESP32

//Global variables

  //Debug serial variables
char mainRBufferCount=-1;
char mainRBuffer[MAIN_BUFFER_SIZE]={0};
char mainWBufferCount=0;
char mainWBuffer[MAIN_BUFFER_SIZE]={0};
  //Debug serial variables
char debugRBufferCount=-1;
char debugRBuffer[DEBUG_BUFFER_SIZE]={0};
char debugWBufferCount=0;
char debugWBuffer[DEBUG_BUFFER_SIZE]={0};

//Function prototypes
void ParseCommand(char * rcvdBuffer);

//Tasks
void mainSerialTask (void * parameter)
{
  const TickType_t delay_10ms = TASK_DELAY_10MS / portTICK_PERIOD_MS;

  char buffer=0;
  //Initialize main serial
  Serial.begin(115200);

  while(true)
  {
    if(Serial.available()) 
    {
      if(mainRBufferCount==-1)
      {
        buffer = Serial.read();
        if(buffer != 'X'){mainRBufferCount=-1;}else{mainRBufferCount=0;}
      }
      else
      {
        buffer = Serial.read();
        mainRBuffer[mainRBufferCount]=buffer;
        mainRBufferCount++;
        if(buffer=='\r')
        {
          //Debug received data:
          //add null terminator
          //mainRBuffer[mainRBufferCount-1]='\0';
          //add received data to debug buffer
          //debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Received data: %s \r\n",mainRBuffer);
          //reset main buffer count
          
          mainRBufferCount=-1;

          ParseCommand(mainRBuffer);
        }
      }
    }
    vTaskDelay(delay_10ms);
  }
}

void debugSerialTask (void * parameter)
{
  const TickType_t delay_100ms = TASK_DELAY_100MS / portTICK_PERIOD_MS;

  char buffer=0;
  //Initialize debug serial
  Serial2.begin(115200,SERIAL_8N1,DEBUG_RX_PIN,DEBUG_TX_PIN);

  while(true)
  {
    //Read debug serial for manage internal commands
    if(Serial2.available()) 
    {
      if(debugRBufferCount==-1)
      {
        buffer = Serial2.read();
        if(buffer != 'X'){debugRBufferCount=-1;}else{debugRBufferCount=0;}
      }
      else
      {
        buffer = Serial2.read();
        debugRBuffer[debugRBufferCount]=buffer;
        debugRBufferCount++;
        if(buffer=='\r')
        {
          //ParseCommand(commandbuffer);
          debugRBufferCount=-1;
          //SendDebug()
        }
      }
    }

    //Write debug buffer to debug serial output
    for(int i = 0; i < debugWBufferCount; i++)
    {
      Serial2.write(debugWBuffer[i]);
    }

    debugWBufferCount=0;

    vTaskDelay(delay_100ms);
  }
}

void ParseCommand(char * rcvdBuffer)
{
  int pos=0;
  switch(rcvdBuffer[0])
  {
    //Command for setting motors target position
    case 'S':
      pos=rcvdBuffer[1]*256+rcvdBuffer[2];
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Pos: %d \r\n",pos);
      break;
    //Command for enable/disable motors
    case 'E':
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Enable: %d \r\n",rcvdBuffer[1]);
      break;
    //Command for setting Proportional value
    case 'P':
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Proportional: %d \r\n",rcvdBuffer[1]);
      break;
    //Command for setting Integral value
    case 'I':
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Integral: %d \r\n",rcvdBuffer[1]);
      break;
    //Command for setting Derivative value
    case 'D':
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Derivative: %d \r\n",rcvdBuffer[1]);
      break;
    //Command for setting homing cycle
    case 'H':
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Homing cycle \r\n");
      break;
    default:
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Invalid command \r\n");
      break;
      
  }  
}

void setup()
{

  xTaskCreate(
                    mainSerialTask,          /* Task function. */
                    "MainSerialTask",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */

  xTaskCreate(
                    debugSerialTask,          /* Task function. */
                    "DebugSerialTask",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */

}

void loop()
{
	
}

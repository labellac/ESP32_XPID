/*
	X-Sim PID
  This is a branch of the original program developed by Martin Wiedenbauer that replace the control of two motor H-Bridge with analogue feedback 
  with two open loop stepper motors. Target is an ESP32, but it should also work with Atmel 328.
	Copyright (c) 2013 Martin Wiedenbauer, particial use is only allowed with a reference link to the x-sim.de project

	Command input protocol	(always 5 bytes, beginning with 'X' character and ends with a XOR checksum)
	'X' 1 H L C				Set motor 1 position to High and Low value 0 to 1023
	'X' 2 H L C				Set motor 2 position to High and Low value 0 to 1023
	'X' 3 H L C				Set motor 1 P Proportional value to High and Low value
	'X' 4 H L C				Set motor 2 P Proportional value to High and Low value
	'X' 5 H L C				Set motor 1 I Integral value to High and Low value
	'X' 6 H L C				Set motor 2 I Integral value to High and Low value
	'X' 7 H L C				Set motor 1 D Derivative value to High and Low value
	'X' 8 H L C				Set motor 2 D Derivative value to High and Low value

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
	
	EEPROM memory map
	00		empty eeprom detection, 111 if set, all other are indicator to set default
	01-02	minimum 1
	03-04	maximum 1
	05		dead zone 1
	06-07	minimum 2
	08-09	maximum 2
	10		dead zone 2
	11-12	P component of motor 1
	13-14	I component of motor 1
	15-16	D component of motor 1
	17-18	P component of motor 2
	19-20	I component of motor 2
	21-22	D component of motor 2
	23		pwm1 offset
	24		pwm2 offset
	25		pwm1 maximum
	26		pwm2 maximum
	27		pwm frequency divider (1,8,64)

	Pin out of ESP32 for steppers

	GPIO 23 - Step pulse for left stepper. 
	GPIO 22 - Direction signal for left stepper.
	GPIO  1 - Enable signal for left stepper 
	GPIO  3 - Stop switch for left stepper
  GPIO 36 - Step pulse for right stepper. 
  GPIO 39 - Direction signal for right stepper.
  GPIO 34 - Enable signal for right stepper 
  GPIO 35 - Stop switch for right stepper

	Auxiliar signals

	GPIO 14 - Emergency stop
	GPIO 12 - Main relay control
  GPIO 9  - Debug RX
  GPIO 10 - Debug TX

*/

#include <EEPROM.h>
#include "FastAccelStepper.h"

#define LOWBYTE(v)   ((unsigned char) (v))								//Read
#define HIGHBYTE(v)  ((unsigned char) (((unsigned int) (v)) >> 8))
#define BYTELOW(v)   (*(((unsigned char *) (&v) + 1)))					//Write
#define BYTEHIGH(v)  (*((unsigned char *) (&v)))

#define   GUARD_MOTOR_1_GAIN   100.0     
#define   GUARD_MOTOR_2_GAIN   100.0

//Steppers defines

#define   REVOLUTION_ANGLE     360    //Complete revolution angle in degrees
#define   OPERATION_ANGLE      100     //Desired operation angle in degrees
#define   REDUCTION_RATIO      60     //GearBox reduction ratio
#define   STEPS_PER_REVOLUTION 200    //Motor steps per revolution
#define   SECONDS_TO_MICROS    (long)1000000  //Divide ratio for seconds to useconds
#define   N_MOTORS             2      //Number of used motors
#define   MOTOR_ONE            0      //Motor one index
#define   MOTOR_TWO            1      //Motor two index
#define   HOMING_SPEED         1000    //Homing speed in steps/second
#define   HOMING_OFFSET        200    //Home offset in steps
#define   HOMING_TIMEOUT       5000   //Home cycle time out in ms
#define   MOTOR_MAX_SPEED      4000    //Motor max speed in steps/second
#define   MOTOR_MIN_SPEED      300     //Motor min speed in steps/second
#define   MOTOR_ACCELERATION   10000   //Motor acceleration in step/s^2 

//Stop switch debounce time

#define   DEBOUNCE_TIME        20    //Debounce time in decens of ms

//Pin defines
#define STEP_L_PIN  23
#define DIR_L_PIN  22
#define EN_L_PIN  1
#define SW_L_PIN  3
#define STEP_R_PIN  36
#define DIR_R_PIN  39
#define EN_R_PIN  34
#define SW_R_PIN  35
#define EMERGENCY_STOP_PIN 14
#define MAIN_RELAY_PIN  12
#define DEBUG_RX 9
#define DEBUG_TX 10

//other defines

#define EEPROM_SIZE 64

//Firmware version info
int firmaware_version_mayor=3;
int firmware_version_minor =0;

int virtualtarget1;
int virtualtarget2;
int currentanalogue1 = 0;
int currentanalogue2 = 0;
int target1=512;
int target2=512;
int low=0;
int high=0;
unsigned long hhigh=0;
unsigned long hlow=0;
unsigned long lhigh=0;
unsigned long llow=0;
int buffer=0;
int buffercount=-1;
int commandbuffer[5]={0};
unsigned long pidcount	= 0;		// unsigned 32bit, 0 to 4,294,967,295
byte errorcount	= 0;		// serial receive error detected by checksum

//Motor data

float AngleRatio = (float)OPERATION_ANGLE/(float)REVOLUTION_ANGLE;
int StepsPerRevolution = STEPS_PER_REVOLUTION*REDUCTION_RATIO;
int MaxStepValue = (int)((float)StepsPerRevolution * AngleRatio);
int MinStepValue = 0;

struct stepper_config_s {
  uint8_t step;
  uint8_t enable_low_active;
  uint8_t enable_high_active;
  uint8_t stop_switch_low_active;
  uint8_t stop_switch_high_active;
  uint8_t direction;
  bool direction_high_count_up;
  bool auto_enable;
  uint32_t on_delay_us;
  uint16_t off_delay_ms;
};

const uint8_t led_pin = 2;
const struct stepper_config_s stepper_config[N_MOTORS] = {
    {
      step : STEP_L_PIN,
      enable_low_active : EN_L_PIN,
      enable_high_active : PIN_UNDEFINED,
      stop_switch_low_active : SW_L_PIN,
      stop_switch_high_active: PIN_UNDEFINED,
      direction : DIR_L_PIN,
      direction_high_count_up : false,
      auto_enable : true,
      on_delay_us : 50,
      off_delay_ms : 10
    },
    {
      step : STEP_R_PIN,
      enable_low_active : EN_R_PIN,
      enable_high_active : PIN_UNDEFINED,
      stop_switch_low_active : SW_R_PIN,
      stop_switch_high_active: PIN_UNDEFINED,
      direction : DIR_R_PIN,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 50,
      off_delay_ms : 10
    }
};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper[N_MOTORS];

int OutputM1 = 0;
int OutputM2 = 0;

// Position data
int FeedbackMax1			= MaxStepValue;		// Maximum position of pot 1 to scale, do not use 1023 because it cannot control outside the pot range
int FeedbackMin1			= 0;		// Minimum position of pot 1 to scale, do not use 0 because it cannot control outside the pot range
int FeedbackMax2			= MaxStepValue;		// Maximum position of pot 2 to scale, do not use 1023 because it cannot control outside the pot range
int FeedbackMin2			= 0;		// Minimum position of pot 2 to scale, do not use 0 because it cannot control outside the pot range
int FeedbackPotDeadZone1	= 5;		// +/- of this value will not move the motor		
int FeedbackPotDeadZone2	= 5;		// +/- of this value will not move the motor

//PID variables
int motordirection1		= 0;			// motor 1 move direction 0=brake, 1=forward, 2=reverse
int motordirection2		= 0;			// motor 2 move direction 0=brake, 1=forward, 2=reverse
int oldmotordirection1	= 0;
int oldmotordirection2	= 0;
double K_motor_1		= 1;
double proportional1	= 4.200;		//initial value
double integral1		= 0.100;
double derivative1		= 0.100;
double K_motor_2		= 1;
double proportional2	= 4.200;
double integral2		= 0.100;
double derivative2		= 0.100;

double integrated_motor_1_error = 0;
double integrated_motor_2_error = 0;
float last_motor_1_error		= 0;
float last_motor_2_error		= 0; 
int disable						= 1; //Motor stop flag

//debug global variables
byte debugbyte =0;				//This values are for debug purpose and can be send via
int debuginteger =0;			//the SendDebug serial 211 command to the X-Sim plugin
double debugdouble =0;

//Loop update variable
volatile bool enableSpeedUpdate=false;

//debounce struct definition
struct debounce_s{
  bool enabled;
  uint8_t debounce_cnt;
};

struct debounce_s debounce[N_MOTORS]={{false,0},{false,0}};

//strcuct emergency stop switch
struct emergency_stop_switch_s{
  bool activeLow;
  bool currentState;
  bool oldState;
  uint8_t pin;
  uint8_t debounce_cnt;  
};

struct emergency_stop_switch_s emergencyStopSwitch={activeLow:false, currentState:0,oldState:0,pin:EMERGENCY_STOP_PIN,debounce_cnt:0};

hw_timer_t * timer = NULL;

//Timer interrupt handler

void IRAM_ATTR debounceTimer() {
  
  uint8_t i;
  uint8_t emer_stop=0;

  enableSpeedUpdate=true;
  for(i=0;i<N_MOTORS;i++){
    if(debounce[i].enabled==true)
    {
      if(debounce[i].debounce_cnt>DEBOUNCE_TIME){
        debounce[i].enabled=false;
        debounce[i].debounce_cnt=0;
      }
      else
      {
        debounce[i].debounce_cnt++;
      }
    }
  }

  //check emergency stop
  emergencyStopSwitch.currentState=digitalRead(emergencyStopSwitch.pin);
  if(emergencyStopSwitch.currentState==0)
  {
    stepper[MOTOR_ONE]->stopMove();
    stepper[MOTOR_TWO]->stopMove();
    DisableMainRealay();
  }
}



void IRAM_ATTR stopSwitchOneIsr(){
  if(debounce[MOTOR_ONE].enabled==false)
  {
    debounce[MOTOR_ONE].enabled=true;
    //Serial.println("Interrupt!");
    stepper[MOTOR_ONE]->stopMove();
    stepper[MOTOR_ONE]->setCurrentPosition(-HOMING_OFFSET);
  }
}

void IRAM_ATTR stopSwitchTwoIsr(){
  if(debounce[MOTOR_TWO].enabled==false)
  {
    debounce[MOTOR_TWO].enabled=true;
    //Serial.println("Interrupt!");
    stepper[MOTOR_TWO]->stopMove();
    stepper[MOTOR_TWO]->setCurrentPosition(-HOMING_OFFSET);
  }
}

void setup()
{
  //Initialize serial ports
	Serial.begin(115200);   //Uncomment this for arduino UNO without ftdi serial chip
  Serial2.begin(115200,SERIAL_8N1,DEBUG_RX,DEBUG_TX);
  Serial2.println("DEBUG PORT ENABLED!");
  //Initialize eeprom
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM"); delay(1000);
  }

  //Initialize timer, update every 10 ms
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &debounceTimer, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);

  //Configure stop stiwch pins and interrupts
  pinMode(stepper_config[MOTOR_ONE].stop_switch_low_active,INPUT_PULLUP);  //Configure stop switch pin as input pullup
  pinMode(stepper_config[MOTOR_TWO].stop_switch_low_active,INPUT_PULLUP);

  attachInterrupt(stepper_config[MOTOR_ONE].stop_switch_low_active, stopSwitchOneIsr, FALLING);
  attachInterrupt(stepper_config[MOTOR_TWO].stop_switch_low_active, stopSwitchTwoIsr, FALLING);

  //Configure emergency stop input
  pinMode(EMERGENCY_STOP_PIN,INPUT_PULLUP);
  //Configure main relay pin output and set to disable
  pinMode(MAIN_RELAY_PIN,OUTPUT);
  digitalWrite(MAIN_RELAY_PIN,LOW);

  //Init stepper engine
  engine.init();
  if (led_pin != PIN_UNDEFINED) {
    engine.setDebugLed(led_pin);
  }

  for (uint8_t i = 0; i < N_MOTORS; i++) {
    FastAccelStepper *s = NULL;
    const struct stepper_config_s *config = &stepper_config[i];
    if (config->step != PIN_UNDEFINED) {
      s = engine.stepperConnectToPin(config->step);
      if (s) {
        s->setDirectionPin(config->direction, config->direction_high_count_up);
        s->setEnablePin(config->enable_low_active, true);
        s->setEnablePin(config->enable_high_active, false);
        s->setAutoEnable(config->auto_enable);
        s->setDelayToEnable(config->on_delay_us);
        s->setDelayToDisable(config->off_delay_ms);
        s->setAcceleration(MotorAcceleration);
        s->setSpeed(SECONDS_TO_MICROS/MotorMaxSpeed);
      }
    }
    stepper[i] = s;
  }
  
  uint8_t res = HomingCycle();
  
  if(res!=0){
    Serial2.println("Homing cycle failed! Setting current position as 0");
    if((res & 2)!=0){
      Serial2.println("Motor one stop switch fail!");
      stepper[MOTOR_ONE]->forceStopAndNewPosition(0);
    }
    if((res & 4)!=0){
      Serial2.println("Motor two stop switch fail!");
      stepper[MOTOR_TWO]->forceStopAndNewPosition(0);
    }
  }
}

uint8_t HomingCycle(void)
{
  int initTime=0;
  int returnVal=0;
  
  stepper[MOTOR_ONE]->setSpeed(SECONDS_TO_MICROS/HOMING_SPEED);
  stepper[MOTOR_TWO]->setSpeed(SECONDS_TO_MICROS/HOMING_SPEED);
  stepper[MOTOR_ONE]->runBackward();
  stepper[MOTOR_TWO]->runBackward();
  //stepper[MOTOR_ONE]->enableOutputs();
  //stepper[MOTOR_TWO]->enableOutputs();

  initTime=millis();

  delay(100);
  //Run motors until stop switch is hit
  while((stepper[MOTOR_ONE]->isRunning())||(stepper[MOTOR_TWO]->isRunning())){
    if((millis()-initTime)>HOMING_TIMEOUT){
      //If timeout expired, return home cycle error
      if(stepper[MOTOR_ONE]->isRunning()){
        returnVal|=2;
      }
      if(stepper[MOTOR_TWO]->isRunning()){
        returnVal|=4;
      }
      returnVal|=1;
      break;
    }
  }
  
  return returnVal;
}

void WriteEEPRomWord(int address, int intvalue)
{
	int low,high;
	high=intvalue/256;
	low=intvalue-(256*high);
	EEPROM.write(address,high);
	EEPROM.write(address+1,low);
}

int ReadEEPRomWord(int address)
{
	int low,high, returnvalue;
	high=EEPROM.read(address);
	low=EEPROM.read(address+1);
	returnvalue=(high*256)+low;
	return returnvalue;
}

void WriteEEProm()
{
	EEPROM.write(0,111);
	WriteEEPRomWord(1,FeedbackMin1);
	WriteEEPRomWord(3,FeedbackMax1);
	EEPROM.write(5,FeedbackPotDeadZone1*10);
	WriteEEPRomWord(6,FeedbackMin2);
	WriteEEPRomWord(8,FeedbackMax2);
	EEPROM.write(10,FeedbackPotDeadZone2*10);
	WriteEEPRomWord(11,int(proportional1*10.000));
	WriteEEPRomWord(13,int(integral1*10.000));
	WriteEEPRomWord(15,int(derivative1*10.000));
	WriteEEPRomWord(17,int(proportional2*10.000));
	WriteEEPRomWord(19,int(integral2*10.000));
	WriteEEPRomWord(21,int(derivative2*10.000));
}

void ReadEEProm()
{
	int evalue = EEPROM.read(0);
	if(evalue != 111) //EEProm was not set before, set default values
	{
		WriteEEProm();
		return;
	}
	FeedbackMin1=ReadEEPRomWord(1);
	FeedbackMax1=ReadEEPRomWord(3);
	FeedbackPotDeadZone1=EEPROM.read(5);
	FeedbackMin2=ReadEEPRomWord(6);
	FeedbackMax2=ReadEEPRomWord(8);
	FeedbackPotDeadZone2=EEPROM.read(10);
	proportional1=double(ReadEEPRomWord(11))/10.000;
	integral1=double(ReadEEPRomWord(13))/10.000;
	derivative1=double(ReadEEPRomWord(15))/10.000;
	proportional2=double(ReadEEPRomWord(17))/10.000;
	integral2=double(ReadEEPRomWord(19))/10.000;
	derivative2=double(ReadEEPRomWord(21))/10.000;
}

void SendAnalogueFeedback(int analogue1, int analogue2)
{
	high=analogue1/256;
	low=analogue1-(high*256);
	Serial.write('X');
	Serial.write(200);
	Serial.write(high);
	Serial.write(low);
	high=analogue2/256;
	low=analogue2-(high*256);
	Serial.write(high);
	Serial.write(low);
}

void SendPidCount()
{
	unsigned long value=pidcount;
	hhigh=value/16777216;
	value=value-(hhigh*16777216);
	hlow=value/65536;
	value=value-(hlow*65536);
	lhigh=value/256;
	llow=value-(lhigh*256);
	Serial.write('X');
	Serial.write(201);
	Serial.write(int(hhigh));
	Serial.write(int(hlow));
	Serial.write(int(lhigh));
	Serial.write(int(llow));
	Serial.write(errorcount);
}

void SendDebugValues()
{
	//The double is transformed into a integer * 10 !!!
	int doubletransfere=int(double(debugdouble*10.000));
	Serial.write('X');
	Serial.write(211);
	Serial.write(debugbyte);
	Serial.write(HIGHBYTE(debuginteger));
	Serial.write(LOWBYTE(debuginteger));
	Serial.write(HIGHBYTE(doubletransfere));
	Serial.write(LOWBYTE(doubletransfere));
}

void SendFirmwareVersion()
{
	Serial.write('X');
	Serial.write('-');
	Serial.write('P');
	Serial.write('I');
	Serial.write('D');
	Serial.write(' ');
	Serial.write(48+firmaware_version_mayor);
	Serial.write('.');
	Serial.write(48+firmware_version_minor);
}

void EEPromToSerial(int eeprom_address)
{
	int retvalue=EEPROM.read(eeprom_address);
	Serial.write('X');
	Serial.write(204);
	Serial.write(retvalue);
}

void ClearEEProm()
{
	for(int z=0; z < 1024; z++)
	{
		EEPROM.write(z,255);
	}
}

void ParseCommand()
{
	if(commandbuffer[0]==1)			//Set motor 1 position to High and Low value 0 to 1023
	{
		target1=(commandbuffer[1]*256)+commandbuffer[2];
		disable=0;
		return;
	}
	if(commandbuffer[0]==2)			//Set motor 2 position to High and Low value 0 to 1023
	{
		target2=(commandbuffer[1]*256)+commandbuffer[2];
		disable=0;
		return;
	}

	if(commandbuffer[0]==200)		//Send both analogue feedback raw values
	{
		SendAnalogueFeedback(currentanalogue1, currentanalogue2);
		return;
	}
	if(commandbuffer[0]==201)		//Send PID count
	{
		SendPidCount();
		return;
	}
	if(commandbuffer[0]==202)		//Send Firmware Version
	{
		SendFirmwareVersion();
		return;
	}
	if(commandbuffer[0]==203)		//Write EEPROM
	{
		EEPROM.write(commandbuffer[1],uint8_t(commandbuffer[2]));
		return;
	}
	if(commandbuffer[0]==204)		//Read EEPROM
	{
		EEPromToSerial(commandbuffer[1]);
		return;
	}
	if(commandbuffer[0]==205)		//Clear EEPROM
	{
		ClearEEProm();
		return;
	}
	if(commandbuffer[0]==206)		//Reread the whole EEPRom and store settings into fitting variables
	{
		ReadEEProm();
		return;
	}
	if(commandbuffer[0]==207 || commandbuffer[0]==208)		//Disable power on both motor
	{
		/*analogWrite(PWMPinM1,	  0);
		UnsetMotor1Inp1();
		UnsetMotor1Inp2();
		analogWrite(PWMPinM2,	  0);
		UnsetMotor2Inp1();
		UnsetMotor2Inp2();*/
		disable=1;
		return;
	}
	if(commandbuffer[0]==209 || commandbuffer[0]==210)		//Enable power on both motor
	{
		/*analogWrite(PWMPinM1,	  128);
		UnsetMotor1Inp1();
		UnsetMotor1Inp2();
		analogWrite(PWMPinM2,	  128);
		UnsetMotor2Inp1();
		UnsetMotor2Inp2();*/
		disable=0;
		return;
	}
	if(commandbuffer[0]==211)		//Send all debug values
	{
		SendDebugValues();
		return;
	}
}

void FeedbackPotWorker()
{
  int currentPosition1, currentPosition2;
  currentanalogue1 = stepper[0]->getCurrentPosition();
  currentanalogue2 = stepper[1]->getCurrentPosition();
	//Notice: Minimum and maximum scaling calculation is done in the PC plugin with faster float support
}

bool CheckChecksum() //Atmel chips have a comport error rate of 2%, so we need here a checksum
{
	byte checksum=0;
	for(int z=0; z < 3; z++)
	{
		byte val=commandbuffer[z];
		checksum ^= val;
	}
	if(checksum==commandbuffer[3]){return true;}
	return false;
}

void SerialWorker()
{
	while(Serial.available()) 
	{
		if(buffercount==-1)
		{
			buffer = Serial.read();
			if(buffer != 'X'){buffercount=-1;}else{buffercount=0;}
		}
		else
		{
			buffer = Serial.read();
			commandbuffer[buffercount]=buffer;
			buffercount++;
			if(buffercount > 3)
			{
				if(CheckChecksum()==true){ParseCommand();}else{errorcount++;}
				buffercount=-1;
			}
		}
	}
}

void CalculateVirtualTarget()
{
		virtualtarget1=target1;
    virtualtarget2=target2;
}

void CalculateMotorDirection()
{
	if(virtualtarget1 > (currentanalogue1 + FeedbackPotDeadZone1) || virtualtarget1 < (currentanalogue1 - FeedbackPotDeadZone1))
	{
		if (OutputM1 >= 0)  
		{                                    
			motordirection1=1;				// drive motor 1 forward
      //stepper[0]->runForward();
		}  
		else 
		{                                              
			motordirection1=2;				// drive motor 1 backward
			OutputM1 = abs(OutputM1);
      //stepper[0]->runBackward();
		}
	}
	else
	{
		motordirection1=0;
    //stepper[0]->stopMove();
	}

	if(virtualtarget2 > (currentanalogue2 + FeedbackPotDeadZone2) || virtualtarget2 < (currentanalogue2 - FeedbackPotDeadZone2))
	{
		if (OutputM2 >= 0)  
		{                                    
			motordirection2=1;				// drive motor 2 forward
      //stepper[1]->runForward();
		}  
		else 
		{                                              
			motordirection2=2;				// drive motor 2 backward
			OutputM2 = abs(OutputM2);
      //stepper[1]->runBackward();
		}
	}
	else
	{
		motordirection2=0;
    //stepper[1]->stopMove();
	}

  OutputM1=constrain(OutputM1,300,MotorMaxSpeed);
  OutputM2=constrain(OutputM2,300,MotorMaxSpeed);
  debuginteger=OutputM1;
}

int updateMotor1Pid(int targetPosition, int currentPosition)   
{
	float error = (float)targetPosition - (float)currentPosition; 
	float pTerm_motor_R = proportional1 * error;
	integrated_motor_1_error += error;                                       
	float iTerm_motor_R = integral1 * constrain(integrated_motor_1_error, -GUARD_MOTOR_1_GAIN, GUARD_MOTOR_1_GAIN);
	float dTerm_motor_R = derivative1 * (error - last_motor_1_error);                            
	last_motor_1_error = error;
	return constrain(K_motor_1*(pTerm_motor_R + iTerm_motor_R + dTerm_motor_R), -MotorMaxSpeed, MotorMaxSpeed);
}

int updateMotor2Pid(int targetPosition, int currentPosition)   
{
	float error = (float)targetPosition - (float)currentPosition; 
	float pTerm_motor_L = proportional2 * error;
	integrated_motor_2_error += error;                                       
	float iTerm_motor_L = integral2 * constrain(integrated_motor_2_error, -GUARD_MOTOR_2_GAIN, GUARD_MOTOR_2_GAIN);
	float dTerm_motor_L = derivative2 * (error - last_motor_2_error);                            
	last_motor_2_error = error;

	return constrain(K_motor_2*(pTerm_motor_L + iTerm_motor_L + dTerm_motor_L), -MotorMaxSpeed, MotorMaxSpeed);
}

void CalculatePID()
{
	OutputM1=updateMotor1Pid(virtualtarget1,currentanalogue1);
	OutputM2=updateMotor2Pid(virtualtarget2,currentanalogue2);
}

void UpdateSteppers()
{

  stepper[0]->setSpeed(SECONDS_TO_MICROS/OutputM1);
  //stepper[0]->setSpeed(SECONDS_TO_MICROS/4000);
  stepper[1]->setSpeed(SECONDS_TO_MICROS/OutputM2);
  //stepper[0]->setSpeed(SECONDS_TO_MICROS/4000);
  stepper[0]->applySpeedAcceleration();
  stepper[1]->applySpeedAcceleration();
  
  if(motordirection1!=0)
  {
    if(motordirection1==1)
    {
      stepper[0]->runForward();
    }
    else
    {
      stepper[0]->runBackward();
    }
  }
  else
  {
    stepper[0]->stopMove();
  }

  if(motordirection2!=0)
  {
    if(motordirection2==1)
    {
      stepper[1]->runForward();
    }
    else
    {
      stepper[1]->runBackward();
    }
  }
  else
  {
    stepper[1]->stopMove();
  }

}

void loop()
{
	//Read all stored PID and Feedback settings
	ReadEEProm();
	//Program loop
	while (1==1) //Important hack: Use this own real time loop code without arduino framework delays
	{
    SerialWorker();
		FeedbackPotWorker();
		CalculateVirtualTarget();
		CalculatePID();
		CalculateMotorDirection();
		if(enableSpeedUpdate==true)
		{
      enableSpeedUpdate=false;
			UpdateSteppers();
      /*Serial2.print(OutputM1);
      Serial2.print(" ");
      Serial2.print(virtualtarget1);
      Serial2.print(" ");
      Serial2.println(currentanalogue1);*/
      if(Serial2.available()!=0)
      {
        char symbol=0;
        symbol=Serial2.read();
        if(symbol=='+'){
          stepper[MOTOR_ONE]->setCurrentPosition(currentanalogue1-100);
        }
        else if(symbol=='-'){
          stepper[MOTOR_ONE]->setCurrentPosition(currentanalogue1+100);
        }
      }
		}
		pidcount++;
	}
}
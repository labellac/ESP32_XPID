/*
	X-Sim PID
  This is a branch of the original program developed by Martin Wiedenbauer that replace the control of two motor H-Bridge with analogue feedback 
  with two open loop stepper motors. Target is an ESP32.

  Sentences to set at converter application USO settings:
  Datapacket sent at simulator start: Xa~100~~13~Xs~2~~13~XZ~5~~13~Xm~1~~13~XM~4~~13~XA~10~~13~XE~1~~13~XP~40~~13~XI~1~~13~XD~1~~13~XH~13~
  Datapacket with axis information: XS~a01~~a02~~13~
  Datapacket sent at simulator stop: XE~0~~13~
  
	Command input protocol for main serial
  'X''S'{16 bit left position value}{16 bit right position value}  Set Left and Right target position value
  'X''E'{bool}"        Enable left and right motor true=enabled, false=disabled
  'X''P'{value}"       Proportional value for left and right motor
  'X''I'{value}"       Integral value for left and right motor
  'X''D'{value}"       Derivative value for left and right motor
  'X''H'               Make Homing cycle
  'X''a'{value}        Set operation angle in degrees
  'X''s'{value}        Set steps per revolution (x100)
  'X''Z'{value}        Set steps deadzone
  'X''m'{value}        Set min step speed (steps/second)
  'X''M'{value}        Set mas step speed (x1000) (steps/second)
  'X''A'{value}        Set acceleraion (x1000) (steps/second^2)
  

  Command input/output for debug serial
  TODO!

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
#include "FastAccelStepper.h"   //Using version 0.18.3

//Defines
#define DEBUG_BUFFER_SIZE 256  //Debug buffer size in bytes
#define MAIN_BUFFER_SIZE  64  //Debug buffer size in bytes
#define TASK_DELAY_100MS  100 //Task delay in miliseconds
#define TASK_DELAY_10MS   10  //Task delay in miliseconds

#define HOMING_TIMEOUT    50  //Homing timeout in hundred of miliseconds
#define HOMING_SPEED      10  //Homing speed in percentage of max speed
#define HOMING_OFFSET     10  //Homing offset in percentage of max step value

#define REVOLUTION_ANGLE     360    //Complete revolution angle in degrees
#define OPERATION_ANGLE      100     //Desired operation angle in degrees
#define REDUCTION_RATIO      20     //GearBox reduction ratio
#define STEPS_PER_REVOLUTION 200    //Motor steps per revolution
#define SECONDS_TO_MICROS    (long)1000000  //Divide ratio for seconds to useconds
#define N_MOTORS          2    //Number of used motors

#define ONE 0
#define TWO 1
#define ON  1
#define OFF 0

#define GUARD_MOTOR_GAIN 100
#define DEBOUNCE_TIME   10    //debounce time in hundred of ms

//Pin defines
#define STEP_L_PIN  21
#define DIR_L_PIN  19
#define EN_L_PIN  18
#define SW_L_PIN  4
#define STEP_R_PIN  32
#define DIR_R_PIN  25
#define EN_R_PIN  26
#define SW_R_PIN  27
#define EMERGENCY_STOP_PIN 14
#define MAIN_RELAY_PIN  13
#define DEBUG_RX_PIN 16
#define DEBUG_TX_PIN 17

//Global variables

  //Main serial variables
int mainRBufferCount=-1;
char mainRBuffer[MAIN_BUFFER_SIZE]={0};
int mainWBufferCount=0;
char mainWBuffer[MAIN_BUFFER_SIZE]={0};
  //Debug serial variables
int debugRBufferCount=-1;
char debugRBuffer[DEBUG_BUFFER_SIZE]={0};
int debugWBufferCount=0;
char debugWBuffer[DEBUG_BUFFER_SIZE]={0};
uint8_t debugCount=0;

  //Tasks
  //Steppers
long motorMaxSpeed=4000;      //default motor max speed in steps per second
int motorMinSpeed=1;
long motorAcceleration=10000; //default motor max speed in steps per second^2
float angleRatio = (float)OPERATION_ANGLE/(float)REVOLUTION_ANGLE;
long stepsPerRevolution = STEPS_PER_REVOLUTION*REDUCTION_RATIO;
int maxStepValue = (int)((float)stepsPerRevolution * angleRatio);
  //Homing cycle
bool homingCycle=false;
int homingSpeed=(motorMaxSpeed*HOMING_SPEED)/100;
int homingOffset=(maxStepValue*HOMING_OFFSET)/100; //motor homming offset

int disable=1;

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
      on_delay_us : 0,
      off_delay_ms : 1000
    },
    {
      step : STEP_R_PIN,
      enable_low_active : EN_R_PIN,
      enable_high_active : PIN_UNDEFINED,
      stop_switch_low_active : SW_R_PIN,
      stop_switch_high_active: PIN_UNDEFINED,
      direction : DIR_R_PIN,
      direction_high_count_up : false,
      auto_enable : true,
      on_delay_us : 0,
      off_delay_ms : 1000
    }
};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper[N_MOTORS];

  //PID
long targetPos[N_MOTORS]={0};
long currentPos[N_MOTORS]={0};
long motorSpeed[N_MOTORS]={0};
int motorDirection[N_MOTORS]={0};
int oldMotorDirection[N_MOTORS]={0};
float K=1;
float Kp=4.2;
float Ki=0.1;
float Kd=0.1;
long integralError[N_MOTORS]={0};
long derivativeError[N_MOTORS]={0};
int deadZone=5;
int pidCounter=0;
int pidCount=0;

//debounce struct definition
struct debounce_s{
  bool enabled;
  uint8_t debounce_cnt;
};

struct debounce_s debounce[3]={{false,0},{false,0},{false,0}};

//strcuct emergency stop switch
struct emergency_stop_switch_s{
  bool currentState;
  bool oldState;
  uint8_t pin; 
};

struct emergency_stop_switch_s emergencyStopSwitch={currentState:0,oldState:0,pin:EMERGENCY_STOP_PIN};

//Function prototypes
void UpdateSpeedAndDirection(void);
void CalculateMotorDirection(void);
long ComputePID(long targetPosition, long currentPosition, int nmotor);
void InitPID(void);
void ParseCommand(char * rcvdBuffer);
void InitSteppers(void);
void InitIO(void);
void EnableSteppers(void);
void DisableSteppers(void);
void SerialTask(void);
void SetMainRelayState(uint8_t state);
void SendDebug(void);

//interrupts
//Interrupt handler for motor one stop switch
void IRAM_ATTR stopSwitchOneIsr(){
  uint8_t temp=0;

  //Check for switch debounce time
  if(debounce[ONE].enabled==false)
  {
    temp=digitalRead(stepper_config[ONE].stop_switch_low_active);
    debounce[ONE].enabled=true;
    if(temp==0)
    {
      //Stop motor and set initial position
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Stop switch 1 interrupt! \r\n");
      //disable interrupt during homing cycle to avoid undesired behavior
      if(homingCycle)
      {
        detachInterrupt(stepper_config[ONE].stop_switch_low_active);
      }
      stepper[ONE]->stopMove();
      stepper[ONE]->setCurrentPosition(-homingOffset);
    }
  }
}

//Interrupt handler for motor two stop switch
void IRAM_ATTR stopSwitchTwoIsr(){
  uint8_t temp=0;

  if(debounce[TWO].enabled==false)
  {
    temp=digitalRead(stepper_config[TWO].stop_switch_low_active);
    debounce[TWO].enabled=true;
    if(temp==0)
    {
      //Stop motor and set initial position
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Stop switch 2 interrupt! \r\n");
      //disable interrupt during homing cycle to avoid undesired behavior
      if(homingCycle)
      {
        detachInterrupt(stepper_config[TWO].stop_switch_low_active);
      }
      stepper[TWO]->stopMove();
      stepper[TWO]->setCurrentPosition(-homingOffset); 
    }
  }
}
//Tasks
void debugSerialTask (void * parameter)
{
  const TickType_t delay_100ms = TASK_DELAY_100MS / portTICK_PERIOD_MS;

  char buffer=0;
  //Initialize debug serial
  Serial2.begin(1000000,SERIAL_8N1,DEBUG_RX_PIN,DEBUG_TX_PIN);

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

void homingCycleTask (void * parameter)
{
  const TickType_t delay_100ms = TASK_DELAY_100MS / portTICK_PERIOD_MS;

  uint8_t timeCount=0;
  uint8_t machineState=0;

  //Make both motors run backwards until stop switch is hit
  stepper[ONE]->setSpeed(SECONDS_TO_MICROS/homingSpeed);
  stepper[TWO]->setSpeed(SECONDS_TO_MICROS/homingSpeed);
  stepper[ONE]->runBackward();
  stepper[TWO]->runBackward();

  while(true)
  {
    switch(machineState)
    {
      case 0:
        //Check for motor stop condition that happens at stop switch interrupt
        if((stepper[ONE]->isRunning())||(stepper[TWO]->isRunning()))
        {
          if(timeCount>HOMING_TIMEOUT)
          {
            //If timeout is met stop both motors and fix initial position
            stepper[ONE]->forceStopAndNewPosition(0);
            stepper[TWO]->forceStopAndNewPosition(0);
            //then exit homing cycle
            machineState=6;
            debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Homing cycle failed!\r\n");
            break;
          }
          timeCount++;
        }
        else
        {
          machineState=1;
          debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Homing cycle step 1!\r\n");
        }
        break;
        
      case 1:
        //Move to middle position
        debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Stpr 1 pos: %d\r\n",stepper[ONE]->getCurrentPosition());
        debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Stpr 2 pos: %d\r\n",stepper[TWO]->getCurrentPosition());
        stepper[ONE]->moveTo(maxStepValue/2);
        stepper[TWO]->moveTo(maxStepValue/2);
        debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Homing cycle step 2!\r\n");
        machineState=2;
        break;
      case 2:
      case 3:
      case 4:
        //pasive delay
        debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Homing cycle step 3!\r\n");
        machineState+=1;
        break;
        
      case 5:
        //wait untiel middle position is reached to exit homming cycle
        debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Homing cycle step 4!\r\n");
        if((!stepper[ONE]->isRunning())&&(!stepper[TWO]->isRunning()))
        {
          machineState+=1;
          debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Homing cycle succes!\r\n");
          //After homing cycle completes, attahc interrupt again
          attachInterrupt(stepper_config[ONE].stop_switch_low_active, stopSwitchOneIsr, CHANGE);
          attachInterrupt(stepper_config[TWO].stop_switch_low_active, stopSwitchTwoIsr, CHANGE);
        }
        debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Stpr 1 state: %d\r\n",stepper[ONE]->isRunning());
        debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Stpr 1 state: %d\r\n",stepper[ONE]->isRunning());
        break;
      case 6:
        homingCycle=false;
        debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Homing cycle step 5!\r\n");
        vTaskDelete(NULL);
        
        break;
    }
    
    vTaskDelay(delay_100ms);
  }
  
}

void timedTask(void * parameter)
{
  const TickType_t delay_100ms = 100 / portTICK_PERIOD_MS;
  
  while(true)
  {
    //Debounce
    for(int i=0;i<3;i++){
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

    //check for emergency stop
    emergencyStopSwitch.currentState=digitalRead(emergencyStopSwitch.pin);
    //Active high emergency stop!
    if((emergencyStopSwitch.currentState==1)&&(emergencyStopSwitch.currentState!=emergencyStopSwitch.oldState)){
      if(debounce[2].enabled==false){
        debounce[2].enabled=true;
        DisableSteppers();
      }  
    }
    else if((emergencyStopSwitch.currentState==0)&&(emergencyStopSwitch.currentState!=emergencyStopSwitch.oldState)){
      ESP.restart();  
    }
    emergencyStopSwitch.oldState=emergencyStopSwitch.currentState;

    //Enable send debug data when motor are enabled
    if((disable==0)&&(homingCycle==0))
    {
      SendDebug();
    }
    
    pidCount=pidCounter;
    pidCounter=0;
    
    vTaskDelay(delay_100ms);
  }
}

//Set main relay state function
void SetMainRelayState(uint8_t state)
{
  digitalWrite(MAIN_RELAY_PIN,state);
}

void SendDebug(void)
{
  switch (debugCount)
  {
    case 0:
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Tareget Pos: %d\r\n",targetPos[ONE]);
      debugCount++;
      break;
    case 1:
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Current Pos: %d\r\n",currentPos[ONE]);
      debugCount++;
      break;
    case 2:
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Motor Speed: %d\r\n",motorSpeed[ONE]);
      debugCount++;
      break;
    case 3:
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"PID Count 100ms: %d\r\n",pidCount);
      debugCount++;
      break;
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
      debugCount++;
      break;
    default:
      debugCount=0;
      break;
  }
}

void CalculateMotorDirection(void)
{
  for (int i=0;i<N_MOTORS;i++)
  {
    if((targetPos[i] > (currentPos[i] + deadZone)) || (targetPos[i] < (currentPos[i] - deadZone)))
    {
      if(motorSpeed[i]>=0)
      {
        motorDirection[i]=1;
      }
      else
      {
        motorDirection[i]=2;
        motorSpeed[i]=abs(motorSpeed[i]);
      }
    }
    else
    {
      motorDirection[i]=0;
    }
    motorSpeed[i]=constrain(motorSpeed[i],motorMinSpeed,motorMaxSpeed);
  }
}

void UpdateSpeedAndDirection (void)
{
  for(int i=0;i<N_MOTORS;i++)
  {
    if(oldMotorDirection[i]!=motorDirection[i])
    {
      if(motorDirection[i]!=0)
      {
        if(motorDirection[i]==1)
        {
          stepper[i]->runForward();
        }
        else
        {
          stepper[i]->runBackward();
        }
      }
      else
      {
        stepper[i]->stopMove();
      }
    }
    
    stepper[i]->setSpeed(SECONDS_TO_MICROS/motorSpeed[i]);
    stepper[i]->applySpeedAcceleration();
    
    oldMotorDirection[i]=motorDirection[i];
  }
}

long ComputePID (long targetPosition, long currentPosition, int nmotor)   
{
  float error = (float)targetPosition - (float)currentPosition; 
  float pTerm_motor_R = Kp * error;
  integralError[nmotor] += error;                                       
  float iTerm_motor_R = Ki * constrain(integralError[nmotor], -GUARD_MOTOR_GAIN, GUARD_MOTOR_GAIN);
  float dTerm_motor_R = Kd * (error - derivativeError[nmotor]);                            
  derivativeError[nmotor] = error;
  return constrain(K*(pTerm_motor_R + iTerm_motor_R + dTerm_motor_R), -motorMaxSpeed, motorMaxSpeed);
}

void ParseCommand(char * rcvdBuffer)
{
  int temp=0;
  
  switch(rcvdBuffer[0])
  {
    //Command for setting motors target position
    case 'S':
      temp=rcvdBuffer[1]*256+rcvdBuffer[2];
      targetPos[ONE]=map(temp,0,65535,0,maxStepValue);
      temp=rcvdBuffer[3]*256+rcvdBuffer[4];
      targetPos[TWO]=map(temp,0,65535,0,maxStepValue);
      break;
    //Command for enable/disable motors
    case 'E':
      if(rcvdBuffer[1]=='1')
      {
        engine.init();
        EnableSteppers();
      }
      else
      {
        DisableSteppers();
        delay(1000);
        ESP.restart();
      }
      //debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Enable: %d \r\n",rcvdBuffer[1]);
      break;
    //Command for setting Proportional value
    case 'P':
      temp=rcvdBuffer[1];
      Kp=(float)temp/(float)10;
      //debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Proportional: %d \r\n",rcvdBuffer[1]);
      break;
    //Command for setting Integral value
    case 'I':
      temp=rcvdBuffer[1];
      Ki=(float)temp/(float)10;
      //debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Integral: %d \r\n",rcvdBuffer[1]);
      break;
    //Command for setting Derivative value
    case 'D':
      temp=rcvdBuffer[1];
      Kd=(float)temp/(float)10;
      //debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Derivative: %d \r\n",rcvdBuffer[1]);
      break;
    //Command for setting homing cycle
    case 'H':
      if(!homingCycle)
      {
        homingCycle=true;
        InitPID();
        xTaskCreate(homingCycleTask,"HomingCycleTask",1000,NULL,1,NULL);
        //debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Homing cycle \r\n");
      }
      break;
    case 'A':
      //Set acceleration
      motorAcceleration=rcvdBuffer[1]*1000;
      //debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Motor acceleration set to: %ld\r\n",motorAcceleration);
      stepper[ONE]->setAcceleration(motorAcceleration);
      stepper[TWO]->setAcceleration(motorAcceleration);
      break;
    case 'M':
      //Set max speed
      motorMaxSpeed=rcvdBuffer[1]*1000;
      homingSpeed=(motorMaxSpeed*HOMING_SPEED)/100;
      //debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Motor max speed set to: %ld\r\n",motorMaxSpeed);
      break;
    case 'm':
      //Set min speed
      motorMinSpeed=rcvdBuffer[1];
      //debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Motor min speed set to: %d\r\n",motorMinSpeed);
      break;
    case 'Z':
      //set deadzone
      deadZone=rcvdBuffer[1];
      //debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Dead zone set to: %d\r\n",deadZone);
      break;
    case 'a':
      //set angle ratio
      temp=rcvdBuffer[1];
      angleRatio = (float)temp/(float)REVOLUTION_ANGLE;
      //debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Operation angle set to: %d\r\n",temp);
      break;
    case 's':
      //set steps per revolution
      temp=rcvdBuffer[1]*100;
      stepsPerRevolution = temp*REDUCTION_RATIO;
      maxStepValue = (int)((float)stepsPerRevolution * angleRatio);
      homingOffset=(maxStepValue*HOMING_OFFSET)/100; //motor homming offset
      //debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Max step value: %ld\r\n",maxStepValue);
      break;
    default:
      debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Invalid command \r\n");
      break;
      
  }  
}

void SerialTask(void)
{
  char buffer=0;

  while(Serial.available()) 
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
        mainRBufferCount=-1;
        ParseCommand(mainRBuffer);
        break;
      }
    }
  }
}

void InitPID (void)
{
  for(int i=0;i<N_MOTORS;i++)
  {
    targetPos[i]=maxStepValue/2;
    motorSpeed[i]=0;
    motorDirection[i]=0;
    oldMotorDirection[i]=0;
    integralError[i]=0;
    derivativeError[i]=0; 
  }
}

void InitSteppers(void)
{
  //Init stepper engine
  //engine.init();
  if (led_pin != PIN_UNDEFINED) {
    engine.setDebugLed(led_pin);
  }

  //Initialize steppers configuration
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
        s->setAcceleration(motorAcceleration);
        s->setSpeed(SECONDS_TO_MICROS/motorMaxSpeed);
      }
    }
    stepper[i] = s;
  }
}

void InitIO(void)
{
  //Configure stop stiwch pins and interrupts
  pinMode(stepper_config[ONE].stop_switch_low_active,INPUT_PULLUP);  //Configure stop switch pin as input pullup
  pinMode(stepper_config[TWO].stop_switch_low_active,INPUT_PULLUP);

  attachInterrupt(stepper_config[ONE].stop_switch_low_active, stopSwitchOneIsr, CHANGE);
  attachInterrupt(stepper_config[TWO].stop_switch_low_active, stopSwitchTwoIsr, CHANGE);

  //Configure emergency stop input
  pinMode(EMERGENCY_STOP_PIN,INPUT_PULLUP);
  //Configure main relay pin output and set to disable
  pinMode(MAIN_RELAY_PIN,OUTPUT);
  digitalWrite(MAIN_RELAY_PIN,LOW);
}


void EnableSteppers(void)
{
  stepper[ONE]->setAutoEnable(true);
  stepper[TWO]->setAutoEnable(true);
  
  SetMainRelayState(ON);
  disable=0;
}

void DisableSteppers(void)
{
  for (int i=0;i<N_MOTORS;i++)
  {
    stepper[i]->stopMove();
    stepper[i]->setAutoEnable(false);
    stepper[i]->disableOutputs();
  }

  disable=1;
  SetMainRelayState(OFF);
}

void setup()
{
  debugWBufferCount+=sprintf(debugWBuffer+debugWBufferCount,"Program INIT!\r\n");
  //Initialize main serial
  Serial.begin(115200);
  //Create task for debug serial
  xTaskCreate(debugSerialTask,"DebugSerialTask", 10000,NULL,1,NULL);

  InitSteppers();
  InitIO();

  xTaskCreate(timedTask,"TimedTask",1000,NULL,1,NULL);
}

void loop()
{
  SerialTask();
  if((disable==0)&&(!homingCycle))
  {
    for(int i=0;i<N_MOTORS;i++)
    {
      currentPos[i]=stepper[i]->getCurrentPosition();
      motorSpeed[i]=ComputePID(targetPos[i],currentPos[i],i);
    }
    CalculateMotorDirection();
    UpdateSpeedAndDirection();
  }
  pidCounter++;
}

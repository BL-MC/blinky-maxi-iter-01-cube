const boolean CHATTY_CATHY  = true;
const boolean MQTT  = false;

#define VALVE_CYCLE_STATE_INIT 0
#define VALVE_NUM_CYCLES_INIT 5
#define VALVE_CYCLE_INTERVAL_INIT 30
#define BOUNCE_DELAY 50 

const char*   MQTT_SERVER   = "192.168.1.1";
const char*   MQTT_USERNAME = "bl-iter-01";
const char*   MQTT_PASSWORD = "andLetThereBeLite!";
const char*   BOX           = "bl-iter-01";
const char*   TRAY_TYPE     = "blinky-mqtt";
const char*   TRAY_NAME     = "transit-test-01";
const char*   HUB           = "cube";

#include <Controllino.h>  

union CubeData
{
  struct
  {
    int16_t state;
    int16_t watchdog;
    int16_t valveCycleState;
    int16_t valveNumCycles;
    int16_t valveCycleInterval;  //seconds
    int16_t valveState;
    int16_t valveCycleCount;
    int16_t switch0State;
    int16_t switch1State;
    int16_t time1to0;   //mS
    int16_t time0to1;   //mS
    int16_t newData;
  };
  byte buffer[16];
};
CubeData cubeData;
byte mac[] = { 0x42, 0x4C, 0x30, 0x30, 0x30, 0x31 };

#include "BlinkyEtherCube.h"

unsigned long lastPublishTime;
unsigned long lastValveCycleTime;
unsigned long publishInterval = 2000;
unsigned long valveCycleInterval = 0; //mS

struct PositionSwitch
{
  int currentState;
  int lastState;
  int inputPin;
  int hotPin;
  unsigned long lastDebounceTime;  
  unsigned long switchTime;  
  unsigned long debounceDelay;    
};
PositionSwitch posSwitch[2];

void setup()
{
  if (CHATTY_CATHY)
  {
    Serial.begin(9600);
    delay(10000);
    Serial.println("Starting setup");
  }

  // Optional setup to overide defaults
  if(MQTT)
  {
    BlinkyEtherCube.setChattyCathy(CHATTY_CATHY);
    BlinkyEtherCube.setMqttRetryMs(3000);
    BlinkyEtherCube.setBlMqttKeepAlive(8);
    BlinkyEtherCube.setBlMqttSocketTimeout(4);
 
  // Must be included
    BlinkyEtherCube.setMqttServer(mac, MQTT_SERVER, MQTT_USERNAME, MQTT_PASSWORD);
    BlinkyEtherCube.setMqttTray(BOX,TRAY_TYPE,TRAY_NAME, HUB);
    BlinkyEtherCube.init(&cubeData);
  }

  lastPublishTime = millis();
  cubeData.state = 1;
  cubeData.watchdog = 0;
  cubeData.valveCycleState = VALVE_CYCLE_STATE_INIT;
  cubeData.valveNumCycles = VALVE_NUM_CYCLES_INIT;
  cubeData.valveCycleCount = 0;
  cubeData.valveCycleInterval = VALVE_CYCLE_INTERVAL_INIT;
  valveCycleInterval = ((unsigned long) cubeData.valveCycleInterval) * 1000;
  cubeData.valveState = 0;
  cubeData.switch0State = 0;
  cubeData.switch1State = 0;
  cubeData.time1to0 = 0;   //mS
  cubeData.time0to1 = 0;   //mS
 
  posSwitch[0].inputPin = CONTROLLINO_A1;
  posSwitch[0].hotPin   = CONTROLLINO_D1;

  posSwitch[1].inputPin = CONTROLLINO_A2;
  posSwitch[1].hotPin   = CONTROLLINO_D2;

  for (int ii = 0; ii < 2; ++ii)
  {
    posSwitch[ii].debounceDelay = BOUNCE_DELAY;
    pinMode(posSwitch[ii].inputPin, INPUT);
    pinMode(posSwitch[ii].hotPin, OUTPUT);
    digitalWrite(posSwitch[ii].hotPin, 1);
    delay(posSwitch[ii].debounceDelay);
    posSwitch[ii].currentState = digitalRead(posSwitch[ii].inputPin);
    posSwitch[ii].lastState = posSwitch[ii].currentState;
    posSwitch[ii].lastDebounceTime = millis();
    posSwitch[ii].switchTime = posSwitch[ii].lastDebounceTime;
  }

  cubeData.newData = 0;

  pinMode(CONTROLLINO_D0, OUTPUT);    
  digitalWrite(CONTROLLINO_D0, cubeData.valveState);    
  lastValveCycleTime = lastPublishTime;

}

void loop()
{
  unsigned long nowTime = millis();
  checkSwitch(0);
  checkSwitch(1);
  valveCycle01(nowTime);

  if ((nowTime - lastPublishTime) > publishInterval)
  {
    lastPublishTime = nowTime;
    cubeData.watchdog = cubeData.watchdog + 1;
    if (cubeData.watchdog > 32760) cubeData.watchdog= 0 ;
    if(MQTT) BlinkyEtherCube.publishToServer();
    cubeData.newData = 0;
  }  
  if(MQTT) BlinkyEtherCube.loop();
}

void handleNewSettingFromServer(uint8_t address)
{
  switch(address)
  {
    case 0:
      break;
    case 1:
      break;
    case 2:
      switch(cubeData.valveCycleState)
      {
        case 0:
          break;
        case 1:
          cubeData.valveCycleCount = 0;
          break;
        default:
          break;
      }
      break;
    case 3:
      if (cubeData.valveNumCycles < 0) cubeData.valveNumCycles = 0;
      break;                
    case 4:
      if (cubeData.valveCycleInterval < 1) cubeData.valveNumCycles = 1;
      valveCycleInterval = ((unsigned long) cubeData.valveCycleInterval) * 1000;
      break;                
    case 5:
      digitalWrite(CONTROLLINO_D0, cubeData.valveState); 
      lastValveCycleTime = millis();  
      cubeData.newData = 1;
      break;                
    default:
      break;
  }
}

void valveCycle01(unsigned long nowTime)
{
  if (cubeData.valveCycleState != 1) return;
  if ((nowTime - lastValveCycleTime) < valveCycleInterval) return;
  if (cubeData.valveState == 0)
  {
    cubeData.valveState = 1;
  }
  else
  {
    cubeData.valveState = 0;
  }
  if (CHATTY_CATHY)
  {
    Serial.print("Valve state: ");
    Serial.print(cubeData.valveState);
    Serial.print("; cycle count: ");
    Serial.println(cubeData.valveCycleCount);
    Serial.println("");
  }
  digitalWrite(CONTROLLINO_D0, cubeData.valveState); 
  lastValveCycleTime = nowTime;  
  cubeData.valveCycleCount = cubeData.valveCycleCount + 1;
  cubeData.newData = 1;
  
  lastPublishTime = nowTime;
  cubeData.watchdog = cubeData.watchdog + 1;
  if (cubeData.watchdog > 32760) cubeData.watchdog= 0 ;
  if(MQTT)
  {
    BlinkyEtherCube.publishToServer();
    BlinkyEtherCube.loop();
  }
  cubeData.newData = 0;
  
// end cycling if we reach max cycle count
  if (cubeData.valveCycleCount >= cubeData.valveNumCycles) cubeData.valveCycleState = 0;
}


//modified  this routine from https://docs.arduino.cc/built-in-examples/digital/Debounce/
boolean readSwitch(PositionSwitch* pswitch)
{
  int switchReading = digitalRead(pswitch->inputPin);
  unsigned long now = millis();
  boolean newState = false;
  
  if (switchReading != pswitch->lastState) 
  {
    pswitch->lastDebounceTime = now;
  }

  if ((now - pswitch->lastDebounceTime) > pswitch->debounceDelay) 
  {
    if (switchReading != pswitch->currentState) 
    {
      pswitch->currentState = switchReading;
      pswitch->switchTime = now;
      newState = true;
    }
  }
  pswitch->lastState = switchReading;
  return newState;
}

void checkSwitch(int switchNo)
{
  int otherSwitch = 1;
  if (switchNo == 1) otherSwitch = 0;
  if (readSwitch(&posSwitch[switchNo]))
  {
    if (switchNo == 0)
    {
      cubeData.switch0State = posSwitch[switchNo].currentState;
    }
    else
    {
      cubeData.switch1State = posSwitch[switchNo].currentState;
    }
    if (CHATTY_CATHY)
    {
      Serial.print("Switch ");
      Serial.print(switchNo);
      Serial.print(" state: ");
      Serial.print(posSwitch[switchNo].currentState);
      Serial.print("; ");
    }
    if (posSwitch[switchNo].currentState == 1)
    {
      if (posSwitch[otherSwitch].currentState == 0)
      {
        if (posSwitch[switchNo].switchTime >  posSwitch[otherSwitch].switchTime)
        {
          unsigned long deltaT = posSwitch[switchNo].switchTime - posSwitch[otherSwitch].switchTime;
          if (deltaT> 32765) deltaT = 32765;
          if (switchNo == 0)
          {
            cubeData.time1to0 = (int16_t) deltaT;
            if (CHATTY_CATHY)
            {
              Serial.print("1 to 0  Transtion time: ");
              Serial.print(cubeData.time1to0);
              Serial.print("mS");
            }
          }
          else
          {
            cubeData.time0to1 = (int16_t) deltaT;
            if (CHATTY_CATHY)
            {
              Serial.print("0 to 1  Transtion time: ");
              Serial.print(cubeData.time0to1);
              Serial.print("mS");
            }
          }
        }
      }
    }
    if (CHATTY_CATHY) Serial.println("\n");
  }
  
}

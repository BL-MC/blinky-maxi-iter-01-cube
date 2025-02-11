const boolean CHATTY_CATHY  = true;
const boolean MQTT  = true;

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
    int16_t valveOpenedState;
    int16_t valveClosedState;
    int16_t timeToOpen;   //mS
    int16_t timeToClose;   //mS
    int16_t newData;
  };
  byte buffer[24];
};
CubeData cubeData;
byte mac[] = { 0x42, 0x4C, 0x30, 0x30, 0x30, 0x31 };

#include "BlinkyEtherCube.h"

unsigned long lastPublishTime;
unsigned long lastValveCycleTime;
unsigned long publishInterval = 2000;
unsigned long valveCycleInterval = 0; //mS
unsigned long startOpenTime = 0;
unsigned long startCloseTime = 0;
unsigned long nowTime = 0;

struct ValveSwitch
{
  int16_t currentState;
  int16_t lastState;
  int inputPin;
  int hotPin;
  unsigned long switchTime;
  unsigned long lastDebounceTime;  
};
ValveSwitch valveOpenedSwitch;
ValveSwitch valveClosedSwitch;

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
  if (CHATTY_CATHY) Serial.println("Init cubeData");

  lastPublishTime = millis();
  cubeData.state = 1;
  cubeData.watchdog = 0;
  cubeData.valveCycleState = VALVE_CYCLE_STATE_INIT;
  cubeData.valveNumCycles = VALVE_NUM_CYCLES_INIT;
  cubeData.valveCycleCount = 0;
  cubeData.valveCycleInterval = VALVE_CYCLE_INTERVAL_INIT;
  valveCycleInterval = ((unsigned long) cubeData.valveCycleInterval) * 1000;
  cubeData.valveState = 0;
  cubeData.valveOpenedState = 0;
  cubeData.valveClosedState = 0;
  cubeData.timeToOpen = 0;   //mS
  cubeData.timeToClose = 0;   //mS
 
  if (CHATTY_CATHY) Serial.println("Init valveOpenedSwitch");
  setupValveSwitch(&valveOpenedSwitch, CONTROLLINO_A1, CONTROLLINO_D1);
  if (CHATTY_CATHY) Serial.println("Init valveClosedSwitch");
  setupValveSwitch(&valveClosedSwitch, CONTROLLINO_A2, CONTROLLINO_D2);

  cubeData.newData = 0;

  pinMode(CONTROLLINO_D0, OUTPUT);    
  digitalWrite(CONTROLLINO_D0, cubeData.valveState);    
  lastValveCycleTime = lastPublishTime;

  if (cubeData.valveCycleState == 1)
  {
    if (CHATTY_CATHY) Serial.println("Start Cycling");
    cubeData.valveCycleCount = 0;
    startOpenTime = millis();
    cubeData.valveState = 1;
    if (CHATTY_CATHY) Serial.println("Open Valve");
    digitalWrite(CONTROLLINO_D0, cubeData.valveState); 
  }

}

void loop()
{
  nowTime = millis();
  boolean newValveOpenedSwitchState = readSwitch(&valveOpenedSwitch);
  if (newValveOpenedSwitchState)
  {
    if (CHATTY_CATHY) Serial.print("ValveOpenedSwitchState: ");
    if (CHATTY_CATHY) Serial.println(valveOpenedSwitch.currentState);
  }
  boolean newValveClosedSwitchState = readSwitch(&valveClosedSwitch);
  if (newValveClosedSwitchState)
  {
    if (CHATTY_CATHY) Serial.print("ValveClosedSwitchState: ");
    if (CHATTY_CATHY) Serial.println(valveClosedSwitch.currentState);
  }
  cubeData.valveOpenedState = valveOpenedSwitch.currentState;
  cubeData.valveClosedState = valveClosedSwitch.currentState;
  if (cubeData.valveCycleState == 1)
  {
    if (cubeData.valveState == 1)
    {
      if (newValveOpenedSwitchState && (valveOpenedSwitch.currentState == 1))
      {
        cubeData.timeToOpen = nowTime - startOpenTime;
        if (CHATTY_CATHY) Serial.print("Open switch detected. Time to open: ");
        if (CHATTY_CATHY) Serial.print(cubeData.timeToOpen);
        if (CHATTY_CATHY) Serial.println(" ms");
      }
      if ((nowTime - startOpenTime) >= valveCycleInterval)
      {
        startCloseTime = nowTime;
        cubeData.valveState = 0;
        if (CHATTY_CATHY) Serial.println("Close Valve");
        digitalWrite(CONTROLLINO_D0, cubeData.valveState); 
        cubeData.newData = 1;
        lastPublishTime = nowTime;
        cubeData.timeToClose = 0;
        if(MQTT)
        {
          if (CHATTY_CATHY) Serial.println("Publish cycle data");
          BlinkyEtherCube.publishToServer();
          BlinkyEtherCube.loop();
        }
        cubeData.newData = 0;
      }
    }
    else
    {
      if (newValveClosedSwitchState && (valveClosedSwitch.currentState == 1))
      {
        cubeData.timeToClose = nowTime - startCloseTime;
        if (CHATTY_CATHY) Serial.print("Close switch detected. Time to close: ");
        if (CHATTY_CATHY) Serial.print(cubeData.timeToClose);
        if (CHATTY_CATHY) Serial.println(" ms");
      }
      if ((nowTime - startCloseTime) >= valveCycleInterval)
      {
        startOpenTime = nowTime;
        if (CHATTY_CATHY) Serial.println("Open Valve");
        if (cubeData.valveCycleCount < (cubeData.valveNumCycles - 1) )
        {
          cubeData.valveState = 1;
          digitalWrite(CONTROLLINO_D0, cubeData.valveState); 
        }
        cubeData.newData = 1;
        lastPublishTime = nowTime;
        cubeData.watchdog = cubeData.watchdog + 1;
        if (cubeData.watchdog > 32760) cubeData.watchdog= 0 ;
        cubeData.timeToOpen = 0;
        if(MQTT)
        {
          if (CHATTY_CATHY) Serial.println("Publish cycle data");
          BlinkyEtherCube.publishToServer();
          BlinkyEtherCube.loop();
        }
        cubeData.newData = 0;
        cubeData.valveCycleCount = cubeData.valveCycleCount + 1;
        if (CHATTY_CATHY) Serial.print("valveCycleCount: ");
        if (CHATTY_CATHY) Serial.println(cubeData.valveCycleCount);
        if (cubeData.valveCycleCount >= cubeData.valveNumCycles)
        {
          if (CHATTY_CATHY) Serial.println("Stop Cycling");
          cubeData.valveCycleState = 0;
        }
      }
    }
  }
  else
  {
    if ((nowTime - lastPublishTime) > publishInterval)
    {
      if (CHATTY_CATHY) Serial.println("Publish idle data");
      cubeData.timeToOpen = 0;
      cubeData.timeToClose = 0;
      lastPublishTime = nowTime;
      cubeData.watchdog = cubeData.watchdog + 1;
      if (cubeData.watchdog > 32760) cubeData.watchdog= 0 ;
      if(MQTT) BlinkyEtherCube.publishToServer();
      cubeData.newData = 0;
    }  
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
          if (CHATTY_CATHY) Serial.println("Cycling interrupted");
          break;
        case 1:
          if (CHATTY_CATHY) Serial.println("Start Cycling");
          cubeData.valveCycleCount = 0;
          startOpenTime = nowTime;
          cubeData.valveState = 1;
          digitalWrite(CONTROLLINO_D0, cubeData.valveState); 
          break;
        default:
          break;
      }
      break;
    case 3:
      if (CHATTY_CATHY) Serial.println("Number of cycles changed");
      if (cubeData.valveNumCycles < 0) cubeData.valveNumCycles = 0;
      break;                
    case 4:
      if (CHATTY_CATHY) Serial.println("valveCycleInterval changed");
      if (cubeData.valveCycleInterval < 1) cubeData.valveNumCycles = 1;
      valveCycleInterval = ((unsigned long) cubeData.valveCycleInterval) * 1000;
      break;                
    case 5:
      if (CHATTY_CATHY) Serial.println("Acting on valve directly");
      digitalWrite(CONTROLLINO_D0, cubeData.valveState); 
      lastValveCycleTime = millis();  
      cubeData.newData = 1;
      break;                
    default:
      break;
  }
}



//modified  this routine from https://docs.arduino.cc/built-in-examples/digital/Debounce/
boolean readSwitch(ValveSwitch* pValveSwitch)
{
  int16_t switchReading = (int16_t) digitalRead(pValveSwitch->inputPin);
  boolean newState = false;
  
  if (switchReading != pValveSwitch->lastState) 
  {
    pValveSwitch->lastDebounceTime = nowTime;
  }

  if ((nowTime - pValveSwitch->lastDebounceTime) > BOUNCE_DELAY) 
  {
    if (switchReading != pValveSwitch->currentState) 
    {
      pValveSwitch->currentState = switchReading;
      pValveSwitch->switchTime = nowTime;
      newState = true;
    }
  }
  pValveSwitch->lastState = switchReading;
  return newState;
}
void setupValveSwitch(ValveSwitch* pValveSwitch, int inputPin, int hotPin)
{
    pValveSwitch->inputPin = inputPin;
    pValveSwitch->hotPin = hotPin;
    
    pinMode(pValveSwitch->inputPin, INPUT);
    pinMode(pValveSwitch->hotPin, OUTPUT);
    digitalWrite(pValveSwitch->hotPin, 1);
    delay(BOUNCE_DELAY);
    pValveSwitch->currentState = (int16_t) digitalRead(pValveSwitch->inputPin);
    pValveSwitch->lastState = pValveSwitch->currentState;
    pValveSwitch->lastDebounceTime = millis();
    pValveSwitch->switchTime = pValveSwitch->lastDebounceTime;
}

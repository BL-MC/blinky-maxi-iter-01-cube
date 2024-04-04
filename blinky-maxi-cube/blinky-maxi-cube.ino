const boolean CHATTY_CATHY  = true;
const char*   MQTT_SERVER   = "192.168.1.1";
const char*   MQTT_USERNAME = "bl-iter-01";
const char*   MQTT_PASSWORD = "andLetThereBeLite!";
const char*   BOX           = "bl-iter-01";
const char*   TRAY_TYPE     = "blinky-mqtt";
const char*   TRAY_NAME     = "maxi-03";
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
    int16_t valveCycleCount;
    int16_t valveState;
  };
  byte buffer[14];
};
CubeData cubeData;
byte mac[] = { 0x42, 0x4C, 0x30, 0x30, 0x30, 0x31 };

#include "BlinkyEtherCube.h"

unsigned long lastPublishTime;
unsigned long lastValveCycleTime;
unsigned long publishInterval = 2000;
unsigned long valveCycleInterval = 0; //mS

void setup()
{
  if (CHATTY_CATHY)
  {
    Serial.begin(9600);
    delay(10000);
  }

  // Optional setup to overide defaults
  BlinkyEtherCube.setChattyCathy(CHATTY_CATHY);
  BlinkyEtherCube.setMqttRetryMs(3000);
  BlinkyEtherCube.setBlMqttKeepAlive(8);
  BlinkyEtherCube.setBlMqttSocketTimeout(4);
 
  // Must be included
  BlinkyEtherCube.setMqttServer(mac, MQTT_SERVER, MQTT_USERNAME, MQTT_PASSWORD);
  BlinkyEtherCube.setMqttTray(BOX,TRAY_TYPE,TRAY_NAME, HUB);
  BlinkyEtherCube.init(&cubeData);

  lastPublishTime = millis();
  cubeData.state = 1;
  cubeData.watchdog = 0;
  cubeData.valveCycleState = 0;
  cubeData.valveNumCycles = 5;
  cubeData.valveCycleCount = 0;
  cubeData.valveCycleInterval = 30;
  valveCycleInterval = ((unsigned long) cubeData.valveCycleInterval) * 1000;
  cubeData.valveState = 0;

  pinMode(CONTROLLINO_D0, OUTPUT);    
  digitalWrite(CONTROLLINO_D0, cubeData.valveState);    
  lastValveCycleTime = lastPublishTime;

}

void loop()
{
  unsigned long nowTime = millis();
  valveCycle42(nowTime);
 
  if ((nowTime - lastPublishTime) > publishInterval)
  {
    lastPublishTime = nowTime;
    cubeData.watchdog = cubeData.watchdog + 1;
    if (cubeData.watchdog > 32760) cubeData.watchdog= 0 ;
    BlinkyEtherCube.publishToServer();
  }  
  BlinkyEtherCube.loop();
}

void valveCycle42(unsigned long nowTime)
{
  if (cubeData.valveCycleState != 42) return;
  if ((nowTime - lastValveCycleTime) < valveCycleInterval) return;
  if (cubeData.valveState == 0)
  {
    cubeData.valveState = 1;
  }
  else
  {
    cubeData.valveState = 0;
  }
  digitalWrite(CONTROLLINO_D0, cubeData.valveState); 
  lastValveCycleTime = nowTime;  
  cubeData.valveCycleCount = cubeData.valveCycleCount + 1;
// end cycling if we reach max cycle count
  if (cubeData.valveCycleCount >= cubeData.valveNumCycles) cubeData.valveCycleState = 0;
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
        case 42:
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
    case 6:
      digitalWrite(CONTROLLINO_D0, cubeData.valveState); 
      lastValveCycleTime = millis();  
      break;                
    default:
      break;
  }
}

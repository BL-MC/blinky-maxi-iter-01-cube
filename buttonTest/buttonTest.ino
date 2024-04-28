#include <Controllino.h> 
#define BOUNCE_DELAY 50 

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

int16_t time1to0 = 0;
int16_t time0to1 = 0;

void setup() 
{
  Serial.begin(9600);
  
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

void loop() 
{
  if (readSwitch(&posSwitch[0]))
  {
    if (posSwitch[0].currentState == 1)
    {
      if (posSwitch[1].currentState == 0)
      {
        if (posSwitch[0].switchTime >  posSwitch[1].switchTime)
        {
          unsigned long deltaT = posSwitch[0].switchTime - posSwitch[1].switchTime;
          if (deltaT> 32765) deltaT = 32765;
          time1to0 = (int16_t) deltaT;
          Serial.print("1 to 0  Transtion time: ");
          Serial.print(time1to0);
          Serial.println("mS");
          Serial.println("");
          
        }
      }
    }
  }
  if (readSwitch(&posSwitch[1]))
  {
    if (posSwitch[1].currentState == 1)
    {
      if (posSwitch[0].currentState == 0)
      {
        if (posSwitch[1].switchTime >  posSwitch[0].switchTime)
        {
          unsigned long deltaT = posSwitch[1].switchTime - posSwitch[0].switchTime;
          if (deltaT> 32765) deltaT = 32765;
          time0to1 = (int16_t) deltaT;
          Serial.print("0 to 1  Transtion time: ");
          Serial.print(time0to1);
          Serial.println("mS");
          Serial.println("");
          
        }
      }
    }
  }
  
}

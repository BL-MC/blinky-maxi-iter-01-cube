#include <Ethernet.h>
#include <SPI.h>
#include <PubSubClient.h>

#define BL_MQTT_KEEP_ALIVE    8
#define BL_MQTT_SOCKETTIMEOUT 3
#define MQTT_RETRY            3000

union SubscribeData
{
  struct
  {
      uint8_t command;
      uint8_t address;
      int16_t value;
  };
  byte buffer[4];
};

SubscribeData subscribeData;
static void   BlinkyEtherCubeCallback(char* topic, byte* payload, unsigned int length);
void          handleNewSettingFromServer(uint8_t address);

class BlinkyEtherCube
{
  private:
  
    String   g_mqttServer;
    String   g_mqttUsername;
    String   g_mqttPassword;
    String   g_mqttClientId;
    String   g_mqttPublishTopic;
    String   g_mqttSubscribeTopic;
    byte*    g_mac;

    EthernetClient g_ethernetClient;
    PubSubClient  g_mqttClient;

    boolean       g_init = true;
    unsigned long g_mqttRetry = MQTT_RETRY;
    unsigned long g_mqttLastTry = 0;
    unsigned long g_lastMsgTime = 0;
    boolean       g_publishNow = false;
    boolean       g_chattyCathy = false;
    SubscribeData g_subscribeData;
    CubeData*     g_pcubeData;
    unsigned int  g_cubeDataSize;
    uint16_t      g_blMqttKeepAlive = BL_MQTT_KEEP_ALIVE;
    uint16_t      g_blMqttSocketTimeout = BL_MQTT_SOCKETTIMEOUT;

    boolean       reconnect(); 

  public:
    BlinkyEtherCube();
    void          loop();
    void          init(CubeData* pcubeData);
    void          setChattyCathy(boolean chattyCathy);
    void          mqttCubeCallback(char* topic, byte* payload, unsigned int length);
    void          setMqttRetryMs(unsigned long mqttRetry){g_mqttRetry = mqttRetry;};
    void          setBlMqttKeepAlive(uint16_t blMqttKeepAlive){g_blMqttKeepAlive = blMqttKeepAlive;};
    void          setBlMqttSocketTimeout(uint16_t blMqttSocketTimeout){g_blMqttSocketTimeout = blMqttSocketTimeout;};
    static void   checkForSettings();
    void          publishToServer();
    void          setMqttTray(String box, String mqttTrayType, String mqttTrayName, String mqttHub);
    void          setMqttServer(byte* mac, String mqttServer, String mqttUsername, String mqttPassword);

};
void BlinkyEtherCube::setMqttServer(byte* mac, String mqttServer, String mqttUsername, String mqttPassword)
{
  g_mac = mac;
  g_mqttServer = mqttServer;
  g_mqttUsername = mqttUsername;
  g_mqttPassword = mqttPassword;
}
void BlinkyEtherCube::setMqttTray(String box, String mqttTrayType, String mqttTrayName, String mqttHub)
{
  g_mqttClientId = g_mqttUsername + "-" + mqttTrayType + "-" + mqttTrayName;
  g_mqttPublishTopic = box + "/" + mqttHub + "/" + mqttTrayType + "/" + mqttTrayName + "/reading";
  g_mqttSubscribeTopic = box + "/" + mqttHub + "/" + mqttTrayType + "/" + mqttTrayName + "/setting";
}

BlinkyEtherCube::BlinkyEtherCube()
{
  g_cubeDataSize = (unsigned int) sizeof(CubeData);
}
void BlinkyEtherCube::publishToServer()
{
  g_publishNow = true;
}
void BlinkyEtherCube::loop()
{
  if (!g_mqttClient.connected()) 
  {
    if (!reconnect()) return;
  }
  g_mqttClient.loop();
  if (g_publishNow)
  {
      g_publishNow = false;
      g_mqttClient.publish(g_mqttPublishTopic.c_str(), g_pcubeData->buffer, g_cubeDataSize);
      if (g_chattyCathy) Serial.print("Publishing MQTT ");
      if (g_chattyCathy) Serial.println(g_pcubeData->watchdog);
      g_lastMsgTime = millis();
  }
}
void BlinkyEtherCube::init(CubeData* pcubeData)
{
  g_pcubeData = pcubeData;
  g_init = true;
  g_mqttClient = PubSubClient(g_ethernetClient);
  Ethernet.begin(mac);

  g_mqttClient.setKeepAlive(g_blMqttKeepAlive);
  g_mqttClient.setSocketTimeout(g_blMqttSocketTimeout);

  if (g_chattyCathy) Serial.println("Starting Communications");
  g_mqttClient.setServer(g_mqttServer.c_str(), 1883);
  g_mqttClient.setCallback(BlinkyEtherCubeCallback);
  g_init = false;
  g_mqttLastTry = 0;
  delay(g_mqttRetry);
  reconnect();
}
boolean BlinkyEtherCube::reconnect() 
{
  unsigned long now = millis();
  boolean connected = g_mqttClient.connected();
  if (connected) return true;
  if ((now - g_mqttLastTry) < g_mqttRetry) return false;

  if (g_chattyCathy) Serial.print("Attempting MQTT connection using ID: ");
  if (g_chattyCathy) Serial.print(g_mqttClientId);
  if (g_chattyCathy) Serial.print("...");
  connected = g_mqttClient.connect(g_mqttClientId.c_str(),g_mqttUsername.c_str(), g_mqttPassword.c_str());
  g_mqttLastTry = now;
  if (connected) 
  {
    if (g_chattyCathy) Serial.println("...connected");
    g_mqttClient.subscribe(g_mqttSubscribeTopic.c_str());
    return true;
  } 
  int mqttState = g_mqttClient.state();
  if (g_chattyCathy) Serial.print(" failed, rc=");
  if (g_chattyCathy) Serial.print(mqttState);
  if (g_chattyCathy) Serial.print(": ");

  switch (mqttState) 
  {
    case -4:
      if (g_chattyCathy) Serial.println("MQTT_CONNECTION_TIMEOUT");
      break;
    case -3:
      if (g_chattyCathy) Serial.println("MQTT_CONNECTION_LOST");
      break;
    case -2:
      if (g_chattyCathy) Serial.println("MQTT_CONNECT_FAILED");
       break;
    case -1:
      if (g_chattyCathy) Serial.println("MQTT_DISCONNECTED");
      break;
    case 0:
      if (g_chattyCathy) Serial.println("MQTT_CONNECTED");
      break;
    case 1:
      if (g_chattyCathy) Serial.println("MQTT_CONNECT_BAD_PROTOCOL");
      break;
    case 2:
      if (g_chattyCathy) Serial.println("MQTT_CONNECT_BAD_CLIENT_ID");
      break;
    case 3:
      if (g_chattyCathy) Serial.println("MQTT_CONNECT_UNAVAILABLE");
      break;
    case 4:
      if (g_chattyCathy) Serial.println("MQTT_CONNECT_BAD_CREDENTIALS");
      break;
    case 5:
      if (g_chattyCathy) Serial.println("MQTT_CONNECT_UNAUTHORIZED");
      break;
    default:
      break;
  }
  return false;
}
void BlinkyEtherCube::setChattyCathy(boolean chattyCathy)
{
  g_chattyCathy = chattyCathy;

  return;
}
void BlinkyEtherCube::mqttCubeCallback(char* topic, byte* payload, unsigned int length)
{
  if (g_chattyCathy) Serial.print("Message arrived [");
  if (g_chattyCathy) Serial.print(topic);
  if (g_chattyCathy) Serial.print("] {command: ");
  for (int i = 0; i < length; i++) 
  {
    g_subscribeData.buffer[i] = payload[i];
  }
  if (g_chattyCathy) Serial.print(g_subscribeData.command);
  if (g_chattyCathy) Serial.print(", address: ");
  if (g_chattyCathy) Serial.print(g_subscribeData.address);
  if (g_chattyCathy) Serial.print(", value: ");
  if (g_chattyCathy) Serial.print(g_subscribeData.value);
  if (g_chattyCathy) Serial.println("}");
  if (g_subscribeData.command == 1)
  {
    cubeData.buffer[g_subscribeData.address * 2] = payload[2];
    cubeData.buffer[g_subscribeData.address * 2 + 1] = payload[3];
    handleNewSettingFromServer(g_subscribeData.address);
  }
  return;
}

BlinkyEtherCube BlinkyEtherCube;


void BlinkyEtherCubeCallback(char* topic, byte* payload, unsigned int length) 
{
  BlinkyEtherCube.mqttCubeCallback(topic, payload,  length);
}

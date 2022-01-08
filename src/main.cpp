#include <WiFi.h>
#include <Wire.h>    // I2C library
#include "ccs811.h"  // CCS811 library
#include <credentials.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

CCS811 ccs811(23); // nWAKE on 23
int LED_BUILTIN = 2;

String clientId = "KarliESP-CCS811-1";
String command = "";
String TOPIC = "s/us";
unsigned long lastMsg = 0;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;


void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFINAME, WIFIPASS);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
  command = "100,"+clientId+",c8y_MQTTdevice";
  mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    default:
      Serial.println("Unhandled wifi event just happened!");
    }
}

void onMqttConnect(bool sessionPresent) {
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void alarmSend(String message, bool finish) {
  Serial.println(message);
  if(finish) {
    command = "301,c8y_CriticalAlarm,"+message;
    mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
  }
  else {
    command = "304,c8y_WarningAlarm,"+message;
    mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
  }
    while(finish) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(2500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(2500);
    }
}

void setup() {
//Start serial and LED
  Serial.begin(115200);
  while(!Serial);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println();
  Serial.println();

    //Create MQTT timers
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  //Create WIFI state listener
  WiFi.onEvent(WiFiEvent);

  //Create MQTT state methods
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(URL, 1883);
  mqttClient.setCredentials(TENANT,IOTPASS);
  mqttClient.setClientId(clientId.c_str());

  //Start connection
  connectToWifi();

    //Sensor initiate
    Serial.println("");
    Serial.println("setup: Starting CCS811 basic demo");
    Serial.print("setup: ccs811 lib  version: "); Serial.println(CCS811_VERSION);
    
    // Enable I2C
    Wire.begin(); 
    
    // Enable CCS811
    ccs811.set_i2cdelay(50); // Needed for ESP8266 because it doesn't handle I2C clock stretch correctly
    bool ok= ccs811.begin();
    if( !ok ) alarmSend("Setup: CCS811 begin FAILED",true);
    
    // Print CCS811 versions
    Serial.print("setup: hardware    version: "); Serial.println(ccs811.hardware_version(),HEX);
    Serial.print("setup: bootloader  version: "); Serial.println(ccs811.bootloader_version(),HEX);
    Serial.print("setup: application version: "); Serial.println(ccs811.application_version(),HEX);
    
    // Start measuring
    ok= ccs811.start(CCS811_MODE_1SEC);
    if( !ok ) alarmSend("Setup: CCS811 start FAILED",true);
}


double getAvg(int* vals, int val) {
  for (int i = 3; i >= 0; i = i - 1) {
    vals[i+1] = vals[i];
  }
  vals[0] = val;
  int sum = 0;
  for (int i = 0; i < 5; i = i + 1) {
    sum += vals[i];
  }
  return sum/5;
}

int ecoVals[] = {400,400,400,400,400};
int etvocVals[] = {0,0,0,0,0};
void loop() {
  unsigned long now = millis();
  if (now - lastMsg > 5000) { // Wait 5 seconds between every request
    lastMsg = now;
    uint16_t eco2, etvoc, errstat, raw;
    ccs811.read(&eco2,&etvoc,&errstat,&raw); 
    if( errstat==CCS811_ERRSTAT_OK ) { 
      int eco2Avg = getAvg(ecoVals,eco2);
      int etvocAvg = getAvg(etvocVals,etvoc);
      Serial.print("CCS811: ");
      Serial.print("eco2=");  Serial.print(eco2Avg);     Serial.print(" ppm  ");
      Serial.print("etvoc="); Serial.print(etvocAvg);    Serial.print(" ppb  ");
      command = "200,eco2Measurement,particles per million,"+String(eco2Avg)+",ppm";
      mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
      command = "200,etvocMeasurement,particles per billion,"+String(etvocAvg)+",ppb";
      mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
      Serial.println();
    } else if( errstat==CCS811_ERRSTAT_OK_NODATA ) {
      Serial.println("CCS811: waiting for (new) data");
    } else if( errstat & CCS811_ERRSTAT_I2CFAIL ) { 
      alarmSend("CCS811: I2C error",true);
    } else {
      Serial.print("CCS811: errstat="); Serial.print(errstat,HEX); 
      Serial.print("="); Serial.println( ccs811.errstat_str(errstat) );
      alarmSend("CCS811: sensor error",true);
    }
  }
}

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>    // I2C library
#include "ccs811.h"  // CCS811 library

CCS811 ccs811(23); // nWAKE on 23
int LED_BUILTIN = 2;

const char* ssid     = "MegaAeglane";
const char* password = "Sahman10";
const char* url = "testing.iot.cs.ut.ee";
const char* tenant = "testing/bachmann";
const char* passwordiot = "Kripits8%";
String clientId = "KarliESP-CCS811-1";
String command = "";


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(115200);
  while(!Serial);
    //=========================================
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");
    //=========================================
    //MQTT
    mqttClient.setServer(url, 1883);
    reconnect();
    //=========================================
    digitalWrite(LED_BUILTIN, LOW);

    //=========================================
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

void alarmSend(String message, bool finish) {
  Serial.println(message);
  if(finish) {
    command = "301,c8y_CriticalAlarm,"+message;
    mqttClient.publish("s/us", (char*) command.c_str());
  }
  else {
    command = "304,c8y_WarningAlarm,"+message;
    mqttClient.publish("s/us", (char*) command.c_str());
  }
  while(finish) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(2500);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(),tenant,passwordiot)) {
      Serial.println("Connected");
      String command = "100,"+clientId+",c8y_MQTTdevice";
      mqttClient.publish("s/us", (char*) command.c_str());
      digitalWrite(LED_BUILTIN, LOW);
      // Connected - do something useful - subscribe to topics, publish messages, etc.
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      // Wait 5 seconds before retrying
      Serial.println("Disconnecting wifi");
      WiFi.disconnect();
      Serial.println("Reconnecting wifi in 5 seconds");
      Serial.print("Connecting to ");
      Serial.println(ssid);
      delay(5000);
      WiFi.reconnect();
      int wifiCounter = 0;
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        wifiCounter +=1;
        if(wifiCounter > 30) {
          Serial.println("Wifi reconnect timed out. Restarting ESP.");
          ESP.restart();
        }
      }
    }
  }
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
  // put your main code here, to run repeatedly:

// put your main code here, to run repeatedly:

  if (!mqttClient.connected()) {
    reconnect();
  }

  mqttClient.loop();

  uint16_t eco2, etvoc, errstat, raw;
  ccs811.read(&eco2,&etvoc,&errstat,&raw); 
  if( errstat==CCS811_ERRSTAT_OK ) { 
    int eco2Avg = getAvg(ecoVals,eco2);
    int etvocAvg = getAvg(etvocVals,etvoc);
    Serial.print("CCS811: ");
    Serial.print("eco2=");  Serial.print(eco2Avg);     Serial.print(" ppm  ");
    Serial.print("etvoc="); Serial.print(etvocAvg);    Serial.print(" ppb  ");
    command = "200,eco2Measurement,particles per million,"+String(eco2Avg)+",ppm";
    mqttClient.publish("s/us", (char*) command.c_str());
    command = "200,etvocMeasurement,particles per billion,"+String(etvocAvg)+",ppb";
    mqttClient.publish("s/us", (char*) command.c_str());
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
 
  delay(10000);
}

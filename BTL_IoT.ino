// Import thu vien
#include <ESP8266WiFi.h>
#include <MQTT.h>
#include <EEPROM.h>
#include <DHT.h>

#define DEBUG

// Cai dat wifi
#define AP_SSID "Abc"
#define AP_PASSWORD "1234567800"

// Cau hinh ket noi easyiot cloud
#define EIOTCLOUD_USERNAME "quang21"
#define EIOTCLOUD_PASSWORD "quang21"
// Tao MQTT object
#define EIOT_CLOUD_ADDRESS "cloud.iot-playground.com"

#define DHTTYPE DHT11

// Cac chan cam vao ESP8266
#define PIN_PUMP D5      // Chan bom
#define PIN_HUM_SOIL A0  // Chan do am dat
#define PIN_DHT D6       // Chan DHT11

#define MAX_ANALOG_VAL 956
#define MIN_ANALOG_VAL 250

#define IRRIGATION_TIME 10         // Thoi gian bom (10s)
#define IRRIGATION_PAUSE_TIME 300  // Thoi gian dung bom (300s) - Trong che do auto

// Trang thai bom
typedef enum {
  s_idle = 0,              // Khong bom
  s_irrigation_start = 1,  // Bat bom
  s_irrigation = 2,        // Dang bom
  s_irrigation_stop = 3,   // Tat bom
} e_state;

#define CONFIG_START 0
#define CONFIG_VERSION "v01"

struct StoreStruct {
  char version[4];
  uint moduleId;
} storage = {
  CONFIG_VERSION,
  0,
};

#define PARAM_HUMIDITY_TRESHOLD "Sensor.Parameter1"
#define PARAM_MANUAL_AUTO_MODE "Sensor.Parameter2"
#define PARAM_PUMP_ON "Sensor.Parameter3"
#define PARAM_HUMIDITY_SOIL "Sensor.Parameter4"
#define PARAM_HUMIDITY "Sensor.Parameter5"
#define PARAM_TEMPERATURE "Sensor.Parameter6"

#define MS_IN_SEC 1000  // 1S

DHT dht(PIN_DHT, DHTTYPE);
MQTT myMqtt("", EIOT_CLOUD_ADDRESS, 1883);

int state;
bool stepOk = false;
int soilHumidityThreshold;
bool autoMode;
String valueStr("");
String topic("");
boolean result;
int lastAnalogReading;
bool autoModeOld;
int soilHumidityThresholdOld;
unsigned long startTime;
int soilHum;
int irrigatorCounter;
float hum;
float temperature;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PUMP, OUTPUT);
  dht.begin();

  state = s_idle;
  autoMode = true;
  stepOk = false;
  soilHumidityThresholdOld = -1;
  startTime = millis();
  soilHum = -1;
  hum = -1;
  temperature = -1;

  // Ket noi wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASSWORD);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(AP_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  };
  Serial.println("WiFi connected");
  Serial.println("Connecting to MQTT server");

  EEPROM.begin(512);
  loadConfig();

  //set client id
  // Tao client name dua tren dia chi MAC va  8 bits cuoi cua bo dem microsecond57
  String clientName;
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac);
  clientName += "-";
  clientName += String(micros() & 0xff, 16);
  myMqtt.setClientId((char*)clientName.c_str());
  Serial.print("MQTT client id:");
  Serial.println(clientName);

  // setup callbacks
  myMqtt.onConnected(myConnectedCb);
  myMqtt.onDisconnected(myDisconnectedCb);
  myMqtt.onPublished(myPublishedCb);
  myMqtt.onData(myDataCb);

  Serial.println("connect mqtt...");
  myMqtt.setUserPwd(EIOTCLOUD_USERNAME, EIOTCLOUD_PASSWORD);
  myMqtt.connect();

  delay(500);

  Serial.print("ModuleId: ");
  Serial.println(storage.moduleId);

  if (storage.moduleId != 0) {
    Serial.println("Module da ton tai");
  } else {
    //create module
    Serial.println("create module: /NewModule");
    storage.moduleId = myMqtt.NewModule();

    if (storage.moduleId == 0) {
      Serial.println("Module NOT created. Check module limit");
      while (1)
        delay(100);
    }

    // set module type
    Serial.println("Set module type");
    myMqtt.SetModuleType(storage.moduleId, "MT_GENERIC");

    // create Sensor.Parameter1 - humidity treshold value
    Serial.println("new parameter: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_TRESHOLD);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_HUMIDITY_TRESHOLD);
    // set Description
    Serial.println("set description: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_TRESHOLD);
    myMqtt.SetParameterDescription(storage.moduleId, PARAM_HUMIDITY_TRESHOLD, "HUMIDITY TRESHOLD");

    // create Sensor.Parameter2
    // Sensor.Parameter2 - manual/auto mode 0 - manual, 1 - auto mode
    Serial.println("new parameter: /" + String(storage.moduleId) + "/" + PARAM_MANUAL_AUTO_MODE);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_MANUAL_AUTO_MODE);
    // set Description
    Serial.println("set description: /" + String(storage.moduleId) + "/" + PARAM_MANUAL_AUTO_MODE);
    myMqtt.SetParameterDescription(storage.moduleId, PARAM_MANUAL_AUTO_MODE, "AUTO MODE");

    // create Sensor.Parameter3
    // Sensor.Parameter3 - pump on/ pump off
    Serial.println("new parameter: /" + String(storage.moduleId) + "/" + PARAM_PUMP_ON);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_PUMP_ON);
    // set Description
    Serial.println("set description: /" + String(storage.moduleId) + "/" + PARAM_PUMP_ON);
    myMqtt.SetParameterDescription(storage.moduleId, PARAM_PUMP_ON, "PUMP");

    // create Sensor.Parameter4
    // Sensor.Parameter4 - current soil humidity
    Serial.println("new parameter: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_SOIL);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_HUMIDITY_SOIL);
    // set Description
    Serial.println("set description: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_SOIL);
    myMqtt.SetParameterDescription(storage.moduleId, PARAM_HUMIDITY_SOIL, "SOIL MOIST");
    // set Unit
    Serial.println("set Unit: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_SOIL);
    myMqtt.SetParameterUnit(storage.moduleId, PARAM_HUMIDITY_SOIL, "%");
    // set dbLogging
    Serial.println("set Unit: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_SOIL);
    myMqtt.SetParameterDBLogging(storage.moduleId, PARAM_HUMIDITY_SOIL, true);

    // create Sensor.Parameter5
    // Sensor.Parameter5 - current humidity
    Serial.println("new parameter: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_HUMIDITY);
    // set Description
    Serial.println("set description: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY);
    myMqtt.SetParameterDescription(storage.moduleId, PARAM_HUMIDITY, "HUMIDITY");
    // set Unit
    Serial.println("set Unit: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY);
    myMqtt.SetParameterUnit(storage.moduleId, PARAM_HUMIDITY, "%");
    // set dbLogging
    Serial.println("set Unit: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY);
    myMqtt.SetParameterDBLogging(storage.moduleId, PARAM_HUMIDITY, true);

    // create Sensor.Parameter6
    // Sensor.Parameter6 - current temperature
    Serial.println("new parameter: /" + String(storage.moduleId) + "/" + PARAM_TEMPERATURE);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_TEMPERATURE);
    // set Description
    Serial.println("set description: /" + String(storage.moduleId) + "/" + PARAM_TEMPERATURE);
    myMqtt.SetParameterDescription(storage.moduleId, PARAM_TEMPERATURE, "TEMPERATURE");
    // set Unit
    Serial.println("set Unit: /" + String(storage.moduleId) + "/" + PARAM_TEMPERATURE);
    myMqtt.SetParameterUnit(storage.moduleId, PARAM_TEMPERATURE, "°C");
    // set dbLogging
    Serial.println("set Unit: /" + String(storage.moduleId) + "/" + PARAM_TEMPERATURE);
    myMqtt.SetParameterDBLogging(storage.moduleId, PARAM_TEMPERATURE, true);

    // save new module id
    saveConfig();
  }

  subscribe();
  lastAnalogReading = analogRead(PIN_HUM_SOIL);
  hum = dht.readHumidity();
  temperature = dht.readTemperature();
  autoModeOld = !autoMode;
}

void loop() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
#ifdef DEBUG
    Serial.print(".");
#endif
  }

  // post treshold changes
  if (soilHumidityThreshold != soilHumidityThresholdOld) {
    soilHumidityThresholdOld = soilHumidityThreshold;
    valueStr = String(soilHumidityThreshold);

    topic = "/" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_TRESHOLD;
    result = myMqtt.publish(topic, valueStr, 0, 1);

    Serial.print("Publish topic: ");
    Serial.print(topic);
    Serial.print(" value: ");
    Serial.println(valueStr);
  }

  if (IsTimeout()) {
    startTime = millis();
    // process every second
    int aireading = analogRead(PIN_HUM_SOIL);

    Serial.print("Analog value: ");
    Serial.print(aireading);
    Serial.print(" ");
    // filter s
    lastAnalogReading += (aireading - lastAnalogReading) / 10;
    Serial.print(lastAnalogReading);

    // calculate soil humidity in %
    int newSoilHum = map(lastAnalogReading, MIN_ANALOG_VAL, MAX_ANALOG_VAL, 100, 0);
    Serial.print(", Soil hum %:");
    Serial.println(newSoilHum);

    // limit to 0-100%
    if (newSoilHum < 0)
      newSoilHum = 0;

    if (newSoilHum > 100)
      newSoilHum = 100;

    // report soil humidity if changed
    if (soilHum != newSoilHum) {
      soilHum = newSoilHum;
      //esp.send(msgHum.set(soilHum));

      valueStr = String(soilHum);
      topic = "/" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_SOIL;
      result = myMqtt.publish(topic, valueStr, 0, 1);

      Serial.print("Publish topic: ");
      Serial.print(topic);
      Serial.print(" value: ");
      Serial.println(valueStr);
    }

    // Gui du lieu do am khong khi
    hum = dht.readHumidity();
    if (hum < 0.0) {
      hum = 0.0;
    } else if (hum > 100.0) {
      hum = 100.0;
    }
    valueStr = String(hum);
    topic = "/" + String(storage.moduleId) + "/" + PARAM_HUMIDITY;
    result = myMqtt.publish(topic, valueStr, 0, 1);
    Serial.print("Publish topic: ");
    Serial.print(topic);
    Serial.print(" value: ");
    Serial.println(valueStr);

    // Gui du lieu nhiet do
    temperature = dht.readTemperature();
    valueStr = String(temperature);
    topic = "/" + String(storage.moduleId) + "/" + PARAM_TEMPERATURE;
    result = myMqtt.publish(topic, valueStr, 0, 1);
    Serial.print("Publish topic: ");
    Serial.print(topic);
    Serial.print(" value: ");
    Serial.println(valueStr);

    // irrigator state machine
    switch (state) {
      case s_idle:
        if (irrigatorCounter <= IRRIGATION_PAUSE_TIME)
          irrigatorCounter++;

        if (irrigatorCounter >= IRRIGATION_PAUSE_TIME && autoMode) {
          if (soilHum <= soilHumidityThreshold && soilHum <= 95)
            state = s_irrigation_start;
        }
        break;
      case s_irrigation_start:
        if (soilHum <= 95) {
          irrigatorCounter = 0;
          digitalWrite(PIN_PUMP, HIGH);
          //esp.send(msgMotorPump.set((uint8_t)1));
          valueStr = String(1);
          topic = "/" + String(storage.moduleId) + "/" + PARAM_PUMP_ON;
          result = myMqtt.publish(topic, valueStr, 0, 1);

          Serial.print("Publish topic: ");
          Serial.print(topic);
          Serial.print(" value: ");
          Serial.println(valueStr);
        }
        state = s_irrigation;
        break;
      case s_irrigation:
        if (irrigatorCounter++ > IRRIGATION_TIME)
          state = s_irrigation_stop;
        break;
      case s_irrigation_stop:
        irrigatorCounter = 0;
        state = s_idle;
        //esp.send(msgMotorPump.set((uint8_t)0));
        valueStr = String(0);
        topic = "/" + String(storage.moduleId) + "/" + PARAM_PUMP_ON;
        result = myMqtt.publish(topic, valueStr, 0, 1);

        digitalWrite(PIN_PUMP, LOW);
        break;
    }
  }
}

void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] && EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] && EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
    for (unsigned int t = 0; t < sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
}

void saveConfig() {
  for (unsigned int t = 0; t < sizeof(storage); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&storage + t));

  EEPROM.commit();
}

String macToStr(const uint8_t* mac) {
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

boolean IsTimeout() {
  unsigned long now = millis();
  if (startTime <= now) {
    if ((unsigned long)(now - startTime) < MS_IN_SEC)
      return false;
  } else {
    if ((unsigned long)(startTime - now) < MS_IN_SEC)
      return false;
  }

  return true;
}

void subscribe() {
  if (storage.moduleId != 0) {
    // Sensor.Parameter1 - humidity treshold value
    myMqtt.subscribe("/" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_TRESHOLD);

    // Sensor.Parameter2 - manual/auto mode 0 - manual, 1 - auto mode
    myMqtt.subscribe("/" + String(storage.moduleId) + "/" + PARAM_MANUAL_AUTO_MODE);

    // Sensor.Parameter3 - pump on/ pump off
    myMqtt.subscribe("/" + String(storage.moduleId) + "/" + PARAM_PUMP_ON);
  }
}

void myConnectedCb() {
#ifdef DEBUG
  Serial.println("connected to MQTT server");
#endif
  subscribe();
}

void myDisconnectedCb() {
#ifdef DEBUG
  Serial.println("disconnected. try to reconnect...");
#endif
  delay(500);
  myMqtt.connect();
}

void myPublishedCb() {
#ifdef DEBUG
  Serial.println("published.");
#endif
}

void myDataCb(String& topic, String& data) {
#ifdef DEBUG
  Serial.print("Receive topic: ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(data);
#endif
  if (topic == String("/" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_TRESHOLD)) {
    soilHumidityThreshold = data.toInt();
    Serial.println("soilHumidityThreshold");
    Serial.println(data);
  } else if (topic == String("/" + String(storage.moduleId) + "/" + PARAM_MANUAL_AUTO_MODE)) {
    autoMode = (data == String("1"));
    Serial.println("Auto mode");
    Serial.println(data);
  } else if (topic == String("/" + String(storage.moduleId) + "/" + PARAM_PUMP_ON)) {
    //switchState = (data == String("1"))? true: false;
    if (data == String("1"))
      state = s_irrigation_start;
    else
      state = s_irrigation_stop;
    Serial.println("Pump");
    Serial.println(data);
  }
}
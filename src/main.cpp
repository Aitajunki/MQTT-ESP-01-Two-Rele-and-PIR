/*
   MQTT Binary Sensor - Motion (PIR) for Home-Assistant - NodeMCU (ESP8266)
   https://home-assistant.io/components/binary_sensor.mqtt/

   Libraries :
    - ESP8266 core for Arduino : https://github.com/esp8266/Arduino
    - PubSubClient : https://github.com/knolleary/pubsubclient

   Sources :
    - File > Examples > ES8266WiFi > WiFiClient
    - File > Examples > PubSubClient > mqtt_auth
    - File > Examples > PubSubClient > mqtt_esp8266
    - https://learn.adafruit.com/pir-passive-infrared-proximity-motion-sensor/using-a-pir

  Configuration (HA) :
    rele1:
      platform: mqtt
      name: 'Habitacion aitas (Rele1)'
      state_topic: 'habitacion_aitas/rele1/status'
      command_topic: 'habitacion_aitas/rele1/switch'
      optimistic: false

   Configuration (HA) :
    rele2:
      platform: mqtt
      name: 'Habitacion aitas (Rele2)'
      state_topic: 'habitacion_aitas/rele2/status'
      command_topic: 'habitacion_aitas/rele2/switch'
      optimistic: false

    Configuration (HA) :
      binary_sensor:
       platform: mqtt
       name: 'Habitacion aitas (PIR)'
       state_topic: 'habitacion_aitas/motion/status'
       sensor_class: motion

   Samuel M. - v1.1 - 08.2016
   If you like this example, please add a star! Thank you!
   https://github.com/mertenats/open-home-automation
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define MQTT_VERSION MQTT_VERSION_3_1_1

// Wifi: SSID and password
const PROGMEM char* WIFI_SSID = "[Redacted]";
const PROGMEM char* WIFI_PASSWORD = "[Redacted]";

// MQTT: ID, server IP, port, username and password
const PROGMEM char* MQTT_CLIENT_ID = "office_aitas";
const PROGMEM char* MQTT_SERVER_IP = "[Redacted]";
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
const PROGMEM char* MQTT_USER = "[Redacted]";
const PROGMEM char* MQTT_PASSWORD = "[Redacted]";

// MQTT: topic
const char* MQTT_RELE1_STATE_TOPIC = "office/rele1/status";
const char* MQTT_RELE1_COMMAND_TOPIC = "office/rele1/switch";
const char* MQTT_RELE2_STATE_TOPIC = "office/rele2/status";
const char* MQTT_RELE2_COMMAND_TOPIC = "office/rele2/switch";
const char* MQTT_MOTION_STATUS_TOPIC = "office/motion/status";

// default payload
const char* RELE1_ON = "ON";
const char* RELE1_OFF = "OFF";
const char* RELE2_ON = "ON";
const char* RELE2_OFF = "OFF";
const char* MOTION_ON = "OFF";
const char* MOTION_OFF = "ON";

// RELE1 : GPIO0
const PROGMEM uint8_t RELE1_PIN = 0;
boolean m_rele1_state = false; // rele1 is turned off by default

// RELE2 : GPIO2
const PROGMEM uint8_t RELE2_PIN = 2;
boolean m_rele2_state = false; // rele2 is turned off by default

// PIR : GPIO1
const PROGMEM uint8_t PIR_PIN = 3;
uint8_t m_pir_state = HIGH; // no motion detected
uint8_t m_pir_value = 0;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// function called to publish the state of the rele (on/off)
void publishReleState() {
  if (m_rele1_state) {
    client.publish(MQTT_RELE1_STATE_TOPIC, RELE1_ON, true);
  } else {
    client.publish(MQTT_RELE1_STATE_TOPIC, RELE1_OFF, true);
  }

  if (m_rele2_state) {
    client.publish(MQTT_RELE2_STATE_TOPIC, RELE2_ON, true);
  } else {
    client.publish(MQTT_RELE2_STATE_TOPIC, RELE2_OFF, true);
  }
}

// function called to publish the state of the pir sensor
void publishPirSensorState() {
  if (m_pir_state) {
    client.publish(MQTT_MOTION_STATUS_TOPIC, MOTION_OFF, true);
  } else {
    client.publish(MQTT_MOTION_STATUS_TOPIC, MOTION_ON, true);
  }
}

// function called to turn on/off the rele
void setReleState() {
  if (m_rele1_state) {
    digitalWrite(RELE1_PIN, HIGH);
    Serial.println("INFO: Turn rele1 on...");
  } else {
    digitalWrite(RELE1_PIN, LOW);
    Serial.println("INFO: Turn rele1 off...");
  }

  if (m_rele2_state) {
    digitalWrite(RELE2_PIN, HIGH);
    Serial.println("INFO: Turn rele2 on...");
  } else {
    digitalWrite(RELE2_PIN, LOW);
    Serial.println("INFO: Turn rele2 off...");
  }
}

// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }

  // handle message topic
  if (String(MQTT_RELE1_COMMAND_TOPIC).equals(p_topic)) {
    // test if the payload is equal to "ON" or "OFF"
    if (payload.equals(String(RELE1_ON))) {
      if (m_rele1_state != true) {
        m_rele1_state = true;
        setReleState();
        publishReleState();
      }
    } else if (payload.equals(String(RELE1_OFF))) {
      if (m_rele1_state != false) {
        m_rele1_state = false;
        setReleState();
        publishReleState();
      }
    }
  }

  if (String(MQTT_RELE2_COMMAND_TOPIC).equals(p_topic)) {
    // test if the payload is equal to "ON" or "OFF"
    if (payload.equals(String(RELE2_ON))) {
      if (m_rele2_state != true) {
        m_rele2_state = true;
        setReleState();
        publishReleState();
      }
    } else if (payload.equals(String(RELE2_OFF))) {
      if (m_rele2_state != false) {
        m_rele2_state = false;
        setReleState();
        publishReleState();
      }
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("INFO: Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("INFO: connected");
      // Once connected, publish an announcement...
      publishReleState();
      // ... and resubscribe
      client.subscribe(MQTT_RELE1_COMMAND_TOPIC);
      client.subscribe(MQTT_RELE2_COMMAND_TOPIC);
    } else {
      Serial.print("ERROR: failed, rc=");
      Serial.print(client.state());
      Serial.println(" DEBUG: try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  // init the serial
  Serial.begin(115200);

  // init the reles
  pinMode(RELE1_PIN, OUTPUT);
  pinMode(RELE2_PIN, OUTPUT);
  setReleState();

  // init the PIR
  pinMode(PIR_PIN,INPUT);

  // init the WiFi connection
  Serial.println();
  Serial.println();
  Serial.print("INFO: Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Fixed IP
  IPAddress IP(192,168,0,202);
  IPAddress GATEWAY(192,168,0,1);
  IPAddress SUBNET(255,255,255,0);
  WiFi.config(IP, GATEWAY, SUBNET);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("INFO: WiFi connected");
  Serial.println("INFO: IP address: ");
  Serial.println(WiFi.localIP());

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // read the PIR sensor
  m_pir_value = digitalRead(PIR_PIN);
  if (m_pir_value == HIGH) {
    if (m_pir_state == LOW) {
      // a motion is detected
      Serial.println("INFO: Motion detected");
      publishPirSensorState();
      m_pir_state = HIGH;
    }
  } else {
    if (m_pir_state == HIGH) {
      publishPirSensorState();
      Serial.println("INFO: Motion ended");
      m_pir_state = LOW;
    }
  }
}

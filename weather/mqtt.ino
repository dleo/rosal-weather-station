#include "weather.h"



/**
 * Connect to MQTT Broker
 */
void connectToMqtt() {
  // Setup for mqtt
  uint16_t size =1024;
  mqtt.setBufferSize(size);
  mqtt.setServer(AWS_IOT_ENDPOINT, 8883);
  mqtt.setCallback(callback);
  if (!mqtt.connected()) {
    reconnect();
  }
  mqtt.loop();
}

/**
 * Connect MQTT
 */
void reconnect() {
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    debug("Attempting MQTT re-connection...");
    // Attempt to connect
    if (mqtt.connect("sdk-nodejs-4ef74440-f98c-4451-af49-b8cc98038e51")) {
      debug("connected");
      // Subscribe, for example receive instructions
      //mqtt.subscribe("weather/#");
    } else {
      debug("failed, reason -> ");
      pubSubErr(mqtt.state());
      debug(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
}

/**
 * Publish mqtt errors
 */
void pubSubErr(int8_t MQTTErr) {
  if (MQTTErr == MQTT_CONNECTION_TIMEOUT)
    Serial.print("Connection timeout");
  else if (MQTTErr == MQTT_CONNECTION_LOST)
    Serial.print("Connection lost");
  else if (MQTTErr == MQTT_CONNECT_FAILED)
    Serial.print("Connect failed");
  else if (MQTTErr == MQTT_DISCONNECTED)
    Serial.print("Disconnected");
  else if (MQTTErr == MQTT_CONNECTED)
    Serial.print("Connected");
  else if (MQTTErr == MQTT_CONNECT_BAD_PROTOCOL)
    Serial.print("Connect bad protocol");
  else if (MQTTErr == MQTT_CONNECT_BAD_CLIENT_ID)
    Serial.print("Connect bad Client-ID");
  else if (MQTTErr == MQTT_CONNECT_UNAVAILABLE)
    Serial.print("Connect unavailable");
  else if (MQTTErr == MQTT_CONNECT_BAD_CREDENTIALS)
    Serial.print("Connect bad credentials");
  else if (MQTTErr == MQTT_CONNECT_UNAUTHORIZED)
    Serial.print("Connect unauthorized");
}

/**
 * Sent data to MQTT broker
 */
void sendData(struct sensorData *enviroment) {
  while (true) {
    if (!published)
    {
      if (WiFi.status() == WL_CONNECTED) {
        // Setup MQTT Broker
        connectAWS();
        connectToMqtt();
        getLocalTime(&timeinfo);
        debug("Attempting to publish MQTT...");
        const int capacity = JSON_OBJECT_SIZE(20);
        StaticJsonDocument<capacity> doc;
        doc["date-time"] = now();
        doc["altitude"] = enviroment->altitude;
        doc["humidity"] = int(enviroment->humidity);
        doc["pressure"] = int(enviroment->pressure);
        doc["uv"] = enviroment->UVIndex;
        doc["light"] = enviroment->lux;
        doc["wind-speed"] = enviroment->windSpeed;
        doc["wind-dir"] = enviroment->windDirection;
        doc["rain-hour"] = rainfall.hourlyRainfall[timeinfo.tm_hour-1] * 0.011;
        doc["rain-day"] = last24() * 0.011;
        doc["rain"] = rainfall.hourlyRainfall[timeinfo.tm_hour-1] * 0.011;
        doc["out-temperature"] = enviroment->outTemperature;
        doc["solar-radiation"] = enviroment->irradiation;
        doc["dew-point"] = enviroment->dewPoint;
        doc["heat-index"] = enviroment->heatIndex;
        doc["battery-level"] = enviroment->batteryVoltage;
        doc["in-temperature"] = enviroment->temperature;
        doc["eto"] = enviroment->eto;
        doc["rx-signal"] = enviroment->rxSignal;
        doc["moon-phase"] = enviroment->moonPhaseString;
        char jsonBuffer[512];
        
        serializeJson(doc, jsonBuffer); // print to client;
        // By default, PubSubClient limits the message size to 256 bytes (including header); see the documentation.
        if (!mqtt.publish("weather/summary", jsonBuffer))
          pubSubErr(mqtt.state());
        // Publish details
        serializeJson(doc["altitude"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/altitude", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["humidity"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/humidity", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["pressure"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/pressure", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["uv"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/uv", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["light"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/light", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["wind-speed"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/wind-speed", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["rain-hour"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/rain-hour", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["rain-day"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/rain-day", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["rain"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/rain", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["out-temperature"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/out-temperature", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["dew-point"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/dew-point", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["heat-index"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/heat-index", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["battery-level"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/battery-level", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["in-temperature"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/in-temperature", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["eto"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/eto", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["rx-signal"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/rx-signal", jsonBuffer))
          pubSubErr(mqtt.state());
        serializeJson(doc["moon-phase"], jsonBuffer); // print to client;
        if (!mqtt.publish("weather/moon-phase", jsonBuffer))
          pubSubErr(mqtt.state());
        debug("Finished publishMQTT...");
        published = true;
        sleep(updateWake());
      } else {
        debug("Can't publish because it doesn't connected to WiFi. Trying again");
        wifiOn();
      }
    } else {
      // We will check if has been more than 30 seconds up
      long currentMillis = millis();
      if ((currentMillis - initialMillis) >= (30 * msFactor)) {
        sleep(updateWake());
      }
    }
  } 
}

/**
 * Setup for connect to AWS
 */
void connectAWS()
{
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
}

/**
 * Callback for mqtt subscribe
 */
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

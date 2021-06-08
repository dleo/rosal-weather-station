/**
   Weather Station
   Original idea https://www.instructables.com/Solar-Powered-WiFi-Weather-Station-V30/
   Author dleo.lopez@gmail.com
*/

/**
   Libraries
*/
#include "secrets.h"
#include <BME280I2C.h>
#include "Adafruit_SI1145.h"
#include <BH1750.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "Wire.h"
#include <WiFi.h>
#include "esp_deep_sleep.h"
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <EnvironmentCalculations.h>

/**
   Pin definitions
*/
#define WIND_SPD_PIN 14
#define RAIN_PIN 25
#define WIND_DIR_PIN 35
#define VOLT_PIN 33
#define PV_PIN 32  // ACS712
#define TEMP_PIN 4 // DS18B20 hooked up to GPIO pin 4

/**
 * Objects
 */
WiFiClient client;
BME280I2C bme;
Adafruit_SI1145 uv = Adafruit_SI1145();
BH1750 lightMeter;
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);
WiFiClientSecure net;
PubSubClient mqtt(net);
TaskHandle_t TransmitTask;
TaskHandle_t ReadSensorTask;

/**
   Variables and constants
*/
// Variables used in reading temp,pressure and humidity (BME280)
float temperature, humidity, pressure;
float altitude = 0;
float dewPoint = 0;
float heatIndex = 0;
float outTemperature = 0;

// Variables used in reading UV Index (Si1145)
float UVindex;

// Variables used in reading Lux Level( BH1750 )
float lux;

// Variables used in calculating the windspeed
volatile unsigned long timeSinceLastTick = 0;
volatile unsigned long lastTick = 0;
float windSpeed;

// Variables used in calculating the wind direction
int vin;
String windDir = "";

// Variables and constants used in tracking rainfall
#define S_IN_DAY 86400
#define S_IN_HR 3600
#define NO_RAIN_SAMPLES 2000
#define SEALEVELPRESSURE_HPA (1013.25)
volatile long rainTickList[NO_RAIN_SAMPLES];
volatile int rainTickIndex = 0;
volatile int rainTicks = 0;
int rainLastDay = 0;
int rainLastHour = 0;
int rainLastHourStart = 0;
int rainLastDayStart = 0;
long secsClock = 0;

// Variables used in calculating the battery voltage
float batteryVolt;
float Vout = 0.00;
float Vin = 0.00;
float R1 = 27000.00; // resistance of R1 (27K) // You can also use 33K
float R2 = 100;
// Variables for solar radiation
float solarRadiation;
float R3 = 100;

//Variables for Solar Radiation
float mVperAmpValue = 185;          // If using ACS712 current module : for 5A module key in 185, for 20A module key in 100, for 30A module key in 66
float moduleMiddleVoltage = 1650;   // key in middle voltage value in mV. For 5V power supply key in 2500, for 3.3V power supply, key in 1650 mV
float moduleSupplyVoltage = 3300;   // supply voltage to current sensor module in mV, default 5000mV, may use 3300mV
float currentSampleRead = 0;        // to read the value of a sample
float currentLastSample = 0;        // to count time for each sample. Technically 1 milli second 1 sample is taken
float currentSampleSum = 0;         // accumulation of sample readings
float currentSampleCount = 0;       // to count number of sample
float currentMean;                  // to calculate the average value from all samples
float finalCurrent;                 // the final current reading without taking offset value
float finalCurrent2;                // the final current reading
float ShortCircuitCurrentSTC = 2.9; // Key in the Short Circuit Current (At STC condition) of your Solar Panel or Solar Cell. Value 9 showing 9.0A Isc Panel.
float irradiation = 0.00;           // This shows the irradiation level in W/m2.
/* 1.1 - Offset DC Current */
int OffsetRead = 0;          // To switch between functions for auto callibation purpose
float currentOffset = 0.00;  // to Offset deviation and accuracy. Offset any fake current when no current operates.
                             // the offset will automatically done when you press the <SELECT> button on the LCD display module.
                             // you may manually set offset here if you do not have LCD shield
float offsetLastSample = 0;  // to count time for each sample. Technically 1 milli second 1 sample is taken
float offsetSampleCount = 0; // to count number of sample.

/* 1.2 - Average Accumulate Irradiation */

float accumulateIrradiation = 0;              // Amount of accumulate irradiation
unsigned long startMillisIrradiation;         // start counting time for irradiation energy
unsigned long currentMillisIrradiation;       // current counting time for irradiation energy
const unsigned long periodIrradiation = 1000; // refresh every X seconds (in seconds) Default 1000 = 1 second
float FinalAccumulateIrradiationValue = 0;    // shows the final accumulate irradiation reading

/**
    Deep Sleep Time
 */
/**
//const int UpdateInterval = 1 * 60 * 1000000;  // e.g. 0.33 * 60 * 1000000; // Sleep time
//const int UpdateInterval = 15 * 60 * 1000000;  // e.g. 15 * 60 * 1000000; // // Example for a 15-Min update interval 15-mins x 60-secs * 10000
*/

/**
  WiFi
*/
char ssid[] = "Room";
char pass[] = "Rosal16232425";

/**
 * MQTT Broker setup
 */
IPAddress mqttServer(192, 168, 255, 121);
char *mqttCredentials[] = {"moss", "12345678"};

/**
   Setup function
*/
void setup()
{
  Serial.begin(115200);
  Serial.println("\nWeather station powered on.\n");
  delay(10000);
  Wire.begin();
  sensors.begin();
  //Wire.begin(22, 21);   // for BH1750
  while (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }
  uv.begin(0x60); // 0x60 is the address of the GY1145 module*/
  lightMeter.begin();
  wifiConnect();
  connectAWS();
  mqtt.setServer(AWS_IOT_ENDPOINT, 8883);
  mqtt.setCallback(callback);

  // Wind speed sensor setup. The windspeed is calculated according to the number
  //  of ticks per second. Timestamps are captured in the interrupt, and then converted
  //  into mph.
  pinMode(WIND_DIR_PIN, INPUT); // Wind dir sensor
  pinMode(WIND_SPD_PIN, INPUT); // Wind speed sensor
  attachInterrupt(digitalPinToInterrupt(WIND_SPD_PIN), windTick, FALLING);

  // Rain sensor setup. Rainfall is tracked by ticks per second, and timestamps of
  //  ticks are tracked so rainfall can be "aged" (i.e., rain per hour, per day, etc)
  pinMode(RAIN_PIN, INPUT); // Rain sensor
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainTick, FALLING);
  // Zero out the timestamp array.
  for (int i = 0; i < NO_RAIN_SAMPLES; i++)
    rainTickList[i] = 0;

  // ESP32 Deep SLeep Mode
  // esp_deep_sleep_enable_timer_wakeup(UpdateInterval);
  // Serial.println("Going to sleep now...");
  // esp_deep_sleep_start();
  startMillisIrradiation = millis(); /* Record initial starting time for daily irradiation */
  //We create a thread for send data
  xTaskCreatePinnedToCore(
      sendData,      /* Task function. */
      "sendData",    /* name of task. */
      10000,         /* Stack size of task */
      NULL,          /* parameter of the task */
      1,             /* priority of the task */
      &TransmitTask, /* Task handle to keep track of created task */
      1);            /* pin task to core 1 */
  // We create a thread for read data from sensors
  xTaskCreatePinnedToCore(
      readSensorsData,   /* Task function. */
      "readSensorsData", /* name of task. */
      10000,             /* Stack size of task */
      NULL,              /* parameter of the task */
      1,                 /* priority of the task */
      &ReadSensorTask,   /* Task handle to keep track of created task */
      1);                /* pin task to core 1 */
  delay(500);
}

/**
 * 
 */
void connectAWS()
{
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
}

/**
 * Loop function
 */
void loop()
{
  printData(); // Print all the sensors data on the serial monitor
  delay(30000);
}

/**
 * Connect to MQTT Broker
 */
void connectToMqtt()
{
  if (!mqtt.connected())
  {
    reconnect();
  }
  mqtt.loop();
}

/**
 * Sent data to MQTT broker
 */
void sendData(void *pvParameters)
{
  while (true)
  {
    connectToMqtt();
    Serial.print("Attempting to publishMQTT...");
    StaticJsonDocument<400> doc;
    doc["time"] = millis();
    doc["altitude"] = altitude;
    doc["humidity"] = int(humidity);
    doc["pressure"] = int(pressure);
    doc["uv"] = UVindex;
    doc["light"] = lux;
    doc["wind-speed"] = windSpeed;
    doc["rain-hour"] = float(rainLastHour) * 0.011;
    doc["rain-day"] = float(rainLastDay) * 0.011;
    doc["rain"] = float(rainTicks) * 0.011;
    doc["out-temperature"] = outTemperature;
    doc["solar-radiation"] = irradiation;
    doc["dew-point"] = dewPoint;
    doc["heat-index"] = heatIndex;
    doc["battery-level"] = batteryVolt;
    doc["in-temperature"] = temperature;
    char jsonBuffer[1024];
    //Publish all
    serializeJson(doc, jsonBuffer); // print to client;
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
    /**
    DynamicJsonDocument jsonBuffer(JSON_OBJECT_SIZE(3) + 100);
    JsonObject root = jsonBuffer.to<JsonObject>();
    JsonObject state = root.createNestedObject("state");
    JsonObject state_reported = state.createNestedObject("reported");
    state_reported["value"] = random(100);
    Serial.printf("Sending  [%s]: ", "topic_2");
    serializeJson(root, Serial);
    Serial.println();
    char shadow[measureJson(root) + 1];
    serializeJson(root, shadow, sizeof(shadow));
    if (!mqtt.publish("topic_2", shadow))
      pubSubErr(mqtt.state());
    delay(5000);
    **/
    delay(60000);
  }
}

/**
 * Publish mqtt errors
 */
void pubSubErr(int8_t MQTTErr)
{
  if (MQTTErr == MQTT_CONNECTION_TIMEOUT)
    Serial.print("Connection tiemout");
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
 * Connect MQTT
 */
void reconnect()
{
  // Loop until we're reconnected
  while (!mqtt.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect("sdk-nodejs-4ef74440-f98c-4451-af49-b8cc98038e51"))
    {
      Serial.println("connected");
      // Subscribe
      mqtt.subscribe("weather/#");
    }
    else
    {
      Serial.print("failed, reason -> ");
      pubSubErr(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/**
 * Wifi Network
 */
bool wifiConnect()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.setHostname(THINGNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}

/**
 * Read sensors
 */
void readSensorsData(void *pvParameters)
{
  while (true)
  {
    // Reading BME280 sensor
    bme.read(pressure, temperature, humidity, BME280::TempUnit_Celsius, BME280::PresUnit_hPa);
    EnvironmentCalculations::AltitudeUnit envAltUnit = EnvironmentCalculations::AltitudeUnit_Meters;
    EnvironmentCalculations::TempUnit envTempUnit = EnvironmentCalculations::TempUnit_Celsius;
    altitude = EnvironmentCalculations::Altitude(pressure, envAltUnit, SEALEVELPRESSURE_HPA, envTempUnit);
    dewPoint = EnvironmentCalculations::DewPoint(temperature, humidity, envTempUnit);
    heatIndex = EnvironmentCalculations::HeatIndex(temperature, humidity, envTempUnit);

    // Reading DS18B20 sensor
    sensors.requestTemperatures();
    outTemperature = sensors.getTempCByIndex(0);

    // Reading GY1145 UV sensor
    UVindex = uv.readUV();
    // the index is multiplied by 100 so to get the
    // integer index, divide by 100!
    UVindex /= 100.0;
    // Reading BH1750 sensor
    lux = lightMeter.readLightLevel();
    // Reading Battery Level in %
    float val = analogRead(VOLT_PIN); //reads the analog input
    //Vout = (val * 3.3 ) / 4095.0;           // formula for calculating voltage out
    batteryVolt = Vout * (R2 + R1) / R2; // formula for calculating voltage in
    // Read Weather Meters Datas ( Wind Speed, Rain Fall and Wind Direction )
    calculateIrradiation();
    static unsigned long outLoopTimer = 0;
    static unsigned long wundergroundUpdateTimer = 0;
    static unsigned long clockTimer = 0;
    static unsigned long tempMSClock = 0;
    /**
    // Create a seconds clock based on the millis() count. We use this
    //  to track rainfall by the second. We've done this because the millis()
    //  count overflows eventually, in a way that makes tracking time stamps
    //  very difficult.
    tempMSClock += millis() - clockTimer;
    clockTimer = millis();
    while (tempMSClock >= 1000)
    {
      secsClock++;
      tempMSClock -= 1000;
    }
  
    // This is a once-per-second timer that calculates and prints off various
    //  values from the sensors attached to the system.
    if (millis() - outLoopTimer >= 2000)
    {
      outLoopTimer = millis();
      // Windspeed calculation, in mph. timeSinceLastTick gets updated by an
      //  interrupt when ticks come in from the wind speed sensor.
      if (timeSinceLastTick != 0) windSpeed = 1000.0 / timeSinceLastTick;
  
      // Calculate the wind direction and display it as a string.
      windDirCalc();
      // Calculations for rain
      rainLastHour = 0;
      rainLastDay = 0;
      // If there are any captured rain sensor ticks...
      if (rainTicks > 0)
      {
        // Start at the end of the list. rainTickIndex will always be one greater
        //  than the number of captured samples.
        int i = rainTickIndex - 1;
  
        // Iterate over the list and count up the number of samples that have been
        //  captured with time stamps in the last hour.
        while ((rainTickList[i] >= secsClock - S_IN_HR) && rainTickList[i] != 0)
        {
          i--;
          if (i < 0) i = NO_RAIN_SAMPLES - 1;
          rainLastHour++;
        }
  
        // Repeat the process, this time over days.
        i = rainTickIndex - 1;
        while ((rainTickList[i] >= secsClock - S_IN_DAY) && rainTickList[i] != 0)
        {
          i--;
          if (i < 0) i = NO_RAIN_SAMPLES - 1;
          rainLastDay++;
        }
        rainLastDayStart = i;
      }
    }
    */
    delay(100); // We need define some delay for sensor like DS18B20
  }
}

/**
 * Keep track of when the last tick came in on the wind sensor.
 */
void windTick(void)
{
  timeSinceLastTick = millis() - lastTick;
  lastTick = millis();
}

/**
 * Capture timestamp of when the rain sensor got tripped.
 */
void rainTick(void)
{
  rainTickList[rainTickIndex++] = secsClock;
  if (rainTickIndex == NO_RAIN_SAMPLES)
    rainTickIndex = 0;
  rainTicks++;
}

/**
 * Reading wind direction
 */
void windDirCalc()
{

  vin = analogRead(WIND_DIR_PIN);

  if (vin < 150)
    windDir = "202.5";
  else if (vin < 300)
    windDir = "180";
  else if (vin < 400)
    windDir = "247.5";
  else if (vin < 600)
    windDir = "225";
  else if (vin < 900)
    windDir = "292.5";
  else if (vin < 1100)
    windDir = "270";
  else if (vin < 1500)
    windDir = "112.5";
  else if (vin < 1700)
    windDir = "135";
  else if (vin < 2250)
    windDir = "337.5";
  else if (vin < 2350)
    windDir = "315";
  else if (vin < 2700)
    windDir = "67.5";
  else if (vin < 3000)
    windDir = "90";
  else if (vin < 3200)
    windDir = "22.5";
  else if (vin < 3400)
    windDir = "45";
  else if (vin < 4000)
    windDir = "0";
  else
    windDir = "0";
}

/**
 * Print data
 */
void printData()
{
  Serial.print("Altitude: ");
  Serial.println(altitude);
  Serial.print("Air temperature [°C]: ");
  Serial.println(temperature);
  Serial.print("Humidity [%]: ");
  Serial.println(int(humidity));
  Serial.print("Barometric pressure [hPa]: ");
  Serial.println(pressure);
  Serial.print("UV: ");
  Serial.println(UVindex);
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  Serial.print("Windspeed: ");
  Serial.print(windSpeed * 2.4);
  Serial.println(" mph");
  Serial.print("Wind dir: ");
  Serial.print("  ");
  Serial.println(windDir);
  Serial.print("Rainfall last hour: ");
  Serial.println(float(rainLastHour) * 0.011, 3);
  Serial.print("Rainfall last day: ");
  Serial.println(float(rainLastDay) * 0.011, 3);
  Serial.print("Rainfall to date: ");
  Serial.println(float(rainTicks) * 0.011, 3);
  Serial.print("Battery Level: ");
  Serial.println(batteryVolt);
  Serial.print("Temperature in C: ");
  Serial.print(outTemperature);
  Serial.println();
  Serial.print("Solar Radiation W/M2: ");
  Serial.println(irradiation);
  Serial.print("Dew Point: ");
  Serial.println(dewPoint);
  Serial.print("Heat Index: ");
  Serial.println(heatIndex);
  Serial.print(FinalAccumulateIrradiationValue);
  Serial.println(" Wh/m2/day");
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/**
 * Calculate solar irradiation from ACS712
 */
void calculateIrradiation()
{
  /* 1- DC Current & Irradiation */

  if (millis() >= currentLastSample + 1) /* every 1 milli second taking 1 reading */
  {
    currentSampleRead = analogRead(PV_PIN) - ((moduleMiddleVoltage / moduleSupplyVoltage) * 1024); /* read the sample value */
    currentSampleSum = currentSampleSum + currentSampleRead;                                       /* accumulate value with older sample readings*/
    currentSampleCount = currentSampleCount + 1;                                                   /* to move on to the next following count */
    currentLastSample = millis();                                                                  /* to reset the time again so that next cycle can start again*/
  }

  if (currentSampleCount == 1000) /* after 1000 count or 1000 milli seconds (1 second), do the calculation and display value*/
  {
    currentMean = currentSampleSum / currentSampleCount;                           /* calculate average value of all sample readings taken*/
    finalCurrent = (((currentMean / 1024) * moduleSupplyVoltage) / mVperAmpValue); /* calculate the final current (without offset)*/
    finalCurrent2 = finalCurrent + currentOffset;                                  /* The final current */
    irradiation = (finalCurrent2 / ShortCircuitCurrentSTC * 1000);
    currentSampleSum = 0;   /* to reset accumulate sample values for the next cycle */
    currentSampleCount = 0; /* to reset number of sample for the next cycle */
  }
  /* 1.1 - Offset DC Current */

  if (OffsetRead == 1)
  {
    currentOffset = 0;                    /* set back currentOffset as default first*/
    if (millis() >= offsetLastSample + 1) /* offset 1 - to centralise analogRead waveform*/
    {
      offsetSampleCount = offsetSampleCount + 1;
      offsetLastSample = millis();
    }

    if (offsetSampleCount == 2500)   /* need to wait awhile as to get new value before offset take into calculation.  */
    {                                /* So this code is to delay 2.5 seconds after button pressed */
      currentOffset = -finalCurrent; /* to offset values */
      OffsetRead = 0;                /* until next offset button is pressed*/
      offsetSampleCount = 0;         /* to reset the time again so that next cycle can start again */
    }
  }

  /* 1.2 - Average Accumulate Irradiation */

  currentMillisIrradiation = millis(); /* Count the time for current */

  if (currentMillisIrradiation - startMillisIrradiation >= periodIrradiation)
  {
    accumulateIrradiation = irradiation / 3600 * (periodIrradiation / 1000); /* for smoothing calculation*/
    FinalAccumulateIrradiationValue = FinalAccumulateIrradiationValue + accumulateIrradiation;
    startMillisIrradiation = currentMillisIrradiation; /* Set the starting point again for next counting time */
  }
}

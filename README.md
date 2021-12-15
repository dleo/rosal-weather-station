# Weather Station with Arduino
Weather station is project focus on farms, the data recopiled by its sensor would help to irrigation, crop and taks on daily labors.

This project is based on [Solar Powered WiFi Weather](https://www.instructables.com/Solar-Powered-WiFi-Weather-Station-V30/).

We added features for calculations of evotranspiration (ET), solar radiation, out temperature, in temperature.

The goal is algo integrated with [Weewx](http://weewx.com/) for interacts with weather station.

Right now we can publish to MQTT broker as [AWS IOT](https://aws.amazon.com/iot/) and use node red for a UX on real time.

## Installation
Before uploading the code install the following libraries :

1. [ESP32](https://github.com/espressif/arduino-esp32)

2. [Blynk](https://github.com/blynkkk/blynk-library/releases)

3. [BME280](https://github.com/finitespace/BME280)

4. [Adafruit_SI1145_Library](https://github.com/adafruit/Adafruit_SI1145_Library)

5. [BH1750](https://github.com/claws/BH1750)

6. [One Wire](https://github.com/PaulStoffregen/OneWire)

7. [Dallas Temperature](https://github.com/milesburton/Arduino-Temperature-Control-Library)

### How to Install the Libraries?

You can read [this tutorial](https://learn.sparkfun.com/tutorials/installing-an-arduino-library/all) by Sparkfun to install the Arduino libraries.

## Contributing

Please follow the [PlatformIO](https://docs.platformio.org/en/latest/core/quickstart.html) guidelines for the project structure and compilation.

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
See the [LICENSE](https://github.com/dleo/rosal-weather-station/blob/main/LICENSE) file for license rights and limitations (Apache 2).


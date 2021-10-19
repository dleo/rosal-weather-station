#include "weather.h"

/**
 * Wifi Network
 */
bool wifiOn() {
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  delay(1);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
  }
  debug("WiFi connected. IP address: %s", WiFi.localIP());

  return true;
}

/**
 * Shutdown WiFi
 */
void wifiOff() {
  WiFi.mode(WIFI_OFF);
}

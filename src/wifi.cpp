#include "weather.h"
#include "secrets.h"
#include "WiFi.h"

/**
 * Wifi Network
 */
bool wifiOn() {
  // We start by connecting to a WiFi network
  debug("Connecting to %s", ssid);
  delay(1);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    if (millis()%5000 == 0) return false;
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

#include "weather.h"
#include "secrets.h"
#include "WiFi.h"


/**
 * Wifi Network
 */
bool wifiOn() {
  if (WiFi.status() == WL_CONNECTED) {
    debug("WiFi already is connected\n");
    return true;
  }
  // We start by connecting to a WiFi network
  debug("Connecting to %s", ssid);
  delay(1);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  int cMillis = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if ((millis() - cMillis) > (10 * msFactor)) { // Try 1000
      debug("Can't connected to WiFi");
      return false;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip;
    ip = WiFi.localIP();
    debug("WiFi connected. IP address: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  }

  return true;
}

/**
 * Shutdown WiFi
 */
void wifiOff() {
  debug("WiFi Off");
  WiFi.mode(WIFI_OFF);
}

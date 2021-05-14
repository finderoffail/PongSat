////////////////////////////////////////////////////////////////////////////////////////////
// Feature configuration section
//
////////////
#define FEATURE_SENSOR_MPU    1   // acceleration, angle, temperature
//#define FEATURE_SENSOR_LIS3DH 1   // acceleration, angle, temperature
//#define FEATURE_SENSOR_DHT22  1   // humidity, temperature
//#define FEATURE_SENSOR_DHT11  1   // humidity, temperature
#define FEATURE_SENSOR_BMP280 1   // altitude, pressure, temperature
//#define FEATURE_TNN_RADIO 1
//#define FEATURE_SENSOR_MIC 1

// platform specific features
#if defined(ARDUINO_ESP8266_GENERIC)
  #define FEATURE_DEEPSLEEP     1   // deep sleep for ESP8266
  #define FEATURE_SENSOR_BATTERY  1   // read battery voltage
  #define FEATURE_WIFI_RADIO    1   // WiFi for ESP8266 (and eventually ESP32)
#endif

#ifdef FEATURE_WIFI_RADIO
  #define FEATURE_STATIC_IP     1   // use static IP addresses, saves a DHCP request and hence battery
  #define FEATURE_FOTA          1   // firmware OTA update for ESP8266 (and eventually ESP32)
  #define FEATURE_MQTT          1   // MQTT broker pub/sub connection
#endif

////////////////////////////////////////////////////////////////////////////////////////////

#include "config_private.h"

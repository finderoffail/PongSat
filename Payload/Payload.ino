
#include "config.h"

#if defined( FEATURE_FOTA ) && !defined( PAYLOAD_VERSION )
  #error PAYLOAD_VERSION not defined by makefile
#endif
#if defined( FEATURE_FOTA ) && !(defined( FEATURE_WIFI_RADIO ) || defined( FEATURE_TNN_RADIO ))
  #error FOTA cannot work without radio!
#endif
#if defined( FEATURE_FOTA ) && defined( FEATURE_TNN_RADIO ) && !defined( FEATURE_WIFI_RADIO )
  #error FEATURE_FOTA over FEATURE_TNN_RADIO not implemented yet
#endif

#if defined(ARDUINO_TINYPICO)
 #undef FEATURE_WIFI_RADIO
 #undef FEATURE_MQTT
  #include <TinyPICO.h>
  #define LED_SETUP()
  #define LED_RED()  {tp.DotStar_SetBrightness(64); tp.DotStar_SetPixelColor( 255, 0, 0);}
  #define LED_GREEN() {tp.DotStar_SetBrightness(96); tp.DotStar_SetPixelColor( 0, 255, 0);}
  #define LED_BLUE() {tp.DotStar_SetBrightness(220); tp.DotStar_SetPixelColor( 0, 0, 255);}
  #define LED_OFF() {tp.DotStar_SetBrightness(0); tp.DotStar_SetPixelColor( 0, 0, 0);}
  TinyPICO tp = TinyPICO();
  #define FEATURE_PSRAM 1

#elif defined(ARDUINO_ESP8266_GENERIC)
  #include <ESP8266WiFi.h>
  #if defined( FEATURE_WIFI_RADIO ) && defined( FEATURE_FOTA )
    #include <ESP8266HTTPClient.h>
    #include <ESP8266httpUpdate.h>
  #endif
  #define GPIO0 0
  #define LED_SETUP() { pinMode(GPIO0, OUTPUT); LED_OFF(); }
  #define LED_RED() digitalWrite(GPIO0, HIGH);
  #define LED_GREEN() LED_OFF()
  #define LED_BLUE() LED_RED()
  #define LED_OFF() digitalWrite(GPIO0, LOW);

  #define I2C_SCL 13
  #define I2C_SDA 12

  ADC_MODE(ADC_VCC); // configure ADC for reading battery voltage

#elif defined(ARDUINO_AVR_NANO)
 #undef FEATURE_WIFI_RADIO
 #undef FEATURE_MQTT
  #define LED_SETUP() pinMode(LED_BUILTIN, OUTPUT);
  #define LED_RED() digitalWrite(LED_BUILTIN, HIGH);
  #define LED_GREEN() LED_OFF()
  #define LED_BLUE() LED_RED()
  #define LED_OFF() digitalWrite(LED_BUILTIN, LOW);
#else
  #error unknown board config
#endif


#if defined( FEATURE_SENSOR_MIC ) && !defined( FEATURE_PSRAM )
  #error FEATURE_PSRAM required for FEATURE_SENSOR_MIC
#endif

#ifdef FEATURE_WIFI_RADIO
////////////////////////////////////////////////////////////////////////////////////////////
// Network config parameters:
//
const char* ssid     = CONFIG_SSID;
const char* password  = CONFIG_PASSWORD;
//IPAddress ip( CONFIG_IP );
IPAddress gateway( CONFIG_GATEWAY );
IPAddress subnet( CONFIG_SUBNET );

#ifdef FEATURE_MQTT
const char* mqtt_server = CONFIG_MQTT_SERVER;
char mqtt_topic_status[20] = {0};
char mqtt_topic_in[20] = {0};
char mqtt_topic_out[20] = {0};
#endif
const char *Hostname;
////////////////////////////////////////////////////////////////////////////////////////////
#endif


#define CONNECTION_ATTEMPT_NUM_TRIES  4
#define CONNECTION_ATTEMPT_DELAY      250 // ms

#if defined( FEATURE_SENSOR_MIC )
  uint8_t *wavBuf = NULL;
  #define  WAV_BUF_SIZE  (2*1024*1024)
#endif

#ifdef FEATURE_MQTT
  #include <PubSubClient.h>
#endif

#include <Wire.h>
#if defined(ARDUINO_ESP8266_GENERIC)
TwoWire i2c = TwoWire();
#endif

#ifdef FEATURE_DEEPSLEEP

#define DEEPSLEEP_INTERVAL 5000 // milliseconds

uint32_t calculateCRC32( const uint8_t *data, size_t length ) {
  uint32_t crc = 0xffffffff;
  while( length-- ) {
    uint8_t c = *data++;
    for( uint32_t i = 0x80; i > 0; i >>= 1 ) {
      bool bit = crc & 0x80000000;
      if( c & i ) {
        bit = !bit;
      }

      crc <<= 1;
      if( bit ) {
        crc ^= 0x04c11db7;
      }
    }
  }

  return crc;
}

// The ESP8266 RTC memory is arranged into blocks of 4 bytes. The access methods read and write 4 bytes at a time,
// so the RTC data structure should be padded to a 4-byte multiple.
struct _rtcData {
  uint32_t crc32;

  // wiif settings
  uint8_t wifiValid;
  uint8_t channel;
  uint8_t bssid[6];

  // MPU settings
  #ifdef FEATURE_SENSOR_MPU
    uint32_t mpuValid;
    float mpuXcal;
    float mpuYcal;
    float mpuZcal;
  #endif

  // sleep settings
  uint32_t wakeTime;
  uint32_t globalTimeOffset;
  #ifdef FEATURE_SENSOR_MPU
    uint32_t sensor_mpu_timer;
  #endif
  #ifdef FEATURE_SENSOR_DHT22
    uint32_t sensor_dht22_timer;
  #endif
  #ifdef FEATURE_SENSOR_DHT11
    uint32_t sensor_dht11_timer;
  #endif
  #ifdef FEATURE_SENSOR_BATTERY
    uint32_t sensor_battery_timer;
  #endif
  #ifdef FEATURE_SENSOR_BMP280
    uint32_t sensor_bmp280_timer;
  #endif
  #ifdef FEATURE_SENSOR_LIS3DH
    uint32_t sensor_lis3dh_timer;
  #endif
  #ifdef FEATURE_FOTA
    uint32_t fota_update_timer;
  #endif
  uint32_t led_timer;
};
struct _rtcData rtcData = {};

void wifiSave( void ) {
  // Write current connection info back to RTC
  rtcData.channel = WiFi.channel();
  memcpy( rtcData.bssid, WiFi.BSSID(), 6 ); // Copy 6 bytes of BSSID (AP's MAC address)
  rtcData.wifiValid = true;
}
#ifdef FEATURE_SENSOR_MPU
void mpuSave( uint32_t x, uint32_t y, uint32_t z ) {
  rtcData.mpuXcal = x;
  rtcData.mpuYcal = y;
  rtcData.mpuZcal = z;
  rtcData.mpuValid = true;
}
#endif
void sleepSave( uint32_t awake) {
  rtcData.globalTimeOffset += awake;
  rtcData.crc32 = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
  ESP.rtcUserMemoryWrite( 0, (uint32_t*)&rtcData, sizeof( rtcData ) );
}
bool sleepResume( void ) {
  // Try to read settings from RTC memory
  if( ESP.rtcUserMemoryRead( 0, (uint32_t*)&rtcData, sizeof( rtcData ) ) ) {
    // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
    if( crc == rtcData.crc32 ) {
      Serial.println("RTC valid!");
      rtcData.globalTimeOffset += DEEPSLEEP_INTERVAL;
      Serial.print("globalTimeOffset = "); Serial.println(rtcData.globalTimeOffset);
      return true;
    }
  }
  Serial.println("RTC NOT valid!");
  memset( &rtcData, 0, sizeof( rtcData ) );
  return false;
}
#endif

#ifdef FEATURE_SENSOR_MPU
  #include <MPU6050_tockn.h>
  #if defined(ARDUINO_ESP8266_GENERIC)
    MPU6050 mpu6050(i2c);
  #else
    MPU6050 mpu6050(Wire);
  #endif

  #define SENSOR_MPU_PERIOD 1000 // ms
  #ifdef FEATURE_DEEPSLEEP
    #define sensor_mpu_timer rtcData.sensor_mpu_timer
  #else
    uint32_t sensor_mpu_timer=0;
  #endif
#endif

#ifdef FEATURE_SENSOR_DHT22
  #include <TinyDHT.h>
  DHT dht22_1(6, DHT22);
  #define DHT22_ALL
  #ifdef DHT22_ALL
  DHT dht22_2(7, DHT22);
  DHT dht22_3(8, DHT22);
  DHT dht22_4(9, DHT22);
  DHT dht22_5(10, DHT22);
  #endif
  #define SENSOR_DHT22_PERIOD 10000 // 2s max
  #ifdef FEATURE_DEEPSLEEP
    #define sensor_dht22_timer rtcData.sensor_dht22_timer
  #else
    uint32_t sensor_dht22_timer=0;
  #endif
#endif

#ifdef FEATURE_SENSOR_DHT11
  #include <TinyDHT.h>
  DHT dht11_2(12, DHT11);
  #define DHT11_ALL
  #ifdef DHT11_ALL
  DHT dht11_5(11, DHT11);
  #endif
  #define SENSOR_DHT11_PERIOD 10000 // 2s max
  #ifdef FEATURE_DEEPSLEEP
    #define sensor_dht11_timer rtcData.sensor_dht11_timer
  #else
    uint32_t sensor_dht11_timer=0;
  #endif
#endif

#ifdef FEATURE_SENSOR_BATTERY
  #define SENSOR_BATTERY_PERIOD 10000 // ms
  #ifdef FEATURE_DEEPSLEEP
    #define sensor_battery_timer rtcData.sensor_battery_timer
  #else
    uint32_t sensor_battery_timer=0;
  #endif
#endif


#ifdef FEATURE_SENSOR_BMP280
  #include <Adafruit_BMP280.h>
  #if defined(ARDUINO_ESP8266_GENERIC)
    Adafruit_BMP280 bmp(&i2c);
  #else
    Adafruit_BMP280 bmp;
  #endif
  bool bmp280_init = false;
  #define SENSOR_BMP280_PERIOD 10000 // ms
  #ifdef FEATURE_DEEPSLEEP
    #define sensor_bmp280_timer rtcData.sensor_bmp280_timer
  #else
    uint32_t sensor_bmp280_timer=0;
  #endif
#endif

#ifdef FEATURE_SENSOR_LIS3DH
  #include "Adafruit_LIS3DH.h"
  Adafruit_LIS3DH lis = Adafruit_LIS3DH();
  bool lis3dh_init = false;
  #define SENSOR_LIS3DH_PERIOD 10000 // ms
  #ifdef FEATURE_DEEPSLEEP
    #define sensor_lis3dh_timer rtcData.sensor_lis3dh_timer
  #else
    uint32_t sensor_lis3dh_timer=0;
  #endif
#endif

#ifdef FEATURE_TNN_RADIO
  const char *appEui = "70B3D57ED0041609";
  const char *appKey = "654DAE5088F66FAA9C8B774543A8F395";
#endif

#ifdef FEATURE_WIFI_RADIO
WiFiClient EspClient;
#define RESCAN_MAX_COUNT 15

#ifdef FEATURE_STATIC_IP
struct ipmac {
  char hostname[12];
  char mac[18];
  IPAddress ip;
};
// lookup table for fixed IP addresses to avoid collisions
const struct ipmac ipmacs[] = {
  { "geiger1", "5C:CF:7F:04:C5:22", IPAddress( 192, 168, 1, 77 ) }, // geiger 1 1MB
  { "geiger2", "5C:CF:7F:04:C1:12", IPAddress( 192, 168, 1, 76 ) }, // geiger 2 1MB
  { "module0", "5C:CF:7F:04:C6:05", IPAddress( 192, 168, 1, 75 ) }, // module 0 16MB
  { "pongsat1", "5C:CF:7F:04:C3:BD", IPAddress( 192, 168, 1, 74 ) }, // module 1 16MB
  { "rocket0", "5C:CF:7F:04:C4:99", IPAddress( 192, 168, 1, 73 ) }, // module 2 16MB
  { "module3", "5C:CF:7F:04:C5:FE", IPAddress( 192, 168, 1, 72 ) }, // module 3 16MB
  { "module4", "5C:CF:7F:04:C7:06", IPAddress( 192, 168, 1, 71 ) }, // module 4 16MB
  { "module5", "5C:CF:7F:04:C4:57", IPAddress( 192, 168, 1, 70 ) }, // module 5 16MB
  { "module6", "5C:CF:7F:04:C5:C4", IPAddress( 192, 168, 1, 69 ) }, // module 6 16MB
  { "module7", "5C:CF:7F:04:C7:30", IPAddress( 192, 168, 1, 68 ) }, // module 7 16MB
  { "module8", "5C:CF:7F:04:C5:C0", IPAddress( 192, 168, 1, 67 ) }, // module 8 16MB
  { "module9", "5C:CF:7F:04:C2:67", IPAddress( 192, 168, 1, 66 ) }, // module 9 16MB
  { "module10", "5C:CF:7F:04:C6:D0", IPAddress( 192, 168, 1, 65 ) }, // module 10 1MB
  { "module11", "5C:CF:7F:04:C5:EA", IPAddress( 192, 168, 1, 64 ) }, // module 11 1MB
  { "module12", "5C:CF:7F:04:C1:DE", IPAddress( 192, 168, 1, 63 ) }, // module 12 1MB
  { "pongsat13", "5C:CF:7F:04:C1:B0", IPAddress( 192, 168, 1, 62 ) }, // module 13 1MB
  { "module14", "5C:CF:7F:04:C5:A7", IPAddress( 192, 168, 1, 61 ) }, // module 14 1MB
  { "module15", "5C:CF:7F:04:C5:FF", IPAddress( 192, 168, 1, 60 ) }, // module 15 1MB
//  { "module16", "", IPAddress( 192, 168, 1, 59 ) }, // module 16 dead?
  { "module17", "18:FE:34:9B:68:F6", IPAddress( 192, 168, 1, 58 ) }  // module 17 original (512kB)
};

const IPAddress* getIPAddress( const String &mac ) {
  unsigned int i;
  for( i = 0; i < sizeof(ipmacs)/sizeof(struct ipmac); i++ ) {
    if (mac.equals(ipmacs[i].mac)) {
      return &ipmacs[i].ip;
    }
  }
  return NULL;
}
const char* getHostname( const String &mac ) {
  unsigned int i;
  for( i = 0; i < sizeof(ipmacs)/sizeof(struct ipmac); i++ ) {
    if (mac.equals(ipmacs[i].mac)) {
      return ipmacs[i].hostname;
    }
  }
  return NULL;
}
#endif

void scan() {
  Serial.begin(115200);

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(10);
  WiFi.forceSleepBegin();
  delay(10);
  WiFi.forceSleepWake();
  delay(100);

  Serial.println("\nscanning...");

  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();

  if (n > 0) {
    for (int i = 0; i < n; ++i) {
      unsigned char *bssid = WiFi.BSSID(i);

      // Print SSID and RSSI for each network found
      Serial.print("\rnew AP: ");
      Serial.print(WiFi.SSID(i));
      Serial.print("\t(");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")\t");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE)?" \t":"*\t");
      delay(10);
    }
  }

  // Wait a bit before scanning again
  //delay(100);
}

void wifi_setup() {
  int s,attempt_cnt=0, rescan_cnt=0;
  bool wifiStarted = false;

  Serial.print("Connecting to: ");
  Serial.println(ssid);
  WiFi.forceSleepWake();
  delay( 1 );
  WiFi.persistent(false); // do not save wifi creds
  WiFi.mode(WIFI_STA);
  #ifdef CONFIG_HOSTNAME
    Hostname = CONFIG_HOSTNAME;
  #else
    Hostname = getHostname(WiFi.macAddress());
  #endif
  WiFi.hostname(Hostname);
  Serial.print("My MAC: ");
  Serial.println(WiFi.macAddress());
  while ((s=WiFi.status()) != WL_CONNECTED) {// && WiFi.localIP().toString() == "(IP unset)" && WiFi.waitForConnectResult() != WL_CONNECTED) {
    switch(s) {
      case WL_IDLE_STATUS:
        Serial.println("WL_IDLE_STATUS");
        break;
      case WL_NO_SSID_AVAIL:
        Serial.println("WL_NO_SSID_AVAIL");
        #ifdef FEATURE_DEEPSLEEP
          rtcData.wifiValid = false;
        #endif
        break;
      case WL_SCAN_COMPLETED:
        Serial.println("WL_SCAN_COMPLETED");
        break;
      case WL_CONNECTED:
        Serial.println("WL_CONNECTED");
        break;
      case WL_CONNECTION_LOST:
        Serial.println("WL_CONNECTION_LOST");
        break;
      case WL_DISCONNECTED:
        Serial.print(".");
        break;
      case WL_CONNECT_FAILED:
        Serial.print("WL_CONNECT_FAILED - ");
        switch(s=wifi_station_get_connect_status()) {
          case STATION_IDLE:
            Serial.println("STATION_IDLE");
            break;
          case STATION_CONNECTING:
            Serial.println("STATION_CONNECTING");
            break;
          case STATION_WRONG_PASSWORD:
            Serial.println("STATION_WRONG_PASSWORD");
            break;
          case STATION_NO_AP_FOUND:
            Serial.println("STATION_NO_AP_FOUND");
            break;
          case STATION_CONNECT_FAIL:
            Serial.println("STATION_CONNECT_FAIL");
            break;
          case STATION_GOT_IP:
            Serial.println("STATION_GOT_IP");
            break;
          default:
            Serial.print("unknown station error -");
            Serial.println(s);
            break;
        }
        s = WL_IDLE_STATUS;
        break;
      default:
        Serial.print("unknown return value - ");
        Serial.println(s);
        WiFi.printDiag(Serial);
        s = WL_IDLE_STATUS;
        break;
    }
    delay(CONNECTION_ATTEMPT_DELAY);
    if(attempt_cnt++>CONNECTION_ATTEMPT_NUM_TRIES || s==WL_IDLE_STATUS) {
      Serial.print("\n\rattempting connection..");
      attempt_cnt=0;
      rescan_cnt++;
      if (rescan_cnt == RESCAN_MAX_COUNT ) {
        rescan_cnt=0;
        #ifdef FEATURE_DEEPSLEEP
        rtcData.wifiValid=false; // force a wifi restart
        #endif
        scan();
      }

      #ifdef FEATURE_DEEPSLEEP
      if( rtcData.wifiValid ) {
        // The RTC data was good, make a quick connection
        Serial.print("using stored wifi details, channel ");
        Serial.print(rtcData.channel);
        Serial.print("\n");
        if ( !wifiStarted ) {
          WiFi.begin( ssid, password, rtcData.channel, rtcData.bssid, true );
          wifiStarted = true;
        }
      } else
      #endif
      {
        // The RTC data was not valid, so make a regular connection
        if ( !wifiStarted ) {
          WiFi.begin( ssid, password );
          wifiStarted = true;
        }
      }
      #ifdef FEATURE_STATIC_IP
      const IPAddress *ip = getIPAddress(WiFi.macAddress());
      if (ip != NULL ) {
        // static DHCP data, should probably save this on sleep
        WiFi.config( *ip, gateway, subnet );
      }
      #endif
    }
  }
  //Serial.setDebugOutput(false);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  #ifdef FEATURE_DEEPSLEEP
    wifiSave(); // save wifi stuff for quick connect on sleep resume
  #endif
}
#endif

#ifdef FEATURE_MQTT
PubSubClient client(EspClient);

void reconnect() {
  int ret;
  int retry_count=0;
  while (!client.connected() && retry_count++<CONNECTION_ATTEMPT_NUM_TRIES) {
    Serial.print("Attempting MQTT connection..."); Serial.print(millis()); Serial.print(" ");
    String ClientId = "ESP8266";
    ClientId += String(random(0xffff), HEX);
    if (client.connect(ClientId.c_str())) {
      Serial.println("connected");
      client.publish(mqtt_topic_status, "MQTT connected");
      ret=client.subscribe(mqtt_topic_in);
      Serial.print("subscribe returned: ");
      Serial.println(ret);
      Serial.print("MQTT connected at "); Serial.println(millis());
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      Serial.println("Try again...");
      delay(CONNECTION_ATTEMPT_DELAY);
    }
  }
}

void sendMQTTMessage(const char* topic, const char* value) {
  int retry_count=0;
  Serial.println("sending mqtt msg");
  if (!client.connected()) {
    reconnect();
  }
  client.publish(topic, value, true);
}
bool last_loop_received;
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("mqtt msg arrived (");
  Serial.print(topic);
  Serial.print(") : ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.print("\n");
  last_loop_received = true;
}
#endif

#if defined( FEATURE_WIFI_RADIO ) && defined( FEATURE_FOTA )
const char* fwUrlBase = "http://192.168.2.10:8080/fota/"; // pangolin
#define FOTA_UPDATE_PERIOD 30000 // ms
#ifdef FEATURE_DEEPSLEEP
  #define fota_update_timer rtcData.fota_update_timer
#else
  uint32_t fota_update_timer=0-FOTA_UPDATE_PERIOD;
#endif

void checkForUpdates() {
  String fwURL = String( fwUrlBase );
  fwURL.concat( Hostname );
  String fwVersionURL = fwURL;
  fwVersionURL.concat( ".version" );

  Serial.println( "Checking for firmware updates." );
  Serial.print( "Hostname: " );
  Serial.println( Hostname );
  Serial.print( "Firmware version URL: " );
  Serial.println( fwVersionURL );

  HTTPClient httpClient;
  httpClient.begin( fwVersionURL );
  int httpCode = httpClient.GET();
  if( httpCode == 200 ) {
    String newFWVersion = httpClient.getString();
    newFWVersion.trim();
    String message = "Current fw ver: ";
    message.concat( PAYLOAD_VERSION );
    message.concat( " Available fw ver: ");
    message.concat( newFWVersion );
    Serial.println( message );

    unsigned int newVersion = newFWVersion.toInt();

    if( newVersion > PAYLOAD_VERSION ) {
      Serial.println( "Preparing to update" );
      #ifdef FEATURE_MQTT
        char msg[40];
        snprintf(msg,sizeof(msg), "upgrading to %d", newVersion);
        sendMQTTMessage(mqtt_topic_status, msg);
      #endif

      String fwImageURL = String( fwUrlBase );
      fwImageURL.concat( newFWVersion );
      fwImageURL.concat( ".bin" );
      Serial.println( fwImageURL );
      t_httpUpdate_return ret = ESPhttpUpdate.update( fwImageURL );

      switch(ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
          break;

        case HTTP_UPDATE_OK:
          Serial.println("HTTP_UPDATE_OK");
        default:
          Serial.println("rebooting...");
          ESP.restart();
          for(;;){};
      }
    }
    else {
      Serial.println( "Already on latest version" );
    }
  }
  else {
    Serial.print( "Firmware version check failed, got HTTP response code " );
    Serial.println( httpCode );
  }
  httpClient.end();
}

#endif


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize LED
  Serial.begin(115200);
  #ifdef FEATURE_DEEPSLEEP
  if ( sleepResume() ) {
    Serial.println("resumed");
  } else
  #endif
  {
    Serial.println("boot");
  }
  LED_SETUP();

  #ifdef ARDUINO_ESP8266_GENERIC
    // reduce boot power by turning off wifi radio
    WiFi.mode( WIFI_OFF );
    WiFi.forceSleepBegin();
    delay( 1 );
  #endif

  #ifdef I2C_SDA
    // override default i2c pins
    i2c.begin(I2C_SDA,I2C_SCL);
  #endif

  #ifdef FEATURE_PSRAM
    Serial.print("Total heap: "); Serial.println(ESP.getHeapSize());
    Serial.print("Free heap: "); Serial.println(ESP.getFreeHeap());
    Serial.print("Total PSRAM: "); Serial.println(ESP.getPsramSize());
    Serial.print("Free PSRAM: "); Serial.println(ESP.getFreePsram());
    #if defined( FEATURE_SENSOR_MIC )
      wavBuf = (uint8_t*)ps_malloc( WAV_BUF_SIZE );
      if ( wavBuf == NULL ) {
        Serial.println("ERROR: allocation failed");
        for(;;); // halt
      }
    #endif
    Serial.print("Free PSRAM: "); Serial.println(ESP.getFreePsram());
  #endif

  #ifdef FEATURE_SENSOR_DHT22
    dht22_1.begin();
    #ifdef DHT22_ALL
    dht22_2.begin();
    dht22_3.begin();
    dht22_4.begin();
    dht22_5.begin();
    #endif
  #endif

  #ifdef FEATURE_SENSOR_DHT11
    dht11_2.begin();
    #ifdef DHT11_ALL
    dht11_5.begin();
    #endif
  #endif

  #ifdef FEATURE_SENSOR_BMP280
    if ( !bmp.begin(0x76) ) {
      Serial.println("ERROR: BMP280 failed to init");
    } else {
      Serial.println("BMP280 detected at 0x76");
      bmp280_init = true;
    }
  #endif

  #ifdef FEATURE_SENSOR_LIS3DH
    // this sensor appears to have an offset, add the following values as arg 3 to resolve:
    // #1 - 27degC
    // #2 - 7degC
    // #3 - 14degC
    if( !lis.begin(0x18, 0x33, 27)) {
      Serial.println("ERROR: LIS3DH failed to init");
    } else {
      Serial.println("LIS3DH detected at 0x18");
      lis.setRange(LIS3DH_RANGE_4_G);
      lis.setDataRate(LIS3DH_DATARATE_50_HZ);
      lis3dh_init = true;
    }
  #endif

  #ifdef FEATURE_SENSOR_MPU
    #if defined(ARDUINO_ESP8266_GENERIC)
      wdt_disable();
    #endif
    mpu6050.begin();
    #ifdef FEATURE_DEEPSLEEP
      if ( rtcData.mpuValid ) {
        Serial.println("using stored MPU calibration");
        mpu6050.setGyroOffsets( rtcData.mpuXcal, rtcData.mpuYcal, rtcData.mpuZcal );
      } else {
        mpu6050.calcGyroOffsets(true);
        mpuSave( mpu6050.getGyroXoffset(), mpu6050.getGyroYoffset(), mpu6050.getGyroZoffset() );
      }
    #else
      mpu6050.calcGyroOffsets(true);
    #endif
    #if defined(ARDUINO_ESP8266_GENERIC)
      wdt_enable(WDTO_0MS);
    #endif
    Serial.println("=");
  #endif


  #ifdef FEATURE_WIFI_RADIO
    // turn on wifi
    wifi_setup();
    Serial.print("wifi connected at "); Serial.println(millis());
  #endif

  #ifdef FEATURE_MQTT
    snprintf(mqtt_topic_status, sizeof(mqtt_topic_status), "%s/status", Hostname );
    snprintf(mqtt_topic_in, sizeof(mqtt_topic_in), "%s/in", Hostname );
    snprintf(mqtt_topic_out, sizeof(mqtt_topic_out), "%s/out", Hostname );
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    {
      char msg[40];
      snprintf(msg,sizeof(msg), "version %d", PAYLOAD_VERSION);
      sendMQTTMessage(mqtt_topic_status, msg);
    }

  #endif

}

uint32_t curtime=0;

unsigned int led_state = 0;
#define LED_PERIOD 2000 // ms
#ifdef FEATURE_DEEPSLEEP
  #define led_timer rtcData.led_timer
#else
  uint32_t led_timer=0;
#endif

// the loop function runs over and over again forever
void loop() {
  #ifdef FEATURE_SENSOR_MPU
    mpu6050.update();
  #endif

  #ifdef FEATURE_DEEPSLEEP
    curtime=millis()+rtcData.globalTimeOffset;
  #else
    curtime=millis();
  #endif
  Serial.print("curtime = "); Serial.println(curtime);
  if (curtime-led_timer > LED_PERIOD/2) {
    led_state = (led_state+1);
    if ( led_state > 3 ) led_state = 0;
    switch(led_state) {
      case 0:
        Serial.println("RED");
        LED_RED();
        break;
      case 1:
        Serial.println("GREEN");
        LED_GREEN();
        break;
      case 2:
        Serial.println("BLUE");
        LED_BLUE();
        break;
      case 3:
        Serial.println("OFF");
        LED_OFF();
        break;
    }

    led_timer=curtime;
  }

  #ifdef FEATURE_SENSOR_BATTERY
  if(curtime - sensor_battery_timer > SENSOR_BATTERY_PERIOD) {
    uint16_t rawBat = ESP.getVcc();
    Serial.print("BAT{");
    Serial.print(rawBat); Serial.println("}");
    #ifdef FEATURE_MQTT
    {
      char msg[40];
      snprintf(msg,sizeof(msg), "BAT @%.2fV", ((float)rawBat)/1024);
      sendMQTTMessage(mqtt_topic_out, msg);
      Serial.println(msg);
    }
    #endif
    sensor_battery_timer = curtime;
  }
  #endif

  #ifdef FEATURE_SENSOR_MPU
  if(curtime - sensor_mpu_timer > SENSOR_MPU_PERIOD) {
    Serial.print("MPU{");

    float t = mpu6050.getTemp(); Serial.print(t); Serial.print(",");

    float ax = mpu6050.getAccX(); Serial.print(ax); Serial.print(",");
    float ay = mpu6050.getAccY(); Serial.print(ay); Serial.print(",");
    float az = mpu6050.getAccZ(); Serial.print(az); Serial.print(",");

    float gx = mpu6050.getGyroX(); Serial.print(gx); Serial.print(",");
    float gy = mpu6050.getGyroY(); Serial.print(gy); Serial.print(",");
    float gz = mpu6050.getGyroZ(); Serial.print(gz); Serial.println("}");

    #ifdef FEATURE_MQTT
    {
      char msg[80];
      snprintf(msg,sizeof(msg), "MPU6050 @%.1fC (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f)", t, ax, ay, az, gx, gy, gz);
      sendMQTTMessage(mqtt_topic_out, msg);
      Serial.println(msg);
    }
    #endif

    sensor_mpu_timer = curtime;
  }
  #endif

  #ifdef FEATURE_SENSOR_DHT22
  if(curtime - sensor_dht22_timer > SENSOR_DHT22_PERIOD) {
    Serial.print("DHT22{");
    #ifdef DHT22_ALL
      Serial.print("1{");
    #endif
    Serial.print(dht22_1.readHumidity()); Serial.print(",");
    Serial.print(dht22_1.readTemperature(false));
    #ifdef DHT22_ALL
      Serial.print("},");
    #else
      Serial.println("}");
    #endif
    #ifdef DHT22_ALL
    Serial.print("2{");
    Serial.print(dht22_2.readHumidity()); Serial.print(",");
    Serial.print(dht22_2.readTemperature(false)); Serial.print("},");
    Serial.print("3{");
    Serial.print(dht22_3.readHumidity()); Serial.print(",");
    Serial.print(dht22_3.readTemperature(false)); Serial.print("},");
    Serial.print("4{");
    Serial.print(dht22_4.readHumidity()); Serial.print(",");
    Serial.print(dht22_4.readTemperature(false)); Serial.print("},");
    Serial.print("5{");
    Serial.print(dht22_5.readHumidity()); Serial.print(",");
    Serial.print(dht22_5.readTemperature(false)); Serial.println("}}");
    #endif
    sensor_dht22_timer = curtime;
  }
  #endif

  #ifdef FEATURE_SENSOR_DHT11
  if(curtime - sensor_dht11_timer > SENSOR_DHT11_PERIOD) {
    Serial.print("DHT11{");
    #ifdef DHT11_ALL
      Serial.print("2{");
    #endif
    Serial.print(dht11_2.readHumidity()); Serial.print(",");
    Serial.print(dht11_2.readTemperature(false)); Serial.print("},");
    #ifdef DHT11_ALL
      Serial.print("},");
    #else
      Serial.println("}");
    #endif
    #ifdef DHT11_ALL
    Serial.print("5{");
    Serial.print(dht11_5.readHumidity()); Serial.print(",");
    Serial.print(dht11_5.readTemperature(false)); Serial.println("}}");
    #endif
    sensor_dht11_timer = curtime;
  }
  #endif

  #ifdef FEATURE_SENSOR_BMP280
  if(curtime - sensor_bmp280_timer > SENSOR_BMP280_PERIOD) {
    if ( !bmp280_init && !bmp.begin(0x76) ) {
      Serial.println("ERROR: BMP280 failed to init");
    } else {
      bmp280_init = true;
      Serial.print("BMP280{");
      Serial.print(bmp.readAltitude()); Serial.print(","); // m - meters
      Serial.print(bmp.readPressure()); Serial.print(","); // mb - millibars
      Serial.print(bmp.readTemperature()); Serial.println("}"); // deg C
      #ifdef FEATURE_MQTT
      {
        char msg[40];
        snprintf(msg,sizeof(msg), "BMP280 @%.1fC", bmp.readTemperature());
        sendMQTTMessage(mqtt_topic_out, msg);
        Serial.println(msg);
      }
      #endif
    }
    sensor_bmp280_timer = curtime;
  }
  #endif

  #ifdef FEATURE_SENSOR_LIS3DH
  if(lis3dh_init && (curtime - sensor_lis3dh_timer > SENSOR_LIS3DH_PERIOD)) {
    lis.read();
    Serial.print("LIS3DH{");
    Serial.print(lis.readTemperature()); Serial.print(",");
    Serial.print(lis.x); Serial.print(",");
    Serial.print(lis.y); Serial.print(",");
    Serial.print(lis.z); Serial.print(",");
    Serial.print(lis.x_g); Serial.print(",");
    Serial.print(lis.y_g); Serial.print(",");
    Serial.print(lis.z_g); Serial.println("}");
    sensor_lis3dh_timer = curtime;
  }
  #endif

  #if defined( FEATURE_WIFI_RADIO ) && defined( FEATURE_FOTA )
  if(curtime - fota_update_timer > FOTA_UPDATE_PERIOD ) {
    checkForUpdates(); // does not return if update is successful
    fota_update_timer = curtime;
  }
  #endif

  #ifdef FEATURE_DEEPSLEEP
    uint32_t awake = millis();
    Serial.print("going into deep sleep, was awake for "); Serial.println(awake);
    sleepSave(awake);
    delay(10);
    #ifdef FEATURE_MQTT
      client.setCallback(NULL); // clear callback
    #endif
    #ifdef FEATURE_WIFI_RADIO
      WiFi.disconnect( true );
    #endif
    delay( 1 );

    Serial.println("\nsleep...");
    ESP.deepSleep(DEEPSLEEP_INTERVAL * 1000, WAKE_RF_DISABLED);
    delay( 1 );

    Serial.println("\nsleep failed...");
    delay(100);
  #endif
}


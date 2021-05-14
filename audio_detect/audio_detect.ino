// based on  https://randomnerdtutorials.com/guide-for-microphone-sound-sensor-with-arduino/

/*
 * Rui Santos 
 * Complete Project Details http://randomnerdtutorials.com
*/

#if defined(ARDUINO_TINYPICO)
  #include <TinyPICO.h>
  #define LED_SETUP()
  #define LED_RED()  {tp.DotStar_SetBrightness(64); tp.DotStar_SetPixelColor( 255, 0, 0);}
  #define LED_GREEN() {tp.DotStar_SetBrightness(96); tp.DotStar_SetPixelColor( 0, 255, 0);}
  #define LED_BLUE() {tp.DotStar_SetBrightness(220); tp.DotStar_SetPixelColor( 0, 0, 255);}
  #define LED_OFF() {tp.DotStar_SetBrightness(0); tp.DotStar_SetPixelColor( 0, 0, 0);}
  TinyPICO tp = TinyPICO();
#else
  #error unknown board config
#endif

int sensorPinDigital=4;
int sensorPinAnalog=14;

void setup(){
  Serial.begin(115200);
  Serial.println("boot");
  LED_SETUP();
  pinMode(sensorPinDigital, INPUT);
}

void loop (){
  bool val = digitalRead(sensorPinDigital);
  Serial.println (val);
  // when the sensor detects a signal above the threshold value, LED flashes
  if (val==HIGH) {
    LED_GREEN();
  } else {
    LED_OFF();
  }
}

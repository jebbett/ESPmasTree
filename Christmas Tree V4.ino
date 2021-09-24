#define FASTLED_ALLOW_INTERRUPTS 0 //Stop random blinking
#include "FastLED.h" // Fast LED
FASTLED_USING_NAMESPACE

#include <ESP8266WiFi.h> // Wifi etc for OTA
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h> // UDP for NTP

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define DATA_PIN    3
//#define CLK_PIN   4
#define LED_TYPE    WS2811 //NEOPIXEL  //WS2812B 
#define COLOR_ORDER RGB //GRB
#define NUM_LEDS    200
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          130 //0-255
#define FRAMES_PER_SECOND  100

// ########## OTA Update over Wifi

#define HOSTNAME "Lights-OTA-" ///< Hostename. The setup function adds the Chip ID at the end.
const char* ap_default_ssid = "CHANGEME"; ///< Default SSID.
const char* ap_default_psk = "CHANGEME"; ///< Default PSK.
const char* station_ssid = "OTA";
const char* station_psk = "OTApassword1";
/// Uncomment the next line for verbose output over UART.
//#define SERIAL_VERBOSE


// ######### NTP Script
unsigned int localPort = 2390;      // local port to listen for UDP packets
IPAddress timeServerIP; // NTP server address
const char* ntpServerName = "uk.pool.ntp.org";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  delay(100);
  
//################ WIFI #####################
  // Set Hostname.
  String hostname(HOSTNAME);
  hostname += String(ESP.getChipId(), HEX);
  WiFi.hostname(hostname);
  Serial.println("Hostname: " + hostname);

  // Check WiFi connection
  // ... check mode
  if (WiFi.getMode() != WIFI_STA){
    WiFi.mode(WIFI_STA);
    delay(10);
  }
  WiFi.begin(ap_default_ssid, ap_default_psk);
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // ... Uncomment this for debugging output.
  //WiFi.printDiag(Serial);
  WiFi.begin();
  Serial.println("Wait for WiFi connection.");
  // ... Give ESP 10 seconds to connect to station.
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000){
    Serial.write('.');
    delay(500);
  }
  Serial.println();
  if(WiFi.status() == WL_CONNECTED){
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }else{
    Serial.println("Can not connect to WiFi station. Go into AP mode.");
    WiFi.mode(WIFI_AP);
    delay(10);
    WiFi.softAP(station_ssid, station_psk);
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
  }

//################ OTA #####################

  // Start OTA server.
  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();

//################ NTP #####################

  // Setup UDP for NTP
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());

//################ LED #####################

  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
  fill_solid( leds, NUM_LEDS, CRGB::Gold );
  FastLED.show();
}


// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { onTheHour, holly, RGBThing, bounce, xmasLoop, rgbFade, sparkle, RainbowThing, confetti, sinelon, juggle, bpm, popopo, rainbowSpin };
uint8_t gCurrentPatternNumber = 1; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
uint8_t bulbCount = 0;
uint8_t NextPatternSeconds = 60; //How often to change the pattern
uint8_t lastMinute = 57;  //set to 57 to get an immediate NTP update
uint8_t lastSecond = 59;  //set starting seconds to 0
void loop()
{
  // Handle OTA server.
  ArduinoOTA.handle();
  yield();

  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();
    
  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND);

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  EVERY_N_SECONDS( NextPatternSeconds ) { if( lastMinute != 0 ) { nextPattern(); } } // change patterns periodically
  EVERY_N_SECONDS( 1 ) { // Add 1 second and update on the hour
    
    lastSecond = (lastSecond + 1) % 60;
    Serial.println(lastSecond);
    if (lastSecond == 0){
      lastMinute = (lastMinute + 1) % 60;
      Serial.print(lastMinute);
      Serial.println(" minutes past");
      if (lastMinute == 58){
        requestTime();
      }
      else if (lastMinute <= 1){
        nextPattern();
      }
    }
    if (lastMinute == 58){ //only wait for time update during the 58th minute
      getMinute();
    }
  } 
}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
bool first = true;

void nextPattern()
{   
  if (lastMinute == 0){
      gCurrentPatternNumber = 0;
  }else{
    // add one to the current pattern number, and wrap around at the end
    gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns );
    if (gCurrentPatternNumber == 0){gCurrentPatternNumber = 1;} // so it doesn't loop to the on the hour entry
  }
  first = true;
}

uint8_t timeTries = 0;
bool gotTime = false;
void requestTime(){
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);
  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  timeTries = 0;
  gotTime = false;
}

void getMinute(){
  // wait for a reply for UDP_TIMEOUT miliseconds
  if (gotTime == false){
    unsigned long startMs = millis();
    int cb = udp.parsePacket();
    if (!cb) {
      Serial.println("no packet yet");
      timeTries = timeTries + 1;
      if(timeTries == 5){
        requestTime();
      }
    } else {
      gotTime = true;
      Serial.print("packet received, length=");
      Serial.println(cb);
      // We've received a packet, read the data from it
      udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:
  
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      // now convert NTP time into everyday time:
      unsigned long epoch = secsSince1900 - 2208988800UL; // Unix time starts on Jan 1 1970. In seconds, that's 2208988800, so subtract seventy years:
      lastMinute = (epoch  % 3600) / 60;
      lastSecond = (epoch  % 60);
    }
  }
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address) {
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

uint8_t tenPos = 0;
void onTheHour()
{
  fill_solid( leds, NUM_LEDS, CRGB::Blue );

  for(int i = tenPos; i < NUM_LEDS; i=i+30){
    leds[i] = CRGB( 255, 255, 255);
    leds[i+1] = CRGB( 155, 155, 255);
    leds[i+2] = CRGB( 55, 55, 255);
  }
  tenPos = (tenPos +1) % 30;
}

uint8_t bouncePos = 0;
uint8_t colourNum = 0;

void bounce()
{
  // bounce up the tree
  int prevPos = bouncePos;
  bouncePos = beatsin16( 20, 0, NUM_LEDS-1 );

  for(int q = 0; q < NUM_LEDS; q++){
    if (q <= bouncePos){ 
      if(colourNum == 0){
        leds[q] = CRGB::Red;
      }else if(colourNum == 1){
        leds[q] = CRGB::Green;
      }else{
        leds[q] = CRGB::Blue;
      }
    }else{
      //leds[q] = CRGB::Black;
      leds[q].fadeToBlackBy(64);
    }
  }
  if(bouncePos == 0 && prevPos != 0){
      colourNum = (colourNum + 1) % 3;
  }
}

int cycleNum = 0;
void rgbFade()
{
  for(int i = 0; i < NUM_LEDS; i++){
    if(cycleNum==0){
      // Add and Subtract one color from another.
      leds[i] += CRGB( 1, 0, 0);
      leds[i] -= CRGB( 0, 1, 1);
    }else if (cycleNum==1){
      // Add and Subtract one color from another.
      leds[i] += CRGB( 0, 1, 0);
      leds[i] -= CRGB( 1, 0, 1);
    }else{
      // Add and Subtract one color from another.
      leds[i] += CRGB( 0, 0, 1);
      leds[i] -= CRGB( 1, 1, 0);
    }
  }
  if(leds[0] == CRGB( 255, 0, 0)){ cycleNum=1; }
  if(leds[0] == CRGB( 0, 255, 0)){ cycleNum=2; }
  if(leds[0] == CRGB( 0, 0, 255)){ cycleNum=0; }
}

uint8_t lightPos = 0;
void xmasLoop()
{
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int aThird = NUM_LEDS / 3;
  leds[lightPos] = CRGB::Red;
  leds[(lightPos + aThird) % NUM_LEDS] = CRGB::Green;
  leds[(lightPos + (2 * aThird)) % NUM_LEDS] = CRGB::Blue;
  lightPos = (lightPos + 1) % NUM_LEDS;
  
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowSpin() {
  static byte initialHue = 0;
  initialHue = initialHue + 1;
  byte changeInHue = 255 / NUM_LEDS;
  fill_rainbow(leds, NUM_LEDS, initialHue, changeInHue);
}

void popopo()
{
  //increament lead bulb number
  if(bulbCount==NUM_LEDS){
    bulbCount = 0;
  }else{
    bulbCount++;
  }
  
  // a looping trails
  int NUM_HALF = (30 / (NUM_LEDS/50)); //Groups of 50 bulbs, 20 is a good starting point, 1 = slow, 255 = fast
  int thedelay = 1500 / NUM_LEDS;
  fadeToBlackBy( leds, NUM_LEDS, NUM_HALF);
  
  leds[bulbCount] = CRGB::Green;
  //red trail
  int bulbCount2 = bulbCount + (NUM_LEDS / 3);
  if (bulbCount2 >= NUM_LEDS){
    bulbCount2 = bulbCount2 - NUM_LEDS;
  }
  leds[bulbCount2] = CRGB::Red;

  // blue trail
  int bulbCount3 = bulbCount + 2*(NUM_LEDS / 3);
  if (bulbCount3 >= NUM_LEDS){
    bulbCount3 = bulbCount3 - NUM_LEDS;
  }
  leds[bulbCount3] = CRGB::Blue;
}

void RGBsparkle() 
{
  for(int q = 0; q < NUM_LEDS; q=q+3) {
    leds[q] = CRGB::Red;
    leds[q+1] = CRGB::Green;
    leds[q+2] = CRGB::Blue;
  }
  addGlitter(80);
}

void holly() {
  if(first){
    first = false;
    for(int i = 0; i < NUM_LEDS; i++){
      leds[i] += CRGB( 0, 3, 0);
      leds[i] -= CRGB( 3, 0, 3);
      if(leds[i] != CRGB(0,255,0)){
        first = true;
      }
    }
  }else{
    EVERY_N_SECONDS( 1 ) {
      int pos = random(NUM_LEDS);
      leds[pos] = CRGB::Red;
    }
  }
}

void RGBThing() 
{
  int pos = random(NUM_LEDS);
  int val = random(4);
  switch (val) {
    case 0:
      leds[pos] = CRGB::Red;
      break;
    case 1:
      leds[pos] = CRGB::Green;
      break;
    case 2:
      leds[pos] = CRGB::Blue;
      break;
    case 3:
      leds[pos] = CRGB::Gold;
      break;
  }
}
void RainbowThing() 
{
  for(uint8_t q = 0; q < NUM_LEDS; q++) {
    leds[q] = CHSV( gHue + q, 255, 255);
  }

}

void sparkle() 
{
  // FastLED's built-in rainbow generator
  //fill_rainbow( leds, NUM_LEDS, gHue, 7);
  fill_solid( leds, NUM_LEDS, CRGB::Red );
  addGlitter(80);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

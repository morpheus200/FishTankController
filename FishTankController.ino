//#include <SD.h>
#include <Wire.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <WiFiEspUdp.h>
#include <UTFT.h>
#include <URTouch.h>
#include <tinyFAT.h> // used to acess the SD card
#include <UTFT_tinyFAT.h>  // used to read .raw images from the SD card
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <RCSwitch.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>


const char ssid[] = "Hamm_Unten";  //  your network SSID (name)
const char pass[] = "*******";       // your network password
const IPAddress ownIP(192, 168, 177, 66); //Change own IP
int status = WL_IDLE_STATUS;     // the Wifi radio's status

////////////////////////////////////////////NTP ///////////////////////////////////////
time_t ntpTime;
time_t rtcTime;
// NTP Servers:
static const char ntpServerName[] = "de.pool.ntp.org";
//static const char ntpServerName[] = "time.nist.gov";
const int timeZone = 1;     // Central European Time
unsigned int localPort = 2390;  // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48;  // NTP timestamp is in the first 48 bytes of the message
const int UDP_TIMEOUT = 2000;    // timeout in miliseconds to wait for an UDP packet to arrive
byte packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets
///////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////// PH Probe //////////////////////
#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength + 1]; // store the serial command
byte receivedBufferIndex = 0;

#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the sample voltage
int analogBufferIndex = 0;

#define SlopeValueAddress 0     // (slope of the ph probe)store at the beginning of the EEPROM. The slope is a float number,occupies 4 bytes.
#define InterceptValueAddress (SlopeValueAddress+4)
float slopeValue, interceptValue, averageVoltage;
//boolean enterCalibrationFlag = 0;
#define PHPin A0
#define VREF 5000  //for arduino uno, the ADC reference is the power(AVCC), that is 5000mV

static byte acidCalibrationFinish = 0, alkaliCalibrationFinish = 0;
static float acidValue, alkaliValue;
static float acidVoltage, alkaliVoltage;
//////////////////////////////////////////////////////////////////////////////////////


UTFT    myGLCD(ILI9341_16, 38, 39, 40, 41); //Parameters should be adjusted to your Display/Shield model
URTouch  myTouch( 6, 5, 4, 3, 2);
UTFT_tinyFAT myFiles(&myGLCD);  // start up an instance to read images from the SD card
int currentPage;
// 1-home, 2-power, 3-ph, 4-ph-calibrate,
extern uint8_t BigFont[];
RCSwitch mySwitch = RCSwitch();
OneWire oneWire(11);   //Change Pin
DallasTemperature tempSensor(&oneWire);
WiFiEspUDP Udp;

/*
   Setup initierung
*/
void setup() {
  // Initial setup
  Serial.begin(115200); //startet seriellen Monitor
  Serial.println("Serial ready");
  delay(50);
  // init SD card
  file.setSSpin(53);
  Serial.println(file.initFAT(SPISPEED_VERYHIGH));
  //pinMode(53, OUTPUT);
  //Serial.println("SD-Card state :" + sdCard.init(SPI_HALF_SPEED,53));
  //file.setSSpin(53);
  //file.initFAT(SPISPEED_VERYHIGH);

  myGLCD.InitLCD();
  myTouch.InitTouch();
  myTouch.setPrecision(PREC_MEDIUM);
  myGLCD.clrScr();
  myFiles.loadBitmap(26,80,188,72,"iAqua.raw");
  Serial.println("LCD initied");
  //drawHomeScreen();
  currentPage = 0;

  //readCharacteristicValues(); //Lesen der Chrakteritstichen Werte für PH-Messung aus EEPROM
  //mySwitch.enableReceive(0); //Lesen auf PIN 2
  //mySwitch.enableTransmit(10); //433MHz sender einschalten Change pin

  //initWiFi();
  tempSensor.begin(); //Starten des Temperatursensors
}



void loop() {
  /*if (timeStatus() != timeNotSet) { //wenn die Zeit nicht aktuell ist wird sie mit NTP gesetzt
    ntpTime = receiveNTPPacket();
    if (ntpTime != 0) {
      setTime(ntpTime);
      RTC.set(ntpTime); // Setzt den DS1307RTC
    }
  }*/

}


void initWiFi() {
  Serial3.begin(115200); 
  WiFi.init(&Serial3); // initialize ESP module
  // check for the presence of the shield

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
     // don't continue
     //while (true);
    } else {

     WiFi.config(ownIP); //WIFi eigene IP setzen.
     while ( status != WL_CONNECTED) {
       Serial.print("Attempting to connect to WPA SSID: ");
       Serial.println(ssid);
       // Connect to WPA/WPA2 network
       status = WiFi.begin(ssid, pass); //mit Wifi verbinden
     }

     Serial.println("Connected to wifi");
     printWifiStatus(); //WifiStatus wiedergeben
  
     Serial.print("IP number assigned by DHCP is ");
     Serial.println(WiFi.localIP());
     Serial.println("Starting UDP");
     Udp.begin(localPort); //UDP staten
   }
}

/*
   Gibt die SSID und die IP-Adresse wieder

*/
void printWifiStatus() {
  // print the SSID
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

/*
   Startet das Senden einer NTP Anfrage und verarbeitet die Antwort.
   Werte werden als time_t gespeichert zurückgegeben.
*/
time_t receiveNTPPacket()
{
  sendNTPpacket(ntpServerName); // send an NTP packet to a time server

  // wait for a reply for UDP_TIMEOUT miliseconds
  unsigned long startMs = millis();
  while (!Udp.available() && (millis() - startMs) < UDP_TIMEOUT) {}

  Serial.println(Udp.parsePacket());
  if (Udp.parsePacket()) { //prüfen ob ein UDP Paket vorhanden ist
    Serial.println("packet received");
    // We've received a packet, read the data from it into the buffer
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    unsigned long secsSince1900;
    // convert four bytes starting at location 40 to a long integer
    secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
    secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
    secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
    secsSince1900 |= (unsigned long)packetBuffer[43];
    return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}


/*
    send an NTP request to the time server at the given address via UDP
*/
void sendNTPpacket(char *ntpSrv)
{
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
  Udp.beginPacket(ntpSrv, 123); //NTP requests are to port 123

  Udp.write(packetBuffer, NTP_PACKET_SIZE);

  Udp.endPacket();
}

/*
   Liest die aktuelle Temperatur
*/
float readTemp() {
  tempSensor.requestTemperatures();
  float celsius = tempSensor.getTempCByIndex(0);
  return celsius;
}

/*

*/
boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while (Serial.available() > 0)
  {
    if (millis() - receivedTimeOut > 1000U)
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer, 0, (ReceivedBufferLength + 1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex == ReceivedBufferLength) {
      receivedBufferIndex = 0;
      strupr(receivedBuffer);
      return true;
    }
    else {
      receivedBuffer[receivedBufferIndex] = receivedChar;
      receivedBufferIndex++;
    }
  }
  return false;
}

/*
   Median Filter Algorithmus um stabilen Wert zu erhalten
   Für Mittelwert der Spannung bei der PH Messung
*/
int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
  {
    bTab[i] = bArray[i];
  }
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

/*
   Lessen der Characteristischen Werte um den PH-Wert zu bestimmen
*/
void readCharacteristicValues()
{
  EEPROM_read(SlopeValueAddress, slopeValue);
  EEPROM_read(InterceptValueAddress, interceptValue);
  if (EEPROM.read(SlopeValueAddress) == 0xFF && EEPROM.read(SlopeValueAddress + 1) == 0xFF && EEPROM.read(SlopeValueAddress + 2) == 0xFF && EEPROM.read(SlopeValueAddress + 3) == 0xFF)
  {
    slopeValue = 3.5;   // If the EEPROM is new, the recommendatory slope is 3.5.
    EEPROM_write(SlopeValueAddress, slopeValue);
  }
  if (EEPROM.read(InterceptValueAddress) == 0xFF && EEPROM.read(InterceptValueAddress + 1) == 0xFF && EEPROM.read(InterceptValueAddress + 2) == 0xFF && EEPROM.read(InterceptValueAddress + 3) == 0xFF)
  {
    interceptValue = 0;  // If the EEPROM is new, the recommendatory intercept is 0.
    EEPROM_write(InterceptValueAddress, interceptValue);
  }
}

/*
   PH Wert Messen und ausgeben
*/

int meassurePH ()
{
  static unsigned long sampleTimepoint = millis();
  if (millis() - sampleTimepoint > 40U)
  {
    sampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(PHPin) / 1024.0 * VREF; //read the voltage and store into the buffer,every 40ms
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
    averageVoltage = getMedianNum(analogBuffer, SCOUNT);  // read the stable value by the median filtering algorithm
  }

  return (averageVoltage / 1000.0 * slopeValue + interceptValue);
}

/*
   Home Screen
*/
void drawHomeScreen() {
  myGLCD.clrScr();
  myGLCD.setBackColor(0, 0, 0); //Schwarzer hintergrund

  myGLCD.setColor(16, 167, 103); // Sets green color
  myGLCD.fillRoundRect (35, 90, 285, 130); // Draws filled rounded rectangle
  myGLCD.setColor(255, 255, 255); // Sets color to white
  myGLCD.drawRoundRect (35, 90, 285, 130); // Draws rounded rectangle without a fill, so the overall appearance of the button looks like it has a frame
  myGLCD.setFont(BigFont); // Sets the font to big
  myGLCD.setBackColor(16, 167, 103); // Sets the background color of the area where the text will be printed to green, same as the button
  myGLCD.print("PH", CENTER, 102); // Prints the string

   // Button - RGB LED Control
  myGLCD.setColor(16, 167, 103);
  myGLCD.fillRoundRect (35, 140, 285, 180);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (35, 140, 285, 180);
  myGLCD.setFont(BigFont);
  myGLCD.setBackColor(16, 167, 103);
  myGLCD.print("RGB LED CONTROL", CENTER, 152);

}

/*

*/
void calibrtionSelectScreen() {
  myGLCD.clrScr();
  myGLCD.setBackColor(0, 0, 0); //Schwarzer hintergrund
  myGLCD.setFont(BigFont); // Sets font to big
  myGLCD.print("PH Calibration", CENTER, 10); // Prints the string on the screen
  myGLCD.setColor(255, 0, 0); // Sets color to red
  myGLCD.drawLine(0, 32, 319, 32); // Draws the red line
  myGLCD.setColor(255, 255, 255); // Sets color to white


  // Button -Back
  myGLCD.setColor(16, 167, 103); // Sets green color
  myGLCD.fillRoundRect (35, 90, 285, 130); // Draws filled rounded rectangle
  myGLCD.setColor(255, 255, 255); // Sets color to white
  myGLCD.drawRoundRect (35, 90, 285, 130); // Draws rounded rectangle without a fill, so the overall appearance of the button looks like it has a frame
  myGLCD.setFont(BigFont); // Sets the font to big
  myGLCD.setBackColor(16, 167, 103); // Sets the background color of the area where the text will be printed to green, same as the button
  myGLCD.print("Back", CENTER, 102); // Prints the string
  drawHomeScreen();
}

/*

*/
void calibrationScreen() {
  float acidValueTemp, alkaliValueTemp, newSlopeValue, newInterceptValue;
  char *receivedBufferPtr;

  myGLCD.clrScr();
  myGLCD.setBackColor(0, 0, 0); //Schwarzer hintergrund
  if (!acidCalibrationFinish)
  {
    //Button ACID
    myGLCD.setColor(16, 167, 103); // Sets green color
    myGLCD.fillRoundRect (35, 90, 285, 130); // Draws filled rounded rectangle
    myGLCD.setColor(255, 255, 255); // Sets color to white
    myGLCD.drawRoundRect (35, 90, 285, 130); // Draws rounded rectangle without a fill, so the overall appearance of the button looks like it has a frame
    myGLCD.setFont(BigFont); // Sets the font to big
    myGLCD.setBackColor(16, 167, 103); // Sets the background color of the area where the text will be printed to green, same as the button
    myGLCD.print("ACID", CENTER, 102); // Prints the string

    /// Starten der Kalibrierung mit ACID 4.0
    receivedBufferPtr = strstr(receivedBuffer, "ACID:");
    receivedBufferPtr += strlen("ACID:");
    acidValueTemp = strtod(receivedBufferPtr, NULL);
    if ((acidValueTemp > 3) && (acidValueTemp < 5)) //typical ph value of acid standand buffer solution should be 4.00
    {
      acidValue = acidValueTemp;
      acidVoltage = averageVoltage / 1000.0;      // mV -> V
      acidCalibrationFinish = 1;
      Serial.println(F("Acid Calibration Successful"));
    } else {
      acidCalibrationFinish = 0;
      Serial.println(F("Acid Value Error"));
    }
    calibrationScreen();
  }

  if (!alkaliCalibrationFinish)
  {
    // Button -ALKALI
    myGLCD.setColor(16, 167, 103); // Sets green color
    myGLCD.fillRoundRect (35, 90, 285, 130); // Draws filled rounded rectangle
    myGLCD.setColor(255, 255, 255); // Sets color to white
    myGLCD.drawRoundRect (35, 90, 285, 130); // Draws rounded rectangle without a fill, so the overall appearance of the button looks like it has a frame
    myGLCD.setFont(BigFont); // Sets the font to big
    myGLCD.setBackColor(16, 167, 103); // Sets the background color of the area where the text will be printed to green, same as the button
    myGLCD.print("ALKALI", CENTER, 102); // Prints the string

    //// Starten der Kalibrierung mit ALKALI 10.0
    receivedBufferPtr = strstr(receivedBuffer, "ALKALI:");
    receivedBufferPtr += strlen("ALKALI:");
    alkaliValueTemp = strtod(receivedBufferPtr, NULL);
    if ((alkaliValueTemp > 8) && (alkaliValueTemp < 11)) //typical ph value of alkali standand buffer solution should be 9.18 or 10.01
    {
      alkaliValue = alkaliValueTemp;
      alkaliVoltage = averageVoltage / 1000.0;
      alkaliCalibrationFinish = 1;
      Serial.println(F("Alkali Calibration Successful"));
    } else {
      alkaliCalibrationFinish = 0;
      Serial.println(F("Alkali Value Error"));
    }
    calibrationScreen();
  }

  if (acidCalibrationFinish && alkaliCalibrationFinish)
  {
    newSlopeValue = (acidValue - alkaliValue) / (acidVoltage - alkaliVoltage); // Slope berechnen
    EEPROM_write(SlopeValueAddress, newSlopeValue); //Schreiben des neuen Slope
    newInterceptValue = acidValue - (slopeValue * acidVoltage); //Interceptwert berechnen
    EEPROM_write(InterceptValueAddress, newInterceptValue); // Schreiben den neuen Intercept Wertes
    Serial.print(F("Calibration Successful"));
  } else {
    Serial.print(F("Calibration Failed"));
    acidCalibrationFinish = 0;
    alkaliCalibrationFinish = 0;
  }

}

/*
   Empfangen von 433MHz
*/

void receiveSwitch() {

  if (mySwitch.available()) {
    int value = mySwitch.getReceivedValue();
    if (value == 0) {
      return;
    } else {
      //TODO: Code to receive
    }

    mySwitch.resetAvailable();
  }
}

/*
   Senden 433MHz
*/

void sendSwitch() {

  /* See Example: TypeA_WithDIPSwitches */
  mySwitch.switchOn("11111", "00010");
  delay(1000);
  mySwitch.switchOff("11111", "00010");
  delay(1000);

  /* Same switch as above, but using decimal code */
  mySwitch.send(5393, 24);
  delay(1000);
  mySwitch.send(5396, 24);
  delay(1000);

  /* Same switch as above, but using binary code */
  mySwitch.send("000000000001010100010001");
  delay(1000);
  mySwitch.send("000000000001010100010100");
  delay(1000);

  /* Same switch as above, but tri-state code */
  mySwitch.sendTriState("00000FFF0F0F");
  delay(1000);
  mySwitch.sendTriState("00000FFF0FF0");
  delay(1000);

}


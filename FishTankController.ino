#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <WiFiEspUdp.h>

#include <TimeLib.h>
#include <DS1307RTC.h>

const char ssid[] = "*************";  //  your network SSID (name)
const char pass[] = "********";       // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status
time_t ntpTime;
// NTP Servers:
static const char ntpServerName[] = "us.pool.ntp.org";
//static const char ntpServerName[] = "time.nist.gov";
//static const char ntpServerName[] = "time-a.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-b.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";

const int timeZone = 1;     // Central European Time
unsigned int localPort = 2390;  // local port to listen for UDP packets

const int NTP_PACKET_SIZE = 48;  // NTP timestamp is in the first 48 bytes of the message
const int UDP_TIMEOUT = 2000;    // timeout in miliseconds to wait for an UDP packet to arrive
byte packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets


WiFiEspUDP Udp;

void setup() {
  Serial.begin(115200);
  delay(250);
   Serial1.begin(9600);
  // initialize ESP module
  WiFi.init(&Serial1);
  
  //#Serial.println("TimeNTP Example");

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }
  
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass); //mit Wifi verbinden
  }
  
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.print("IP number assigned by DHCP is ");
  Serial.println(WiFi.localIP());
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  
  //#Serial.println("waiting for sync");
  
  //#setSyncProvider(getNtpTime);
  //#setSyncInterval(300);
  

}

void loop() {
  if (timeStatus() != timeNotSet) { //wenn die Zeit nicht aktuell ist wird sie mit NTP gesetzt
    ntpTime = receiveNTPPacket();
    if (ntpTime != 0){
      setTime(ntpTime);
    }
  }

}

// print the SSID of the network you're attached to
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


// send an NTP request to the time server at the given address
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

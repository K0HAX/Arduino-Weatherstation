#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP085_U.h>
#include <DHT.h>

#define WLAN_SSID "YOUR-SSID"
#define WLAN_PASS "YOUR-WIFI-PASSWORD"
#define WLAN_SECURITY WLAN_SEC_WPA2

#define ADAFRUIT_CC3000_IRQ   3
#define ADAFRUIT_CC3000_VBAT 5
#define ADAFRUIT_CC3000_CS 10

#define DHTPIN 2 // Pin for the humidity sensor
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIVIDER);
Adafruit_CC3000_Client client;

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

uint32_t ip;

const unsigned long responseTimeout = 15L * 1000L;

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Pressure Sensor Test");
  Serial.println("");
  
  /* Initialise the Sensor */
  if(!bmp.begin())
  {
    Serial.print("Oops, no BMP085 detected ... Check you wiring or I2C ADDR!");
    while(1);
  }
  
  dht.begin();
  
  Serial.println(F("\nInitialising the CC3000 ..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
    while(1);
  }
  
  #ifndef CC3000_TINY_DRIVER
    listSSIDResults();
  #endif
  
  uint16_t firmware = checkFirmwareVersion();
  if (firmware < 0x113) {
    Serial.println(F("Wrong firmware version!"));
    for(;;);
  }
  
  /* Delete any old connection data on the module */
  Serial.println(F("\nDeleting old connection profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("Failed!"));
    while(1);
  }
  
  /* Attempt to connect to an access point */
  char *ssid = WLAN_SSID;             /* Max 32 chars */
  Serial.print(F("\nAttempting to connect to ")); Serial.println(ssid);
  
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }  

  /* Display the IP address DNS, Gateway, etc. */  
  while (! displayConnectionDetails()) {
    delay(1000);
  }
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  bmp.getEvent(&event);
  
  /* Display the results (pressure is in hPa) */
  if (event.pressure)
  {
    /* Display the pressure in hPa */
    //Serial.print("Pressure: "); Serial.print(event.pressure); Serial.println(" hPa");
  }
  else
  {
    Serial.println("Sensor error");
  }
  
  float temperature;
  bmp.getTemperature(&temperature);
  temperature = temperature * 9;
  temperature = temperature / 5;
  temperature = temperature + 32;
  
  float humidity = dht.readHumidity();
  
  char jsonTemp[128];
  char t[10];
  char p[10];
  char h[10];
  
  jsonTemp[0] = '\0';

  snprintf(jsonTemp, sizeof jsonTemp, "?temperature=%s&pressure=%s&humidity=%s", dtostrf(temperature, 1, 1, t), dtostrf(event.pressure, 1, 1, p), dtostrf(humidity, 1, 1, h));
  Serial.println(jsonTemp);
  /// void sendJSON(char *address, char *page, char *data)
  sendJSON("example.com", "/temperature.php", jsonTemp);
  
  delay(60000);
}

/**************************************************************************/
/*!
    @brief  Displays the driver mode (tiny of normal), and the buffer
            size if tiny mode is not being used
    @note   The buffer size and driver mode are defined in cc3000_common.h
*/
/**************************************************************************/
void displayDriverMode(void)
{
  #ifdef CC3000_TINY_DRIVER
    Serial.println(F("CC3000 is configure in 'Tiny' mode"));
  #else
    Serial.print(F("RX Buffer : "));
    Serial.print(CC3000_RX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
    Serial.print(F("TX Buffer : "));
    Serial.print(CC3000_TX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
  #endif
}

/**************************************************************************/
/*!
    @brief  Tries to read the CC3000's internal firmware patch ID
*/
/**************************************************************************/
uint16_t checkFirmwareVersion(void)
{
  uint8_t major, minor;
  uint16_t version;
  
#ifndef CC3000_TINY_DRIVER  
  if(!cc3000.getFirmwareVersion(&major, &minor))
  {
    Serial.println(F("Unable to retrieve the firmware version!\r\n"));
    version = 0;
  }
  else
  {
    Serial.print(F("Firmware V. : "));
    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
    version = major; version <<= 8; version |= minor;
  }
#endif
  return version;
}

/**************************************************************************/
/*!
    @brief  Tries to read the 6-byte MAC address of the CC3000 module
*/
/**************************************************************************/
void displayMACAddress(void)
{
  uint8_t macAddress[6];
  
  if(!cc3000.getMacAddress(macAddress))
  {
    Serial.println(F("Unable to retrieve MAC Address!\r\n"));
  }
  else
  {
    Serial.print(F("MAC Address : "));
    cc3000.printHex((byte*)&macAddress, 6);
  }
}


/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

/**************************************************************************/
/*!
    @brief  Begins an SSID scan and prints out all the visible networks
*/
/**************************************************************************/

void listSSIDResults(void)
{
  uint32_t index;
  uint8_t valid, rssi, sec;
  char ssidname[33]; 

  if (!cc3000.startSSIDscan(&index)) {
    Serial.println(F("SSID scan failed!"));
    return;
  }

  Serial.print(F("Networks found: ")); Serial.println(index);
  Serial.println(F("================================================"));

  while (index) {
    index--;

    valid = cc3000.getNextSSID(&rssi, &sec, ssidname);
    
    Serial.print(F("SSID Name    : ")); Serial.print(ssidname);
    Serial.println();
    Serial.print(F("RSSI         : "));
    Serial.println(rssi);
    Serial.print(F("Security Mode: "));
    Serial.println(sec);
    Serial.println();
  }
  Serial.println(F("================================================"));

  cc3000.stopSSIDscan();
}

void sendJSON(char *address, char *page, char *data)
{
  ip = 0;
  // Try looking up the website's IP address
  Serial.print(address); Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(address, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    Serial.println("OK");
    delay(500);
  }
  
  /* Try connecting to the website.
     Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
  */
  Serial.println("Attempting TCP connection");
  client = cc3000.connectTCP(ip, 80);
  if (client.connected()) {
    client.fastrprint(F("GET "));
    client.fastrprint(page);
    client.fastrprint(data);
    client.fastrprint(F(" HTTP/1.1\r\n"));
    client.fastrprint(F("Host: "));
    client.fastrprint(address);
    client.fastrprint(F("\r\n"));
    client.fastrprint(F("\r\n"));
    
    Serial.print(F("OK\r\nAwaiting response..."));
    int c = 0;
    // Dirty trick: instead of parsing results, just look for opening
    // curly brace indicating the start of a successful JSON response.
    while(((c = timedRead()) > 0) && (c != '{'));
    if(c == '{')   Serial.println(F("success!"));
    else if(c < 0) Serial.println(F("timeout"));
    else           Serial.println(F("error (invalid credentials?)"));
    client.close();
    
  } else {
    Serial.println(F("Connection failed"));    
    return;
  }
  
}

// Read from client stream with a 5 second timeout.  Although an
// essentially identical method already exists in the Stream() class,
// it's declared private there...so this is a local copy.
int timedRead(void) {
  unsigned long start = millis();
  while((!client.available()) && ((millis() - start) < responseTimeout));
  return client.read();  // -1 on timeout
}

char specials[] = "$&+,/:;=?@ <>#%{}|~[]`"; ///* String containing chars you want encoded */

static char hex_digit(unsigned char c)
{
  return "01234567890ABCDEF"[c & 0x0F];
}

char *urlencode(char *dst,char *src)
{  
  char c,*d = dst;
  while (c = *src++)
  {  if (strchr(specials,c))
     {  *d++ = '%';
        *d++ = hex_digit(c >> 4);
        *d++ = hex_digit(c);
     }
     else *d++ = c;
  }
  return dst;
}

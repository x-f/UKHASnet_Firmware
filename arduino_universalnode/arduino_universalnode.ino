/*

UKHASnet Repeater Code by Phil Crump M0DNY

Based on UKHASnet rf69_repeater by James Coxon M6JCX

Modified by x-f
Universal code for different node types:
  - gateway/repeater node
  - solar repeater node
  - sensor node
*/

#include <SPI.h>
#include <string.h>
#include <RFM69Config.h>
#include <RFM69.h>
#include <LowPower.h>
#include "NodeConfig.h"

// Anarduino Flash memory
#ifdef ANARDUINO 
#include <SPIFlash.h>
#endif

//************* Misc Setup ****************/
#ifdef ENABLE_BATTV_SENSOR
 float battV=0.0;
#endif
#ifdef SENSOR_MODE
 float battV=0.0;
#endif
uint8_t n, i, j, k, packet_len;
uint32_t count = 1, data_interval = 2; // Initially send a couple of beacons in quick succession
uint8_t zombie_mode; // Stores current status: 0 - Full Repeating, 1 - Low Power shutdown, (beacon only)
uint8_t data_count = 97; // 'a'
char data[64], string_end[] = "]";
uint8_t buf[64], len;
int rx_rssi;

// Singleton instance of the radio
RFM69 rf69(RFM_SSpin); // parameter: RFM chip select pin


#ifdef ENABLE_RFM_TEMPERATURE
int8_t sampleRfmTemp() {
    int8_t rfmTemp = rf69.readTemp();
    while(rfmTemp>100) {
        rfmTemp = rf69.readTemp();
    }
    if(zombie_mode==0) {
        rfmTemp-=RX_TEMP_FUDGE;
    }
    return rfmTemp;
}
#endif

#ifdef ENABLE_BATTV_SENSOR
 float sampleBattv() {
   // External 4:1 Divider
   //int batv = analogRead(BATTV_PIN);
   //return ((float)batv*1.1*4*BATTV_FUDGE)/1023.0;
   
  float R1 = BATTV_R1;
  float R2 = BATTV_R2;
 
  //analogRead(BATTV_PIN);
  //delay(10);
  float value = 0;
  for (byte b = 0; b < 3; b++) {
    value += analogRead(BATTV_PIN);
    delay(5);
  }
  value += analogRead(BATTV_PIN);
  value /= 4; 
  float vout = (value * BATTV_AREF) / 1024.0;
  // Serial.print("val="); Serial.println(value);
  // Serial.print("V="); Serial.println(vout);
  //float vin = vout / (R2/(R1+R2));// - 0.04; 
  float vin = vout * (R1 + R2) / R2;
   if (vin < 0.9) {
     vin = 0.0;//statement to quash undesired reading !
   }
  return vin;
 }
#endif


#ifdef ENABLE_BMP085
  #include <Wire.h>

/*Based largely on code by  Jim Lindblom

  Get pressure, altitude, and temperature from the BMP085.
  Serial.print it out at 9600 baud to serial monitor.
*/

#define BMP085_ADDRESS 0x77  // I2C address of BMP085

const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 


// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature in deg C
float bmp085GetTemperature(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}

#endif


#ifdef ENABLE_DHT22
 #include "dht.h"
 dht DHT;
#endif

#ifdef ENABLE_DS18B20
 #include <OneWire.h>
 #include <DallasTemperature.h>
 //DeviceAddress ds_addr = { 0x28, 0xF7, 0x15, 0x05, 0x05, 0x00, 0x00, 0xF3 };
 //DeviceAddress ds_addr = { 0x28, 0xB7, 0x80, 0x8B, 0x02, 0x00, 0x00, 0x55 };
 OneWire ow(DS18B20_PIN);  // on pin PB1 (arduino: 9)
 DallasTemperature sensors(&ow);
#endif

#ifdef ENABLE_GPS
  #include <TinyGPS_UBX.h>
  TinyGPS gps;
  long gps_lat, gps_lon, gps_alt;
  unsigned long gps_fix_age;
  boolean gps_has_fix = false;
#endif



#ifdef ENABLE_ETHERNET

#include <Ethernet.h>
#include <EthernetUdp.h>

#define ETH_SS 10
// assign a MAC address for the ethernet controller.
// fill in your address here:
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// fill in an available IP address on your network here,
// for manual configuration:
IPAddress ip(10, 0, 1, 80);
// fill in your Domain Name Server address here:
IPAddress myDns(10, 0, 1, 1);
// initialize the library instance:
EthernetClient client;

unsigned int localPort = 8888;
IPAddress remoteIP(10, 0, 1, 255);
unsigned int remotePort = 8888;
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

#define ETH_CFG_MAX_MESSAGE_LEN 21
#define ETH_TX_MAX_MESSAGE_LEN 21
char eth_cfg[ETH_CFG_MAX_MESSAGE_LEN] = ""; // string for incoming serial data
char eth_TX_msg[ETH_TX_MAX_MESSAGE_LEN] = ""; // string to TX as comment
int stringPos = 0; // string index counter
boolean startRead = false; // is reading?


boolean ETH_use_TCP = true;
int ETH_cfg_updates = 3; // cfg upd every Xth loop

void eth_sendData_UDP() {
  digitalWrite(ETH_SS, LOW);

  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(data);
  Udp.endPacket();
  delay(5);
    
  digitalWrite(ETH_SS, HIGH);
}

void eth_sendData() {
  //long unsigned _ts_start = millis();
  
  digitalWrite(ETH_SS, LOW);
  //delay(100);
  
  byte content_len = 16 + packet_len;
  // sending rx'ed packet? add its RSSI
  //if (packet_type == 1)
  //  content_len += 9; // &rssi=000

  //Serial.print(F("S"));
  
  // try twice
  boolean connected = false;
  if (client.connect("212.93.98.119", 80)) {
    connected = true;
    //Serial.print(F("/1"));
  } else if (client.connect("x-f.lv", 80)) {
    connected = true;
    //Serial.print(F("/2"));
  }
  //if (client.connect("x-f.lv", 80)) {
  if (connected) {
    //Serial.print(F("/C"));
    client.println(F("POST /dev/sensenet/api/upload HTTP/1.1"));
    client.println(F("Host: x-f.lv"));
    client.println(F("Content-Type: application/x-www-form-urlencoded"));
    client.println(F("Connection: close"));
    client.print(F("Content-Length: "));
    client.println(content_len);
    client.println();
    client.print(F("origin="));
    client.print(id);
    client.print(F("&data="));
    client.print((char*)data);
    client.println();
    
    //Serial.println(data.length());
    //Serial.println();
    //Serial.print(data);
    //Serial.println();
    //delay(5);
    
  } //else
    //Serial.print(F("/failure"));

  client.stop();
  //client.flush();
  //Serial.print(F("/DC"));
 
  digitalWrite(ETH_SS, HIGH);
  
  //Serial.print(F("/time="));
  //Serial.print(millis() - _ts_start);
  //Serial.println();
}
// piesleedzas www serverim, nolasa configu
// {T=0/C=5/M=}
// T=1 - izmantot TCP
// C=5 - config updates every Xth loop
// M= - message?
// izmanto ik peec X reizeem vai 
void eth_readData() {
  //long unsigned _ts_start = millis();
  digitalWrite(ETH_SS, LOW);
  
  //Serial.print(F("R"));
  if (client.connect("x-f.lv", 80)) {
    //Serial.print(F("/C"));

    byte content_len = 16;

    client.println(F("POST /dev/sensenet/api/upload HTTP/1.1"));
    client.println(F("Host: x-f.lv"));
    client.println(F("Content-Type: application/x-www-form-urlencoded"));
    client.println(F("Connection: close"));
    client.print(F("Content-Length: "));
    client.println(content_len);
    client.println();
    client.print(F("origin="));
    client.print(id);
    client.print(F("&cfg=1"));
    client.println();
    
    delay(200);
    
    // TX message from the server
    stringPos = 0;
    memset(&eth_cfg, 0, ETH_CFG_MAX_MESSAGE_LEN); //clear cfg_msg memory
    memset(&eth_TX_msg, 0, ETH_TX_MAX_MESSAGE_LEN); //clear tx_msg memory

    unsigned long starttime = millis();
    startRead = false; 
    while (true) {
      if (client.available()) {
        char c = client.read();

        if (c == '{') { //'<' is our begining character
          startRead = true; //Ready to start reading the part 
        } else if (startRead) {  
          if (c != '}') { //'>' is our ending character
            if (stringPos < ETH_CFG_MAX_MESSAGE_LEN-1) {
              eth_cfg[stringPos] = c;
              stringPos++;
            } else {
              Serial.print(F("buff full="));
              //Serial.print("sp="); Serial.println(stringPos);
              eth_cfg[stringPos] = '\0';
              //Serial.println(eth_cfg);
              break;
            }
          } else {
            break;
          } // else
        }
      }
      
      if (millis() - starttime > 1000) {
        //Serial.print(F("/timeout"));
        break;
      }
    }

    //got what we need here! We can disconnect now
    startRead = false;
    if (strcmp(eth_cfg, "") != 0) {
      //Serial.print(F("got cfg: "));
      //Serial.println(eth_cfg);
      
      boolean got_comment = false;
      char comment_cnt = 0;
      for (k = 0; k < ETH_CFG_MAX_MESSAGE_LEN-1; k++) {

        // TCP
        if (eth_cfg[k] == 'T') {
          if (eth_cfg[k+2] == '1') {
            ETH_use_TCP = true;
            //Serial.print(F(" TCP="));Serial.print(1);
          }
          if (eth_cfg[k+2] == '0') {
            ETH_use_TCP = false;
            //Serial.print(F(" TCP="));Serial.print(0);
          }
        }
        // config updates every Xth loop
        if (eth_cfg[k] == 'C') {
          //Serial.print(">");Serial.print(eth_cfg[k+2]);Serial.print("<");
          //Serial.print(">");Serial.print(eth_cfg[k+2]-'0');Serial.println("<");
          if (eth_cfg[k+2]-'0' > 0 && eth_cfg[k+2]-'0' <= 9) {
            //Serial.print("u1="); Serial.println(ETH_cfg_updates);
            ETH_cfg_updates = eth_cfg[k+2]-'0';
            //Serial.print("u2="); Serial.println(ETH_cfg_updates);
          }
        }
        //if (eth_cfg[k] == 'M') {
        //  got_comment = true;
        //  k++;
        //}
        if (eth_cfg[k] == '\0') {
          //eth_TX_msg[comment_cnt] = '\0';
          break;
        }
        if (got_comment) {
          if (eth_cfg[k] != '=') {
            eth_TX_msg[comment_cnt] = eth_cfg[k];
            comment_cnt++;
          }
        }
      }
      
    }
    //Serial.print("cmnt>"); Serial.print(eth_TX_msg); Serial.println("<");

  } //else
    //Serial.print(F("/failure"));

  client.stop();
  client.flush();
  //Serial.println(F("/DC"));
 
  digitalWrite(ETH_SS, HIGH);

  //Serial.print(F("/time="));
  //Serial.print(millis() - _ts_start);
  //Serial.println();
}
#endif

uint8_t gen_Data() {

  #ifdef LOCATION_STRING
   if(data_count=='a' or data_count=='z') {
       sprintf(data, "%c%cL%s", num_repeats, data_count, LOCATION_STRING);
   } else {
       sprintf(data, "%c%c", num_repeats, data_count);
   }
  #else
   sprintf(data, "%c%c", num_repeats, data_count);
  #endif

  #ifdef ENABLE_GPS
    GPS_poll();
    gps.get_position(&gps_lat, &gps_lon, &gps_fix_age);
    gps_alt = gps.altitude();
    gps_has_fix = gps.has_fix();
    if (gps_has_fix) {
      char latbuf[10];
      dtostrf(gps_lat/100000.0, 7, 4, latbuf);
      char lonbuf[10];
      dtostrf(gps_lon/100000.0, 7, 4, lonbuf);
      char altbuf[10];
      dtostrf(gps_alt/100.0, 1, 0, altbuf);
  
      sprintf(data, "%sL%s,%s,%s", data, latbuf, lonbuf, altbuf);
    }
  #endif
  
  #ifdef ENABLE_RFM_TEMPERATURE 
    #ifndef ENABLE_DS18B20
      sprintf(data,"%sT%d",data,sampleRfmTemp());
    #endif
  #endif
  
  
  #ifdef ENABLE_BMP085
    digitalWrite(BMP085_PWR, HIGH);
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
    //Sleepy::loseSomeTime(100);
    //delay(100);
    float ftemp = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
    long pressure = bmp085GetPressure(bmp085ReadUP());
    digitalWrite(BMP085_PWR, LOW);

    char tempbuf[6];
    //convert float to string
    dtostrf(ftemp, 3, 1, tempbuf);

    #ifdef ENABLE_BMP085_TEMP
      sprintf(data, "%sT%s", data, tempbuf);
    #endif
    #ifdef ENABLE_BMP085_PRESSURE
      sprintf(data, "%sP%ld", data, pressure);
    #endif
  #endif
  
  
  #ifdef ENABLE_DHT22
   if (DHT.read22(DHT22_PIN) == DHTLIB_OK) {
     #ifdef ENABLE_DHT22_HUMIDITY
       char humbuf[5];
       dtostrf(DHT.humidity, 3, 1, humbuf);
       sprintf(data, "%sH%s", data, humbuf);
     #endif
     #ifdef ENABLE_DHT22_TEMP
       char tempbuf2[6];
       dtostrf(DHT.temperature, 3, 1, tempbuf2);
       sprintf(data, "%sT%s", data, tempbuf2);
     #endif
   }
  #endif
  
  #ifdef ENABLE_DS18B20
   //sensors.setWaitForConversion(false);
   digitalWrite(DS18B20_PWR, HIGH);
   LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
   sensors.requestTemperatures();
   float temp = sensors.getTempC(ds_addr);
   char tempbuf3[6];
   dtostrf(temp, 3, 1, tempbuf3);
   sprintf(data, "%sT%s", data, tempbuf3);
   #ifdef ENABLE_DS18B20_2
     float temp_2 = sensors.getTempC(ds_addr_2);
     char tempbuf4[6];
     dtostrf(temp_2, 3, 1, tempbuf4);
     sprintf(data, "%s,%s", data, tempbuf4);
   #endif
   digitalWrite(DS18B20_PWR, LOW);
  #endif

  #ifdef ENABLE_BATTV_SENSOR
   battV = sampleBattv();
   //char* battStr;
   //char tempStrB[14]; //make buffer large enough for 7 digits
   //battStr = dtostrf(battV,7,3,tempStrB);
   //while( (strlen(battStr) > 0) && (battStr[0] == 32) ) {
   //   strcpy(battStr,&battStr[1]);
   //}
   //sprintf(data, "%sV%s", data, battStr);
    if (battV > 0.1) {
      char battVbuf[6];
      // convert float to string
      dtostrf(battV, 4, 2, battVbuf);
      sprintf(data, "%sV%s", data, battVbuf);
    }
  #endif
    
  #ifdef ADVERTISE_ZOMBIE_MODE
  #ifdef ENABLE_ZOMBIE_MODE
  if (zombie_mode == 1) {
   sprintf(data, "%sZ%d", data, zombie_mode);
  }
  #endif
  #endif

  if (zombie_mode == 0 && rx_rssi != 0) {  
    sprintf(data,"%sR%d", data, rx_rssi);
  }

  return sprintf(data,"%s[%s]",data,id);
}




void setup() {

  #ifdef BATTV_INTERNALREF
    analogReference(INTERNAL); // 1.1V ADC reference
  #endif

  randomSeed(analogRead(6));

  #ifdef ENABLE_UART_OUTPUT
   Serial.begin(9600);
   //Serial.println("go");
  #endif
  
  // disable Anarduino Flash memory to save power
  // http://forum.anarduino.com/posts/list/39.page#198
  #ifdef ANARDUINO
    // put flash memory to sleep  
    // Anarduino: Flash SPI_CS = 5, ID = 0xEF30 (Winbond 4Mbit flash)
    SPIFlash flash(5, 0); // flash(SPI_CS, MANUFACTURER_ID)
    if (flash.initialize()) {
      //Serial.println("Flash Init OK!");
      flash.sleep();   // put flash (if it exists) into low power mode
      //Serial.println("Flash sleep");
    } else {
      //Serial.println("Flash Init FAIL!");
    }
  #endif
  
  #ifdef ENABLE_ETHERNET
    // Ethernet
    Serial.begin(9600);
    pinMode(ETH_SS, OUTPUT);  
    digitalWrite(ETH_SS, LOW);
    delay(100);
    // start the Ethernet connection using a fixed IP address and DNS server:
    Ethernet.begin(mac, ip, myDns);
    // print the Ethernet board/shield's IP address:
    Serial.print(F("My IP address: "));
    Serial.println(Ethernet.localIP());
    Udp.begin(localPort);
    delay(3000);
    digitalWrite(ETH_SS, HIGH);
    delay(100);
  #endif

  
  #ifdef ENABLE_BMP085  
    Wire.begin();
    pinMode(BMP085_PWR, OUTPUT);
    digitalWrite(BMP085_PWR, HIGH);
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
    //Sleepy::loseSomeTime(100);
    //delay(100);
    bmp085Calibration();
    digitalWrite(BMP085_PWR, LOW);
  #endif
  
  
  #ifdef ENABLE_DS18B20
   pinMode(DS18B20_PWR, OUTPUT);
   digitalWrite(DS18B20_PWR, HIGH);
   LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
   sensors.begin();
   //sensors.getAddress(ds_addr, 0);
   sensors.setResolution(ds_addr, 11);
   #ifdef ENABLE_DS18B20_2
     sensors.setResolution(ds_addr_2, 11);
   #endif
   digitalWrite(DS18B20_PWR, LOW);
  #endif

  #ifdef ENABLE_GPS
    Serial.begin(9600);
    delay(150);
    resetGPS();
    delay(500);
    GPS_setup();
  #endif

  #ifdef ENABLE_BATTV_SENSOR
    //ADC 0 - measure battery voltage
    pinMode(BATTV_PIN, INPUT);
    digitalWrite(BATTV_PIN, LOW);
  #endif
  
  //Serial.println("init..");
  while (!rf69.init()){
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
    //Serial.print(".");
  }
  //Serial.println("\ninit done");
  
  packet_len = gen_Data();
  rf69.send((uint8_t*)data, packet_len, rfm_power);
  #ifdef ENABLE_ETHERNET
    Serial.print("tx: "); Serial.println(data);
    eth_sendData_UDP();
    if (ETH_use_TCP)
      eth_sendData();
      
    //Serial.println("read cfg");
    eth_readData();
  #endif
  
  #ifdef ENABLE_ZOMBIE_MODE
   if(battV > ZOMBIE_THRESHOLD) {
     rf69.setMode(RFM69_MODE_RX);
     zombie_mode=0;
     #ifdef SENSITIVE_RX
      rf69.SetLnaMode(RF_TESTLNA_SENSITIVE);
     #endif
   } else {
     rf69.setMode(RFM69_MODE_SLEEP);
     zombie_mode=1;
   }
  #else
   rf69.setMode(RFM69_MODE_RX);
   zombie_mode=0;
   #ifdef SENSITIVE_RX
    rf69.SetLnaMode(RF_TESTLNA_SENSITIVE);
   #endif
  #endif
  
  #ifdef ENABLE_UART_OUTPUT
   // Print out own beacon packet
   for (j=0; j<packet_len; j++)
   {
     if(data[j]==']')
     {
       Serial.println(data[j]);
       break;
     }
     Serial.print(data[j]);
   }
  #endif
}

void loop()
{
  count++;
  
  if(zombie_mode==0) {
    rf69.setMode(RFM69_MODE_RX);
    
    for(i=0;i<255;i++) {
      LowPower.idle(SLEEP_30MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_ON, TWI_OFF);
      
      if (rf69.checkRx()) {

        memset(&buf, 0, 64);
        len = sizeof(buf);

        rx_rssi = rf69.lastRssi();
        
        rf69.recv(buf, &len);
        
        #ifdef ENABLE_UART_OUTPUT
         //rx_rssi = rf69.lastRssi();
         for (j=0; j<len; j++) {
             Serial.print((char)buf[j]);
             if(buf[j]==']') break;
         }
         Serial.print("|");
         Serial.println(rx_rssi);
        #endif
        
        #ifdef ENABLE_ETHERNET
          memset(&data, 0, 64);
          //strcpy(data, (char*)buf); //first copy buf to data (bytes to char)
          //Serial.print("e101: "); Serial.println((char*)buf);
          //Serial.print("e102: "); Serial.println(data);
          for (j=0; j<len; j++) {
            data[j] = ((char)buf[j]);
            if(buf[j]==']') break;
          }
          //rx_rssi = rf69.lastRssi();
          sprintf(data, "%s|%i", data, rx_rssi);
          //Serial.print("e11: "); Serial.println((char*)buf);
          Serial.print("rx: "); Serial.println(data);
          eth_sendData_UDP();
          if (ETH_use_TCP)
            eth_sendData();
        #endif

        // find end of packet & start of repeaters
        uint8_t end_bracket = -1, start_bracket = -1;        
        for (k=0; k<len; k++) {
          if (buf[k] == '[') {
            start_bracket = k;
          }
          else if (buf[k] == ']') {
            end_bracket = k;
            buf[k+1] = '\0';
            break;
          }
        }

        // Need to take the recieved buffer and decode it and add a reference 
        if (buf[0] > '0' && end_bracket != -1 && strstr((const char *)&buf[start_bracket], id) == NULL) {
          // Reduce the repeat value
          buf[0]--;
          
          // Add the repeater ID
          packet_len = end_bracket + sprintf((char *)&buf[end_bracket], ",%s]", id);

          //random delay to try and avoid packet collision
          // originally between 50 and 800 ms
          for (j = 0; j < random(1, 13); j++) {
            LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
          }
          
          rf69.send((uint8_t*)buf, packet_len, rfm_power);
          //#ifdef ENABLE_ETHERNET
          //  memset(&data, 0, 64);
          //  strcpy(data, (char*)buf); //first copy buf to data (bytes to char)
          //  //Serial.print("e21: "); Serial.println((char*)buf);
          //  Serial.print("tx: "); Serial.println(data);
          //  eth_sendData_UDP();
          // if (ETH_use_TCP)
          //  eth_sendData();
          //#endif

          #ifdef ENABLE_UART_OUTPUT
           // Print repeated packet
           for (j=0; j<packet_len; j++) {
               if(buf[j]==']'){
                  Serial.println((char)buf[j]);
                  break;
               }
               Serial.print((char)buf[j]);
           }
          #endif
        }
      }
    }
  } else {
    // Battery Voltage Low - Zombie Mode
    
    // Low Power Sleep for 8 seconds
    rf69.setMode(RFM69_MODE_SLEEP);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    //delay(8000);
  }
  
  if (count >= data_interval){
    data_count++;

    if(data_count > 122){
      data_count = 98; //'b'
    }
    
    packet_len = gen_Data();
    rf69.send((uint8_t*)data, packet_len, rfm_power);
    #ifdef ENABLE_ETHERNET
      Serial.print("tx: "); Serial.println(data);
      eth_sendData_UDP();
      if (ETH_use_TCP)
        eth_sendData();
      
      if (data_count % ETH_cfg_updates == 0) {
        //Serial.println("read cfg");
        eth_readData();
      }
    #endif

    #ifdef ENABLE_UART_OUTPUT
     // Print own Beacon Packet
     for (j=0; j<packet_len; j++)
     {
         if(data[j]==']') // Check for last char in packet
         {
             Serial.println(data[j]);
             break;
         }
         Serial.print(data[j]);
     }
    #endif
    
    data_interval = random((BEACON_INTERVAL/8), (BEACON_INTERVAL/8)+2) + count;
    #ifdef ENABLE_ZOMBIE_MODE
     if(battV > ZOMBIE_THRESHOLD && zombie_mode==1) {
         rf69.setMode(RFM69_MODE_RX);
         zombie_mode=0;
         #ifdef SENSITIVE_RX
          rf69.SetLnaMode(RF_TESTLNA_SENSITIVE);
         #endif
     } else if (battV < ZOMBIE_THRESHOLD && zombie_mode==0) {
         rf69.setMode(RFM69_MODE_SLEEP);
         zombie_mode=1;
     }
    #endif
  }
}







#ifdef ENABLE_GPS

void GPS_setup() {
  Serial.begin(9600);
  delay(100);

  Serial.flush();
  delay(100);
  
  // izslēdz visus GPS NMEA teikumus uBlox GPS modulim
  // ZDA, GLL, VTG, GSV, GSA, GGA, RMC
  // https://github.com/thecraag/craag-hab/blob/master/CRAAG1/code/CRAAG1c/CRAAG1c.ino
  // Turning off all GPS NMEA strings apart on the uBlox module
  // Taken from Project Swift (rather than the old way of sending ascii text)
  uint8_t setNMEAoff[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9};
  sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
  
  delay(100);
  Serial.flush();
  
  // airborne
  // ..
  // Set the navigation mode (Airborne, 1G)
  //Serial.print("Setting uBlox nav mode: ");
  //uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  //sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  //getUBX_ACK(setNav);
  //delay(500);

  // max performance mode
  //GPS_setMPM();
  //delay(500);

}

boolean GPS_poll() {
  //Poll GPS
  //Serial.flush();
  while(Serial.available()) Serial.read();
  Serial.println(F("$PUBX,00*33"));
  delay(100);

  unsigned long starttime = millis();
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (gps.encode(c))
        return true;
    }
    // 
    if (millis() - starttime > 1000) {
      #if DEBUG
        Serial.println(F("timeout"));
      #endif
      break;
    }
  }
  return false;
}

void resetGPS() {
  uint8_t set_reset[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5
  };
  //sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}

// http://ukhas.org.uk/guides:falcom_fsa03#sample_code
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for (int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}

#endif



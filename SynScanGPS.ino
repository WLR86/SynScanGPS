// SynScan GPS emulator using an Arduino with a GPS
// Copyright (C) 2014-2020 tazounet
// Revision by Weetos ©2025

#include <stdint.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// SynScan binary message
// User Position, Velocity & Time II (D1h)
struct BinaryMsg {
  uint16_t weekNo;
  uint32_t timeOfWeek;
  uint32_t date;
  uint32_t time;
  int32_t latitude;
  int32_t longitude;
  int16_t  altitude;
  uint16_t heading;
  uint16_t speed;
  uint8_t  fixIndicator;
  uint8_t  qualityOfFix;
  uint8_t  numberOfSv;
  uint8_t  numberOfSvInFix;
  uint8_t  gdop;
  uint8_t  pdop;
  uint8_t  hdop;
  uint8_t  vdop;
  uint8_t  tdop;
} __attribute__((packed));

static void synscanSendBinMsg(BinaryMsg *binMsg);

// GPS class
Adafruit_GPS gps(&Serial);

// SynScan connexion
#define RXPIN 8
#define TXPIN 9
SoftwareSerial nss(RXPIN, TXPIN);

// LED
#define LEDPIN 13

// Global State
bool sendBinaryMsg = false;

// Synscan command buffer
#define BUFF_SIZE 15
char synscanBuff[BUFF_SIZE] = {0};
uint8_t synscanBuffOffset = 0;

uint8_t uint2bcd(uint8_t ival)
{
   return ((ival / 10) << 4) | (ival % 10);
}

// Compute binary message checksum
static char computeChecksum(char* buf, uint16_t len)
{
  char chksum = 0;
 
  for (uint16_t i = 0; i < len; i++)
  {
    chksum ^= buf[i];
  }
  
  return chksum;
}

static void sendAck()
{
      char msg[] = {'\x25', '\x25', '\x06', '\x13', '\x15', '\x0D', '\x0A'};
      synscanSendMsg(msg, sizeof(msg));
}

static void synscanRead(char c)
{
  // Add byte to buffer
  if (synscanBuffOffset < BUFF_SIZE - 1)
  {
    synscanBuff[synscanBuffOffset] = c;
    synscanBuffOffset++;
  }

  if (c == '\n')
  {
    // End of command
    if (strncmp(synscanBuff, "\x25\x25\xf1\x13\x00\xe2\x0D\x0A", synscanBuffOffset) == 0)
    {
      // No output
      sendBinaryMsg = false;
      
      // Send ack
      sendAck();
    }
    else if (strncmp(synscanBuff, "\x25\x25\xF1\x13\x03\xE1\x0D\x0A", synscanBuffOffset) == 0)

    {
      // Binary output
      sendBinaryMsg = true;
      
      // Send ack
      sendAck();
    }
    
    // Clean buff
    synscanBuffOffset = 0;
  }
}

static void synscanSendBinMsg(BinaryMsg *binMsg)
{
  uint16_t size = 4 + sizeof(BinaryMsg);
  char msg[size];

  msg[0] = '\x25';
  msg[1] = '\x25';
  msg[2] = '\xF2';
  msg[3] = '\xD1';
  
  memcpy(msg + 4, binMsg, sizeof(BinaryMsg));

  for (uint16_t i = 0; i < size; i++)
  {
    nss.write(msg[i]);
  }
  
  // Checksum
  nss.write(computeChecksum(msg, size));
  
  // End
  nss.write('\x0D');
  nss.write('\x0A');
}

static void synscanSendMsg(char *msg, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    nss.write(msg[i]);
  }
}

//
// SETUP
//
void setup()
{
  // Init GPS connexion
  gps.begin(4800);
  
  // Init SynScan connexion
  nss.begin(4800);
  
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
}

//
// LOOP
//
void loop()
{
  // Read command from SynScan
  while (nss.available())
  {
    // Read synscan msg
    synscanRead(nss.read());
  }
 
  // Read GPS msg
  gps.read();

  if (gps.newNMEAreceived())
  {
    if (gps.parse(gps.lastNMEA()))
    {
      BinaryMsg binMsg = {0};
      uint8_t tmpBytes[4];
      
      binMsg.weekNo = 0; // not used
      binMsg.timeOfWeek = 0; // not used

      tmpBytes[0] = uint2bcd(gps.day);
      tmpBytes[1] = uint2bcd(gps.month);
      tmpBytes[2] = uint2bcd(gps.year);
      tmpBytes[3] = 0;
      memcpy(&(binMsg.date), tmpBytes, 4);

      tmpBytes[0] = uint2bcd(gps.seconds);
      tmpBytes[1] = uint2bcd(gps.minute);
      tmpBytes[2] = uint2bcd(gps.hour);
      tmpBytes[3] = 0;
      memcpy(&(binMsg.time), tmpBytes, 4);

      binMsg.latitude = (int32_t) (gps.latitudeDegrees / 360.0 * 4294967296);
      binMsg.longitude = (int32_t) (gps.longitudeDegrees / 360.0 * 4294967296);
      binMsg.altitude = (int16_t) gps.altitude;
      binMsg.heading = (uint16_t) gps.magvariation;
      binMsg.speed = (uint16_t) gps.speed;
      switch (gps.fixquality)
      {
        case 1 : binMsg.fixIndicator = 0; // GPS
        case 2 : binMsg.fixIndicator = 1; // DGPS
        default : binMsg.fixIndicator = 5; // Invalid
      }

      if (gps.fix) {
        // QualityOfFix : 1->2D 2->3D
        binMsg.qualityOfFix = 2 ;//gps.fixquality_3d - 1; // 2D fix or 3D fix
        binMsg.fixIndicator = 1; // GPS
        digitalWrite(LEDPIN, HIGH);
      } else {
        binMsg.qualityOfFix = 0; // no fix
        binMsg.fixIndicator = 5;
        digitalWrite(LEDPIN, LOW);
      }

      binMsg.numberOfSv = gps.satellites;
      binMsg.numberOfSvInFix = gps.satellites;
      binMsg.gdop = 1;
      binMsg.pdop = (uint8_t) gps.PDOP;
      binMsg.hdop = (uint8_t) gps.HDOP;
      binMsg.vdop = (uint8_t) gps.VDOP;
      binMsg.tdop = 1;
        
      // Synscan ask for GPS data
      if (sendBinaryMsg)
      {
        synscanSendBinMsg(&binMsg);
      }
    }
  }
}

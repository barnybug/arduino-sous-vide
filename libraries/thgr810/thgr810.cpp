// base class for simulating a THGR810 sensor
// sensors derive from this and use it to build and send the data packets

// the packet is built with the required THGR810 fields
// checksums and CRCs are calculated
// the data is manchester encoded and sent to the transmitter
// crc code from http://www.rajivchakravorty.com/source-code/uncertainty/multimedia-sim/html/crc8_8c-source.html

#include "Arduino.h"
#include "thgr810.h"

Thgr810::Thgr810(int channel, int transmitterPin, unsigned int code)
{
  m_channel = channel;
  m_madeTable = false;
  m_transmitterPin = transmitterPin;
  
  m_code = code;
  
  pinMode(m_transmitterPin, OUTPUT);

  initCrc8();
}

Thgr810::~Thgr810()
{
}

void Thgr810::transmit(float tempInC, int humidity)
{
  // Nibbles are sent LSB first

  // --- preamble ---
  // The preamble consists of twenty four '1' bits (6 nibbles) for v3.0 sensors
  m_packet[0] = 0xFF;
  m_packet[1] = 0xFF;
  m_packet[2] = 0xFF;

  // A sync nibble (4-bits) which is '0101'
  m_packet[3] = 0xA0;

  // --- payload ---
  // sending THGR810 packet
  // nibbles 0..3 Sensor ID This 16-bit value is unique to each sensor, or sometimes a group of sensors.
  m_packet[3] |= 0x0F;
  m_packet[4] = 0x82;
  m_packet[5] = 0x40;

  // nibble 4 Channel 1 thru 15
  m_packet[5] |= m_channel;

  // nibbles 5..6 code
  m_packet[6] = m_code >> 4 | m_code << 4;

  // nibble 7 Flags - battery status
  // my sensor is plugged in, so power is always good
  m_packet[7] = 0x80;

  // nibble 8..[n-5] Sensor-specific Data
  // nibbles 10..8 Temperature LSD is 0.1 degC

  // change a 23.5 C float value to 235 long value
  long temp = tempInC * 10;
  
  // if temperature is 23.5, then pull out the 5
  m_packet[7] |= (abs(temp) % 10) & 0x0F;

  // if temperature is 23.5, then pull out the 3
  m_packet[8] = ((abs(temp) / 10) % 10) << 4;

  // if temperature is 23.5, then pull out the 2
  m_packet[8] |= (abs(temp) / 100) & 0x0F;

  // nibble 11 Temperature Sign Non-zero for negative values
  m_packet[9] = (tempInC < 0.0) ? 0x80 : 0;

  // nibbles 15..12 is humidity, using this for low water height in inches 0-100"
  // the high distance measurement is actually the low water mark
  // I want to send a water height as a delta from some point
  // in this case, I'll just use 100" as sensor height which puts 0" at 8' 4" down from the dock
  // as my water level should never be lower than that
  
  m_packet[9] |= (humidity % 10) & 0x0F;
  m_packet[10] = ((humidity / 10) % 16) << 4;
  
  // nibbles [n-3]..[n-4] Checksum The 8-bit sum of nibbles 0..[n-5]
  m_packet[11] = calcChecksum();
  
  // CRC-8
  m_packet[12] = calcCrc();

  // send high value
  sendData();
}

unsigned char Thgr810::calcChecksum(void)
{
  unsigned char checksum;

  // does not include all of byte at index 3 (skip the sync pattern 0xA)
  checksum = (m_packet[3] & 0xF);

  // all of 4 - 10
  for (int i=4; i<=OSV3_THGR810_PACKET_LEN-3; i++)
  {
    checksum += (m_packet[i] & 0xF) + ((m_packet[i] >> 4)  & 0xF);
  }

  // nibble swap
  return ((checksum & 0x0F) << 4) | ((checksum & 0xF0) >> 4);
}


unsigned char Thgr810::calcCrc()
{
  unsigned char crc = 0;

  // does not include all of byte at index 3 (skip the sync pattern 0xA)
  crc8(&crc, m_packet[3] & 0x0F);

  // includes bytes at indicies 4 through 10, does not include checksum in index 11   
  for (int i=4; i<=10; i++)
  {
     crc8(&crc, m_packet[i]);
  }

  // nibble swap
  return ((crc & 0x0F) << 4) | ((crc & 0xF0) >> 4);;  
}

void Thgr810::sendData(void)
{
  int i;
  for (i=0; i < OSV3_THGR810_PACKET_LEN; i++)
  {
    Serial.print("Encoding ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(m_packet[i], HEX); 
  }
  
  digitalWrite(m_transmitterPin, LOW);
  delayMicroseconds(2000); 
    
  for (i=0; i < OSV3_THGR810_PACKET_LEN; i++)
  {
    manchesterEncode(m_packet[i], (i+1 == OSV3_THGR810_PACKET_LEN));
  }
  
  digitalWrite(m_transmitterPin, LOW);
}


void Thgr810::manchesterEncode(unsigned char encodeByte, bool lastByte)
{
  unsigned char mask = 0x10;
  int loopCount;
  static int lastbit = 0;
  static unsigned long baseMicros = micros();

  //  488us timing would be 1024 data rate
  // the data rate actually appears to be 1020 or 490us
  // if the timing is off, then the base station won't receive packets reliably
  const unsigned int desiredDelay = 490;

  // due to processing time, the delay shouldn't be a full desired delay time
  const unsigned int shorten = 32;
  
  // bits are transmitted in the order 4 thru 7, then 0 thru 3
  
  for (loopCount = 0; loopCount < 8; loopCount++)
  {  
    baseMicros += desiredDelay;
    unsigned long delayMicros = baseMicros - micros();
    
    if (delayMicros > 2*delayMicros)
    {
      // big delay indicates break between packet transmits, reset timing base
      baseMicros = micros();
    }
    else if (delayMicros > 0) 
    {
      delayMicroseconds(delayMicros); 
    }
    
    if ((encodeByte & mask) == 0)
    {
      // a zero bit is represented by an off-to-on transition in the RF signal
      digitalWrite(m_transmitterPin, LOW);
      delayMicroseconds(desiredDelay - shorten); 
      digitalWrite(m_transmitterPin, HIGH);
      
      // no long delay after last low to high transistion as no more data follows
      if (lastByte) delayMicroseconds(desiredDelay); 

      lastbit = 0;
    }
    else
    {
      digitalWrite(m_transmitterPin, HIGH);
      delayMicroseconds(desiredDelay - shorten); 
      digitalWrite(m_transmitterPin, LOW);
      lastbit = 1;
    }

    if (mask == 0x80)
    {
      mask = 0x01;
    }
    else
    {
      mask <<= 1;
    }

    baseMicros += desiredDelay;
  }
}  



// this CRC code is from crc8.c published by Rajiv Chakravorty
void Thgr810::initCrc8(void)
{
  int i,j;
  unsigned char crc;
  
  if (!m_madeTable) 
  {
    for (i=0; i<256; i++) 
    {
      crc = i;
      for (j=0; j<8; j++)
      {
        crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
      }
      
      m_crc8Table[i] = crc & 0xFF;
    }
    
    m_madeTable = true;
  }
}

void Thgr810::crc8(unsigned char *crc, unsigned char m)
{
  /*
   * For a byte array whose accumulated crc value is stored in *crc, computes
   * resultant crc obtained by appending m to the byte array
   */
  initCrc8();

  *crc = m_crc8Table[(*crc) ^ m];
  *crc &= 0xFF;
}

#ifndef __OSV3_THGR810_SENSOR_H_
#define __OSV3_THGR810_SENSOR_H_

// base class for simulating a THGR810 sensor
// sensors derive from this and use it to build and send the data packets
// class also contains the timing for each THGR810 channel

#define OSV3_THGR810_PACKET_LEN 13

// crc constants
#define GP  0x107   /* x^8 + x^2 + x + 1 */
#define DI  0x07

// can't nicely init array inside class... cheating and putting it here
const int m_secondsBetweenTransmits[] = {53, 59, 61, 67, 71, 79, 83, 87, 91, 93};

class Thgr810
{
  private:
    // channel effects packet tx frequency
    unsigned char m_packet[OSV3_THGR810_PACKET_LEN];
    int m_channel;
    unsigned int m_code;
 
    int m_transmitterPin;
 
    // having a CRC table in each sensor isn't the most efficient...
    unsigned char m_crc8Table[256];     /* 8-bit table */
    int m_madeTable;

    unsigned char calcChecksum(void);
    unsigned char calcCrc(void);
    void sendData(void);
    void manchesterEncode (unsigned char encodeByte, bool lastByte);
    
    void initCrc8(void);
    void crc8(unsigned char *crc, unsigned char m);

    
  public:
    Thgr810(int channel, int transmitterPin, unsigned int code);
    ~Thgr810();
    
    void transmit(float tempInC, int humidity);
  
};

#endif


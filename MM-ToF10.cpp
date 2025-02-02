/*
 *  MM-TOF10.cpp - MM-TOF10 Library
 *  Author Interested-In-Spresense
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "MM-TOF10.h"

#define mmtof10_DEBUG

#ifdef MMTOF10_DEBUG
#define mmtof10_printf printf
#else
#define mmtof10_printf(...) do {} while (0)
#endif

#define MMToF10_SYNC_BYTE 0xFF
#define BYTE_MASK         0xFF

#define MAGIC_HEADER      0xeb
#define MAGIC_TAILER      0xed

struct StSpiData {
  uint32_t  iRange1;
  uint16_t  wLight1;
  uint32_t  iRange[MMToF10_PIXELS];
  uint16_t  wLight[MMToF10_PIXELS];
  uint8_t  iStatus[MMToF10_PIXELS];
};



/*****************************************************************************/
/* Public                                                                    */
/*****************************************************************************/
void MMToF10Class::begin()
{
  m_mode = SyncMode;
  SPI5.begin();
  SPI5.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE3));
}

void MMToF10Class::end()
{
  SPI5.end();
}

void MMToF10Class::sleep()
{
  send(StandbyCommand, PowerStandby);
  delay(500);
}

void MMToF10Class::wakeup()
{
  send(StandbyCommand, PowerActive);
  delay(500);
}

void MMToF10Class::sync()
{
  setMode(SyncMode);
  skip(MMToF10_DATA_SIZE);
  skip();
}

void MMToF10Class::nomal(MMToF10_Distance dist, MMToF10_Rate rate)
{

  m_dist = dist;
  m_rate = rate;

  setMode(NomalMode);

  send(DistanceCommand, dist);
  delay(500);
  skip(MMToF10_DATA_SIZE);

  send(rate, 0);
  delay(500);
  skip(MMToF10_DATA_SIZE);

}

void MMToF10Class::search(uint8_t id)
{
  int data = SPI5.transfer(0) & BYTE_MASK;

  while (id != data) {
    data = SPI5.transfer(0);
  }
}

void MMToF10Class::skip(int cnt)
{
  while (cnt > 0) {
    SPI5.transfer(0);
    cnt--;
  }
}

void MMToF10Class::skip()
{
  int cnt = SPI5.transfer(0) & BYTE_MASK;

  while (cnt > 0) {
    SPI5.transfer(0);
    cnt--;
  }
}

void MMToF10Class::send(uint8_t cmd, uint8_t val)
{
  memset(buffer, 0, MMToF10_DATA_SIZE);
  buffer[0] = MAGIC_HEADER;
  buffer[1] = cmd;
  buffer[2] = 0x01;
  buffer[3] = val;
  buffer[4] = MAGIC_TAILER;

  SPI5.transfer(buffer, 5);
  skip(MMToF10_DATA_SIZE-5);
}

void MMToF10Class::led(uint8_t r, uint8_t g, uint8_t b)
{
    send(LedBlueCommand,b);
    send(LedRedCommand,r);
    send(LedGreenCommand,g);

}

/*****************************************************************************/
/* Private                                                                   */
/*****************************************************************************/
void MMToF10Class::setMode(MMToF10_Mode mode)
{
  m_mode = mode;
  skip(MMToF10_DATA_SIZE);
  delay(500);
  send(0, m_mode);
  delay(500);

}

float MMToF10Class::get1d()
{
  get_data();
  int32_t raw = ((buffer[4] << 24)|(buffer[5] << 16) | (buffer[6] << 8)| buffer[7]);

  float data = (((raw >> 22) & 0x1ff) * 1000) + (raw  & 0x3fffff) * 0.00023842;

  return data;
}

float MMToF10Class::get1p()
{
  get_data();
  uint16_t raw = ((buffer[8] << 8)| buffer[9]) / 0x10;

  float data = (raw >> 4) + (raw & 0xf) * 0.0625;
  return data;
}

void MMToF10Class::get3d(float* ptr)
{
  #define OFFSET_3D (4+4+2)
  get_data();
  for(int i=OFFSET_3D;i<(MMToF10_PIXELS*sizeof(uint32_t)+OFFSET_3D);i+=sizeof(uint32_t),ptr++){
    int32_t raw = ((buffer[i] << 24)|(buffer[i+1] << 16) | (buffer[i+2] << 8)| buffer[i+3]);
    *ptr = (((raw >> 22) & 0x1ff) * 1000) + (raw  & 0x3fffff) * 0.00023842;
  }
  
  return;
}

void MMToF10Class::get3p(uint16_t* ptr)
{
  get_data();
  for ( int i = 4*8*4+10; i < ((MMToF10_PIXELS*sizeof(uint16_t)+MMToF10_PIXELS*sizeof(uint32_t)+OFFSET_3D)); i += sizeof(uint16_t), ptr++ ) {
    *ptr = ((buffer[i] << 8)|buffer[i+1]) / 0x10;
  }
  return;
}

/*****************************************************************************/
/* Private                                                                   */
/*****************************************************************************/
bool MMToF10Class::check_magic(void)
{
  bool ret = false;
  if (buffer[0] != 0xe9){
    mmtof10_printf("magic error! %x\n",buffer[0]);
   printf("magic error! %x\n",buffer[0]);
   printf("magic error! %x\n",buffer[1]);
   printf("magic error! %x\n",buffer[2]);
   printf("magic error! %x\n",buffer[3]);
  }
  else {
    ret = true;
  }
  return ret;
}

bool MMToF10Class::check_sequence_id(void)
{
  bool ret = false;
  if (buffer[1] != buffer[255]){
    mmtof10_printf("sequence id error! %d,%d\n",buffer[1],buffer[255]);
  }
  else {
    ret = true;
  }
  return ret;
}

void MMToF10Class::get_data(void)
{
  bool result = false;
  while (1) {
    SPI5.transfer(buffer,MMToF10_DATA_SIZE);
    result = check_magic();
    if (!result) {
        sync();
        nomal(ShortDistance,LowSpeed);
      continue;
    }
    else {
      result = check_sequence_id();
      if (!result) {
        continue;
      }
      else {
        break;
      }
    }
  }
  return;
}


/* Pre-instantiated Object for this class */
MMToF10Class MMToF10;

/*
 *  MM-ToF10.h - MM-TOF10 Library Header
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

#include <SPI.h>

/*------------------------------------------------------------------*/
/* Definitions                                                      */
/*------------------------------------------------------------------*/

typedef enum e_MMToF10_Command {
  ModeCommand      = 0x00,
  LowSpeedCommand  = 0x10,
  HighSpeedCommand = 0x11,
  DistanceCommand  = 0x12,
  LedRedCommand    = 0xC0,
  LedGreenCommand  = 0xC1,
  LedBlueCommand   = 0xC2,
  StandbyCommand   = 0x80
} MMToF10_Command;

typedef enum e_MMToF10_Mode {
  NomalMode = 0x00,
  SyncMode  = 0xff
} MMToF10_Mode;

typedef enum e_MMToF10_Distance {
  ShortDistance = 0x00,
  LongDistance  = 0x01
} MMToF10_Distance;

typedef enum e_MMToF10_Rata {
  LowSpeed  = LowSpeedCommand,
  HighSpeed = HighSpeedCommand
} MMToF10_Rate;

typedef enum e_MMToF10_Power {
  PowerStandby = 0x01,
  PowerActive  = 0x00
} MMToF10_Power;


#define MMToF10_COLSIZE 8
#define MMToF10_ROWSIZE 4
#define MMToF10_PIXELS  (MMToF10_COLSIZE * MMToF10_ROWSIZE)

#define MMToF10_DATA_SIZE    256

/*------------------------------------------------------------------*/
/* Class                                                            */
/*------------------------------------------------------------------*/
class MMToF10Class{
  
public:
  void begin();
  void end();

  void sleep();
  void wakeup();

  void sync();
  void skip(int);
  void skip();
  void search(uint8_t);

  void nomal(MMToF10_Distance,MMToF10_Rate);
  void led(uint8_t,uint8_t,uint8_t);

  void send(uint8_t cmd, uint8_t val);

  float get(){ return get1d(); }
  float get1d();
  float get1p();
  void  get3d(float*);
  void  get3p(uint16_t*);

private:

  uint8_t buffer[MMToF10_DATA_SIZE];
  MMToF10_Mode m_mode;
  MMToF10_Distance m_dist;
  MMToF10_Rate m_rate;

  void setMode(MMToF10_Mode mode);
  MMToF10_Mode getMode(){ return m_mode; }

  bool check_magic(void);
  bool check_sequence_id(void);
  void get_data(void);

};

extern MMToF10Class MMToF10;

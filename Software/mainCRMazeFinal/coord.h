#ifndef COORD_H
#define COORD_H

#include "Arduino.h"
class Coord {
    uint8_t data;
  public:
    Coord(){data=0;}
    Coord (uint8_t x, uint8_t y){data = ((y & 0x0F) << 4) | (x & 0x0F);}
    uint8_t getX () {return (data & 0x0F);}
    uint8_t getY () {return (data & 0xF0) >> 4;}
    void setX (uint8_t x){data &= 0xF0; data |= (x & 0x0F);}
    void setY (uint8_t y){data &= 0x0F; data |= (y & 0x0F) << 4;}
};

#endif

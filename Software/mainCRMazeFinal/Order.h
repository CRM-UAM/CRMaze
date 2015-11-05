#ifndef ORDER_H
#define ORDER_H

#include "Arduino.h"
class Order {
    uint8_t data;
  public:
    Order(){data=0;}
    Order (uint8_t heading, uint8_t avance){data = ((avance & 0x0F) << 4) | (heading & 0x0F);}
    uint8_t getHeading () {return (data & 0x0F);}
    uint8_t getAvance () {return (data & 0xF0) >> 4;}
    void setHeading (uint8_t heading){data &= 0xF0; data |= (heading & 0x0F);}
    void setAvance (uint8_t avance){data &= 0x0F; data |= (avance & 0x0F) << 4;}
};

#endif

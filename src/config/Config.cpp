#include "config/Config.h"

const char DEFAULT_BLE_NAME[] = "RemoteButtons";

const uint8_t Config::DEFAULT_SHORT[8] = {
    '+', '-', 'n', 'c',
    KEY_UP_ARROW, KEY_LEFT_ARROW, KEY_RIGHT_ARROW, KEY_DOWN_ARROW};

const uint8_t Config::DEFAULT_LONG[8] = {
    0,   // btn1: repeat '+'
    0,   // btn2: repeat '-'
    'd', // btn3: long = 'd', short = 'n'
    0,   // btn4: reserved for config trigger (not configurable)
    0,   // btn5: repeat UP
    0,   // btn6: repeat LEFT
    0,   // btn7: repeat RIGHT
    0    // btn8: repeat DOWN
};

int Config::btnIndex(char key)
{
  if (key >= '1' && key <= '8')
    return key - '1';
  return -1;
}

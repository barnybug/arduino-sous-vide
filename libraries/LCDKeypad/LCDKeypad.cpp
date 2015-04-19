/*
  LCDKeypad.cpp
*/

#include "Arduino.h"

// include this library's description file

#include <LiquidCrystal.h>
#include "LCDKeypad.h"

LCDKeypad::LCDKeypad() : LiquidCrystal(8, 9, 4, 5, 6, 7)
{
}

int LCDKeypad::button()
{
  /*
  Older DFRobot shields measure:
  none 1023
  right 0
  up 99
  down 257
  left 409
  select 639
  Use:
  static int adc_key_val[5] ={  
    30, 150, 360, 535, 760     };  

  Newer DFRobot shields measure:
  none 1023
  right 0
  up 208
  down 411
  left 628
  select 828
  Use:
  static int adc_key_val[5] ={  
    30, 230, 500, 700, 950     };
   */
  static int NUM_KEYS=5;
  static int adc_key_val[5] ={  
    30, 230, 500, 700, 950     };
  int k, input;
  input=analogRead(0);
  for (k = 0; k < NUM_KEYS; k++)
  {
    if (input < adc_key_val[k])
    {
      return k;
    }
  }
  if (k >= NUM_KEYS)
    k = -1;     // No valid key pressed
  return k;
}

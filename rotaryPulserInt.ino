/* 
***************************************************************************  
**  Program  : rotaryPulserInt
**
**  Copyright (c) 2020 Willem Aandewiel
**
**  TERMS OF USE: MIT License. See bottom of file.                                                            
***************************************************************************      

* Pulsgever
* 
*   +--+     +--+
* A |  |     |  |
* --+  +-----+  +-----
* 
*     +--+     +--+
*   B |  |     |  |
* ----+  +-----+  +----
* 
* 
* Connect lineair potmeter 10K between 5v, GND and A0
* 
*   5V  -------+
*              |
*             +-+
*             | |
*             | |<--- A0
*             | |
*             +-+
*              |
*   GND -------+ 
*/

#define SET(a,b)      ((a) |= _BV(b))
#define CLEAR(a, b)   ((a) &= ~_BV(b))
#define SET_LOW(_port, _pin)    ((_port) &= ~_BV(_pin))
#define SET_HIGH(_port, _pin)   ((_port) |= _BV(_pin))


#define pinA    8     // PB0
#define pinB    9     // PB1
#define pinPot  A0    // first analog pin

// #define _HAS_OLED     // activate if you want to use an OLED screen
#define HARDWARE_I2C  // defines: HW I2C else software emulation I2C

#ifdef _HAS_OLED
  #include <Wire.h>  
  #include <U8g2lib.h>

  #ifdef HARDWARE_I2C
    #define pinSDA  21
    #define pinSCL  22
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, pinSCL, pinSDA, U8X8_PIN_NONE);   // hardware I2C
  #else // software I2C
    #define pinSDA  17
    #define pinSCL  16
    U8G2_SSD1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, pinSCL, /pinSDA, U8X8_PIN_NONE);  // pure SW emulated I2C
  #endif
#endif

uint32_t  frequency;
uint8_t   timerValue;
int       prevPotValue = 0, potValue = 0;
uint32_t  timerDuration;
uint8_t   steps;

volatile  int8_t  togglePin = 0;


//====================================================================
ISR(TIMER2_COMPA_vect)  //-- timer0 interrupt 8kHz toggles pin 8
{
  //--- generates pulse wave of frequency 2kHz/2 = 1kHz 
  //--- (takes two cycles for full wave- toggle high then toggle low)
  switch (togglePin)
  {
    case 0: //digitalWrite(pinA, HIGH);
            SET_HIGH(PORTB, 0); // PB0 = D8
            togglePin++;
            break;
    case 1: //digitalWrite(pinA, HIGH);
            SET_HIGH(PORTB, 1); // PB1 = D9
            togglePin++;
            break;
    case 2: //digitalWrite(pinA, LOW);
            SET_LOW(PORTB, 0); // PB0 = D8
            togglePin++;
            break;
    case 3: //digitalWrite(pinB, LOW);
            SET_LOW(PORTB, 1); // PB1 = D9
            togglePin = 0;
            break;
  }

} // ISR()


//====================================================================
void readPotmeter()
{
  potValue = analogRead(pinPot);
  //--- only real changes are processed
  if (potValue > (prevPotValue * 1.05) || potValue < (prevPotValue * 0.95))
  {
    Serial.print(F("potValue:\t"));
    Serial.print(potValue);
    frequency = map(potValue, 0, 1023, 16, 25000);
    Serial.print(F(" \tfreq:\t"));
    Serial.print(frequency);
    prevPotValue = potValue;
    updateTimer2();
    updateOLED();
  }
  
} // readPotmeter()


//====================================================================
void updateTimer2()
{
  if (frequency >= 2000)
  {
    Serial.print(F("\tfreq. >= 2000"));
    // steps  19 -> 25 kHz
    // steps 255 -> 1.9 Hz
    steps = map(frequency, 1900, 25000, 255, 19);
    Serial.print(F("\tsteps:\t")); Serial.println(steps);
    //--- prescaler 1/8
    setupTimer2(steps,  (1 << CS21)); 
  }
  else if (frequency >= 500)
  {
    Serial.print("\tfreq. >= 500");
    //-- 14 -> 2000 Hz 
    //-- 57 ->  500 Hz 
    steps = map(frequency, 500, 2000, 57, 14);
    Serial.print(F("\tsteps:\t")); Serial.println(steps);
    //--- prescaler 1/128
    setupTimer2(steps, (1 << CS22) | (1 << CS20));  
  }
  else
  {
    Serial.print(F("\tfreq. < 500"));
    //--   7  -> 490 Hz 
    //-- 250 ->   16 Hz 
    steps = map(frequency, 16, 500, 250, 7);
    Serial.print(F("\tsteps:\t")); Serial.println(steps);
    //--- prescaler 1/1024 
    setupTimer2(steps, (1 << CS22) | (1 << CS21) | (1 << CS20));       
  }
  
} // updateTimer2()


//====================================================================
void setupTimer2(uint8_t compaireMatch, uint8_t preScaler)
{
  cli(); //--- stop interrupts

  //--- set timer2 interrupt at 8kHz
  TCCR2A = 0; //--- set entire TCCR2A register to 0
  TCCR2B = 0; //--- same for TCCR2B
  TCNT2  = 0; //--- initialize counter value to 0
  //--- set compare match register for 8khz increments
  //>>OCR2A = 249;  // = (16*10^6) / (8000*8) - 1 (must be <256)
  OCR2A = compaireMatch;  // = (16*10^6) / (8000*8) - 1 (must be <256)
  //--- turn on CTC mode
  TCCR2A |= (1 << WGM21);
  
  //--- Set  prescaler -------------------------
  //--- | CS22 | CS21 | CS20 | Opmerking
  //--- |------+------+------+------------------
  //--- |  0   |  0   |  0   | no Clock!!!
  //--- |  0   |  0   |  1   | No Prescaler
  //--- |  0   |  1   |  0   | 1/8 Prescaler
  //--- |  0   |  1   |  1   | 1/32 Prescaler
  //--- |  1   |  0   |  0   | 1/64 Prescaler
  //--- |  1   |  0   |  1   | 1/128 Prescaler
  //--- |  1   |  1   |  0   | 1/256 Prescaler
  //--- |  1   |  1   |  1   | 1/1024 Prescaler
  //--- |------+------+------+------------------
  
  //--- 1/8 prescaler 19=25kHz, 255=1.95kHz -> lower than 19 will crash the processor
  //TCCR2B |= (1 << CS21);                
  
  //--- prescaler 1/32 timer 1=4uSec, 255=500uSec
  //TCCR2B |= (1 << CS21) | (1 << CS20); 
  
  //--- prescaler 1/64 timer 1=8uSec, 255=1.02mSec
  //TCCR2B |= (1 << CS22);
  
  //--- prescaler 1/128 timer 1=16uSec, 255=2.05mSec
  //TCCR2B |= (1 << CS22) | (1 << CS20);
  
  //--- prescaler 1/256 => timer 1=32uSec, 255=4100uSec
  //TCCR2B |= (1 << CS22) | (1 << CS21);
  
  //--- prescaler 1/1024 timer 1=128uSec, 255=16.3mSec
  //TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);

  TCCR2B |= preScaler;

  //--- enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();  //---allow interrupts

} // setupTimer2()


//====================================================================
void setup()
{
  Serial.begin(115200);
  while (!Serial) { /* wait */ }

  //--- initialize OLED library/display
  initOLED();

  pinMode(pinA,  OUTPUT);
  pinMode(pinB,  OUTPUT);
  pinMode(pinPot, INPUT);

  prevPotValue = analogRead(pinPot) + 100;
  readPotmeter();

  Serial.println(F("\nAnd then it begins ...\n"));
  
} // setup()


//====================================================================
void loop()
{
  if (Serial.available())
  {
    char x = Serial.read();
    if (x == '\n')
    {
      switch(frequency) {
        case 50:    frequency = 400;    break;
        case 400:   frequency = 500;    break;
        case 500:   frequency = 600;    break;
        case 600:   frequency = 1900;   break;
        case 1900:  frequency = 2000;   break;
        case 2000:  frequency = 2100;   break;
        case 2100:  frequency = 10000;  break;
        case 10000: frequency = 25000;  break;
        case 25000: frequency = 50;     break;
        default:    frequency = 50;
      }
      Serial.print(F(" \tfreq:\t"));
      Serial.print(frequency);
      timerDuration = (1000000 / frequency) / 4;
      Serial.print(F(" \ttimerDuration:\t"));
      Serial.print(timerDuration);
      updateTimer2();
      updateOLED();
    }
  }

  readPotmeter();

}



/***************************************************************************
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit
* persons to whom the Software is furnished to do so, subject to the
* following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
* OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
* THE USE OR OTHER DEALINGS IN THE SOFTWARE.
* 
****************************************************************************
*/

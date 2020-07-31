/* 
***************************************************************************  
**  rotaryPulserInt     
**
**  Versie datum: 31-07-2020
**  
**  This program is based on the Timer1 interrupt handeling (COMPA)
**  to generate interupts.
**  Every four interrupts generate a full cycle A and B pulse
**  that are 50% shifted from each other.
**  
**  Frequency can either be keyed in from a 4x4 keypad
**  or calculated from the position of a potmeter.
**
**  Copyright (c) 2020 Willem 
**
**  TERMS OF USE: MIT License. See bottom of file.                                                            
***************************************************************************      

* Pulsgever
* 
*      +---+   +---+   +--
*    A |   |   |   |   |        (11)
*    --+   +---+   +---+
*        +---+   +---+   +--
*      B |   |   |   |   |      (12)
*     ---+   +---+   +---+
* 
* Connections:
* 
*   KeyPad
*  ---------+               Arduino
*  top view |              --------- 
*           +-x nc         |       |
* [7] [*]   +------------> 2       0 ----> Tx
*           +------------> 3       1 <---- Rx 
* [8] [0]   +------------> 4       |
*           +------------> 5       4 <---> SDA
* [9] [#]   +------------> 6       5 <---> SCL
*           +------------> 7       |
* [C] [D]   +------------> 8       11 ---> Pulse A
*           +------------> 9       12 ---> Pulse B
*           +-x nc         |       |
*           |              |       A0 <--- Potmeter loper
*  ---------+              ---------
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

*
*/

#define SET(a,b)      ((a) |= _BV(b))
#define CLEAR(a, b)   ((a) &= ~_BV(b))
#define SET_LOW(_port, _pin)    ((_port) &= ~_BV(_pin))
#define SET_HIGH(_port, _pin)   ((_port) |= _BV(_pin))

#define pinA    11    // PB3
#define pinB    12    // PB4
#define pinPot  A0    // PC0

#define _CLOCK    16000000
#define _MAXFREQCHAR    20
#define _HYSTERESIS      5

#include <Wire.h>  
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2); 


#include <Keypad.h>

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {5, 4, 3, 2}; 
byte colPins[COLS] = {9, 8, 7, 6}; 

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

char      customKey;
char      newFrequencyChar[_MAXFREQCHAR];
int32_t   newFrequency, ledBuiltinTimer;
uint8_t   freqKeyPos = 0;
uint16_t  potValue, newPotValue;
bool      potmeterActive = false;

volatile int8_t  togglePin = 0;
volatile int32_t frequency;


//====================================================================
ISR(TIMER1_COMPA_vect)  //-- timer1 interrupt toggles pin 11 & 12
{
  //--- generates pulse wave of frequency 2kHz/2 = 1kHz 
  //--- (takes two cycles for full wave- toggle high then toggle low)
  switch (togglePin)
  {
    case 0: //digitalWrite(pinA, HIGH);
            SET_HIGH(PORTB, 3); // PB3 = 11
            togglePin++;
            break;
    case 1: //digitalWrite(pinA, HIGH);
            SET_HIGH(PORTB, 4); // PB4 = 12
            togglePin++;
            break;
    case 2: //digitalWrite(pinA, LOW);
            SET_LOW(PORTB, 3);  // PP3 = 11
            togglePin++;
            break;
    case 3: //digitalWrite(pinB, LOW);
            SET_LOW(PORTB, 4);  // PB4 = 12
            togglePin = 0;
            break;
  }

} // ISR()


//====================================================================
void initLCD()
{
  //--- initialize the library
  lcd.setCursor(0,0);
  lcd.print("JDengineers");
  lcd.setCursor(0,1);
  lcd.print("Encoder Pulse");
  
} // initOLED()

//====================================================================
void updateLCD()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("FREQUENCY: "); 
  lcd.print(frequency);
  lcd.setCursor(0,1);
  lcd.print("NEW FREQ.: "); 
  lcd.print(newFrequencyChar);
  
  Serial.print(F("FREQUENCY: ")); Serial.println(frequency);
  Serial.print(F("NEW FREQ.: ")); Serial.println(newFrequencyChar);
  
} // updateDisplay()


//====================================================================
void easterLCD()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(" (c) Willem "); 
  lcd.setCursor(0,1);
  lcd.print("Aandewiel"); 
  Serial.println(F("(c) Willem Aandewiel"));
  
} // updateDisplay()


//====================================================================
void uitleg()
{
  Serial.println(F("\r\nEnter frequency with [0]-[9] key's (in Hz)"));
  Serial.println(F("Enter [A] to accept new frequency"));
  Serial.println(F("Enter [C] to clear input"));
  Serial.println(F("Enter [*] + [A] for potmeter reading"));
  
} // uitleg()


//====================================================================
int32_t calculateTimer1(int32_t freqAsked, uint8_t &newTCCR1B)
{
  int32_t compareMatch;

  //-Serial.print(F("freqAsked[")); Serial.print(freqAsked); Serial.println(F("]"));
  //--- we need 4 interrupts for 1 cycle of A and B (4 state changes)
  freqAsked *= 4; 
  
  compareMatch = _CLOCK / (freqAsked*1) - 1;
  //-Serial.print(F("preScale 0 ->compareMatch is [")); Serial.print(compareMatch);
  if (compareMatch < 65536)
  {
    //-Serial.println(F("] < 65536 --> OK!"));
    //-Serial.flush();
    //--- Set CS12, CS11 and CS10 bits for 1 prescaler
    newTCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
    return compareMatch;
  }
  //-Serial.println();

  compareMatch = _CLOCK / (freqAsked*8) - 1;
  //-Serial.print(F("preScale 8 ->compareMatch is [")); Serial.print(compareMatch);
  if (compareMatch < 65536)
  {
    //-Serial.println(F("] < 65536 --> OK!"));
    //-Serial.flush();
    //--- Set CS12, CS11 and CS10 bits for 8 prescaler
    newTCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
    return compareMatch;
  }
  //-Serial.println();

  compareMatch = _CLOCK / (freqAsked*64) - 1;
  //-Serial.print(F("preScale 64 -> compareMatch is [")); Serial.print(compareMatch);
  if (compareMatch < 65536)
  {
    //-Serial.println(F("] < 65536 --> OK!"));
    //-Serial.flush();
    //--- Set CS12, CS11 and CS10 bits for 64 prescaler
    newTCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
    return compareMatch;
  }
  //-Serial.println();

  compareMatch = _CLOCK / (freqAsked*256) - 1;
  //-Serial.print(F("preScale 65536 ->compareMatch is [")); Serial.print(compareMatch);
  if (compareMatch < 65536)
  {
    //-Serial.println(F("] < 65536 --> OK!"));
    //-Serial.flush();
    //--- Set CS12, CS11 and CS10 bits for 256 prescaler
    newTCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
    return compareMatch;
  }
  //-Serial.println();

  //-Serial.println(F("] >= 65536 --> ERROR!"));
  //-Serial.flush();

} // calculateTimer1()


//====================================================================
void setupTimer1(int32_t newFrequency)
{
  uint8_t newTCCR1B = 0;
  
  cli(); //--- stop interrupts

  //--- clear Timer1 interrupt values
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0

  //---   calculate number of interrupts
  OCR1A = calculateTimer1(newFrequency, newTCCR1B);

  //--------- this is an example for a 1kHz interrupt --------------
  //-- set compare match register for 1000 Hz increments
  // OCR1A = 15999; // = 16000000 / (1 * 1000) - 1 (must be <65536)
  //-- turn on CTC mode
  //  TCCR1B |= (1 << WGM12);
  //-- Set CS12, CS11 and CS10 bits for 1 prescaler
  //  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  //-- enable timer compare interrupt
  //  TIMSK1 |= (1 << OCIE1A);
  //------------------ end of example ------------------------------
  
  //--- turn on CTC mode
  TCCR1B |= (1 << WGM12);
  
  //--- Set  prescaler -------------------------
  //--- | CS12 | CS11 | CS10 | Opmerking
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

  //--- add prescaler information from calculateTimer1()
  TCCR1B |= newTCCR1B;

  //--- enable timer compare interrupt from calculateTimer1()
  TIMSK1 |= (1 << OCIE1A);

  //--- update actual frequency
  frequency = newFrequency; 

  sei();  //--- allow interrupts

} // setupTimer1()


//====================================================================
void readPotmeter()
{
  volatile uint16_t potSav;
  
  if (!potmeterActive)  return;
  
  int16_t newPotValue = analogRead(pinPot);
  //--- only changes within hesteresis are processed
  if (newPotValue > (potValue + _HYSTERESIS))       potValue = newPotValue;  
  else if (newPotValue < (potValue - _HYSTERESIS))  potValue = newPotValue;  

  newFrequency = map(potValue, 0, 1024, 5, 25500);

  if (newFrequency < 10)      newFrequency =    10;
  if (newFrequency > 25000)   newFrequency = 25000;
  
  if (potSav != potValue)
  {
    Serial.print(F("Potmeter frequency: ")); Serial.println(newFrequency);
    setupTimer1(newFrequency);
    potSav = potValue;
    sprintf(newFrequencyChar, "potMeter");
    updateLCD();
  }

} // readPotmeter()


//====================================================================
void setup()
{
  Serial.begin(115200);
  while (!Serial) { /* wait */ }
  
  lcd.init();
  lcd.backlight();
  lcd.setBacklight(HIGH);
  initLCD();
  
  pinMode(pinA,        OUTPUT);
  pinMode(pinB,        OUTPUT);
  pinMode(pinPot,      INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  sprintf(newFrequencyChar, "%5d", 1000);
  newFrequency = 1000;

  potValue = analogRead(pinPot);
  
  Serial.println(F("\nAnd then it begins ...\n"));
  
  setupTimer1(newFrequency);

  delay(1000);

  uitleg();
    
} // setup()


//====================================================================
void loop()
{

  readPotmeter();
  
  customKey = customKeypad.getKey();
    
  if (Serial.available())
  {
    customKey = Serial.read();
  }
  
  switch (customKey)
  {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':   potmeterActive = false;
                if (freqKeyPos < _MAXFREQCHAR)
                {
                  newFrequencyChar[freqKeyPos++] = customKey;
                  newFrequencyChar[freqKeyPos] = 0;
                }
                updateLCD();
                break;
                
    case 'A':   Serial.print(F("\r\nfreqKeyed in is [")); Serial.print(newFrequencyChar); Serial.println(F("]"));
                Serial.flush();
                if (freqKeyPos == 0)  break;
                if (newFrequencyChar[0] == '*')  
                {
                  potmeterActive = true;
                  break;
                }
                potmeterActive = false;
                newFrequency = atoi(newFrequencyChar);
                if (newFrequency < 10) 
                {
                  newFrequency = 10;
                  sprintf(newFrequencyChar, "%5d", newFrequency);
                }
                if (newFrequency > 25000) 
                {
                  newFrequency = 25000;
                  sprintf(newFrequencyChar, "%5d", newFrequency);
                }
                setupTimer1(newFrequency);
                updateLCD();
                freqKeyPos = 0;
                newFrequencyChar[freqKeyPos] = 0;
                uitleg();
                break;
                
    case 'C':   Serial.println(F("input cancelled"));
                freqKeyPos = 0;
                newFrequencyChar[freqKeyPos] = 0;
                uitleg();
                break;

    case '#':
    case 'B':
    case '*':   //Serial.print(customKey);
                if (freqKeyPos < _MAXFREQCHAR)
                {
                  newFrequencyChar[freqKeyPos++] = customKey;
                  newFrequencyChar[freqKeyPos] = 0;
                }
                updateLCD();
                break;

    case 'D':   if (strcmp(newFrequencyChar, "*0#") == 0)
                {
                  easterLCD();
                  newFrequency = 1000;
                  setupTimer1(newFrequency);
                  freqKeyPos = 0;
                  newFrequencyChar[freqKeyPos] = 0;
                  break;
                }
                uitleg();
                break;
  } // switch

  if (millis() > ledBuiltinTimer)
  {
    ledBuiltinTimer = millis() + 2000;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  
} // loop()



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


//====================================================================
void initOLED()
{
#ifdef _HAS_OLED
  //--- initialize the library
  u8g2.begin();

  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_helvB08_tr); // choose a suitable font
  u8g2.drawStr(25, 10, "Willem");
  u8g2.drawStr(2, 30, "ENCODER SIMULATOR");
  u8g2.drawStr(2, 40, "FW: v1.2");
  u8g2.drawStr(2, 50, "D: 03072020");
  u8g2.drawStr(2, 60, "DEBUG Set 0-1");
  
  u8g2.sendBuffer();          // transfer internal memory to the display
#endif

} // initOLED()


//====================================================================
void debugOLED()
{
#ifdef _HAS_OLED
  u8g2.clearBuffer();         // clear the internal memory 
  u8g2.setCursor(64, 32);
  u8g2.setFont(u8g2_font_open_iconic_thing_6x_t);
  u8g2.print(char(77));       // pot
  u8g2.sendBuffer();          // transfer internal memory to the display
#endif

} // debugOLED()


//====================================================================
void updateOLED()
{
#ifdef _HAS_OLED
  //--- display frequency on OLED display
  //--- make use of:
  //---  > 'frequency'
  //---  > 'potValue'
  //--- You really don't need anything else
  u8g2.clearBuffer();          // clear the internal memory;
  u8g2.setCursor(50, 15);
  u8g2.setFont(u8g2_font_helvB10_tr); // choose a suitable font
  u8g2.print(frequency);
  u8g2.drawStr(110, 15, "Hz");   // put string of display at position X, Y
  u8g2.setCursor(10, 17);
  u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
  u8g2.print(char(70)); // pulse

  u8g2.setCursor(10, 40);
  u8g2.setFont(u8g2_font_open_iconic_app_2x_t);
  u8g2.print(char(72)); // pot
  u8g2.setFont(u8g2_font_helvB10_tr); // choose a suitable font
  u8g2.setCursor(50, 60);
  u8g2.print(potValue);
  u8g2.drawStr(95, 60, "MAP");   // put string of display at position X, Y
  u8g2.setCursor(10, 63);
  u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);
  u8g2.print(char(87)); // pot
  u8g2.sendBuffer();          // transfer internal memory to the display
#endif

} // updateDisplay()

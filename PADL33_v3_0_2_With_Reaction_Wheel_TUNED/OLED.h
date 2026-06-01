/************************************************
      MnSGC Ballooning PTERODACTYL Sketch
      Created by: Ashton Posey
      Date: 7/29/22
************************************************/
//Purpose: Class to control an OLED display

#define PIN_RESET 9
#define DC_JUMPER 1

MicroOLED oled(PIN_RESET, DC_JUMPER);    // I2C declaration

/************************************************************************************************************************************************
LED Class Definition
************************************************************************************************************************************************/
class OLED{
  public:
    void begin();
    void update(String disp);
  
  private:
    int _pin1;
    int _pin2;
};

/************************************************************************************************************************************************
LED Member Defenition
************************************************************************************************************************************************/
void OLED::begin(){ // this initializes the OLED
  //Wire.begin();
  oled.begin();
  oled.clear(ALL); // Clear the display's internal memory
  oled.display();  // Display what's in the buffer (splashscreen)
  oled.clear(PAGE); // Clear the buffer.

  randomSeed(analogRead(A0) + analogRead(A1));
  
}

void OLED::update(String disp){ //displays a string
  oled.clear(PAGE);
  oled.setFontType(0);
  oled.setCursor(0, 0);
  oled.println(disp);
  oled.display();
}
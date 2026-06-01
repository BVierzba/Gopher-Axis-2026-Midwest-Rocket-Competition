/************************************************
      MnSGC Ballooning PTERODACTYL Sketch
      Created by: Ashton Posey
      Date: 7/29/22
************************************************/
//Prpose: This sketch is a LED Class that will allow any number of LEDs
//         to be controlled
// Mostly taken from: https://youtu.be/S_uaROFnWSg

/***********************************************************************************************************************************************
LED Class Definition
************************************************************************************************************************************************/
class LED{
  public:
    LED(int pin);
    void begin();
    void on();
    void off();
    bool status();
  
  private:
    int _pin;
    bool _status;
    
};

/************************************************************************************************************************************************
LED Member Defenition
************************************************************************************************************************************************/
LED::LED(int pin){  // constructor
  _pin = pin; // here is where the pin you want to use is stored within the class DigitalLED
}

void LED::begin(){ // this initializes the pin
  pinMode(_pin, OUTPUT);
}

void LED::on(){ //this turns the pin on
  digitalWrite(_pin, HIGH);
  _status = 1;
}

void LED::off(){ // this turns the pin off
  digitalWrite(_pin, LOW);
  _status = 0;
}

bool LED::status(){ // this returns the 
  return _status;
}

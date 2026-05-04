/************************************************
      MnSGC Ballooning PTERODACTYL Sketch
      Created by: Ashton Posey
      Date: 7/29/22
/************************************************/
//Purpose: This Sketch will allow any number of Thermistors 
//         to be controlled frictionlessly

/***********************************************************************************************************************************************
Class Definition
************************************************************************************************************************************************/
class Thermistor{
  public:
    Thermistor(int pin);
    bool begin(int analogResolutionBits);
    bool updateStatus();
    bool getStatus();
    float getTempC();
    float getTempF();
    void update();
    
  private:
    int _pin;
    int _analogResolutionBits;
    float adcMax; // The maximum adc value given to the thermistor
    float A = 0.001125308852122;   // A, B, and C are constants used for a 10k resistor and 10k thermistor for the steinhart-hart equation
    float B = 0.000234711863267;
    float C = 0.000000085663516; 
    float R1 = 10000; // 10k Ω resistor
    float Tinv;
    float adcVal;
    float logR;
    float T; // these three variables are used for the calculation from adc value to temperature
    float currentTempC = 999; // The current temperature in Celcius
    float currentTempF = 999; // The current temperature in Fahrenheit
    bool _status = false;
};

/************************************************************************************************************************************************
Member Defenition
************************************************************************************************************************************************/
Thermistor::Thermistor(int pin){  // constructor
  _pin = pin; // here is where the pin you want to use is stored within the class DigitalLED
}

bool Thermistor::begin(int analogResolutionBits){ // this initializes the pin
  // analogReadResolution(analogResolutionBits);  // this value should be
  _analogResolutionBits = analogResolutionBits;
  update();
  return updateStatus();
}

bool Thermistor::updateStatus(){
  _status = (currentTempF > -80 && currentTempF < 160) ? true : false;
  return _status;
}

bool Thermistor::getStatus(){
  return _status;
}

float Thermistor::getTempC(){ // Sends the latest temperature but the output value is in Celsius
  return currentTempC;
}

float Thermistor::getTempF(){ // Sends the latest temperature but the output value is in Farenheit
  return currentTempF;
}

void Thermistor::update(){ // This fcation updates the temperature variables in the class.
  // analogReadResolution(_analogResolutionBits); // Tells the microcontroller how many bits to read when an analog read is sent
  adcMax = pow(2, _analogResolutionBits)-1.0;
  adcVal = analogRead(_pin); // Steinhart-Hart equation begin:
  logR = log(((adcMax/adcVal)-1)*R1);
  Tinv = A+B*logR+C*logR*logR*logR;
  T = 1/Tinv;
  currentTempC = T-273.15; // converting to celcius
  currentTempF = currentTempC*9/5+32;
}
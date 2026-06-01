/************************************************
      MnSGC Ballooning/Rocketry PADL-33 Flight Code
      Created by: Ashton Posey

      Modification Date: 5/8/226
      Modification by: Broc Vierzba
      Modifications:
        - 
************************************************/
//Purpose: Code for the PADL-33 Flight Computer
#define Version "Version 3.0"

#include <ReefwingLPS22HB.h>
#include <Arduino_HS300x.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include "DFRobot_BMM150.h"
#include <Arduino_APDS9960.h>
#include <SdFat.h>
#include <Wire.h>
//#include <SafeString.h>
#include <math.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <SparkFun_KX13X.h> //http://librarymanager/All#SparkFun_KX13X
#include <SFE_MicroOLED.h>

#include <PID_v1.h> // Used for PID logic
#include <Servo.h> // Used to control the RC motor

#include "OLED.h"
#include "Thermistor.h"
#include "LED.h"
#include "variables.h"

// VARIABLES TO EDIT FOR CONFIGURATION
#define GPS_FREQUENCY 10
#define GPS_DATA_DELAY 100 // milliseconds
#define DATA_DELAY 100 // milliseconds : can be different than GPS_DATA_DELAY if you want the gps to update slower than the rest of the sensors
#define GPS_BAUD 38400 // 38400 for M9N : 9600 for M8N

int photocellPin = 0;     // the cell and 10K pulldown are connected to a0
int photocellReading;     // the analog reading from the sensor divider
int LEDpin = 11;          // connect Red LED to pin 11 (PWM pin)
int LEDbrightness;        // 

/////////////////////////////// REACTION WHEEL VARIABLES START /////////////////////////////////
// UNTUNED //
double Kp = 0.05, Ki = 0.001, Kd = 0.01;

double setPoint = 0; // Target Angle
double PIDInput; // Current Angle
double PIDOutput; // Motor Speed

PID myPID(&PIDInput, &PIDOutput, &setPoint, Kp, Ki, Kd, DIRECT);

Servo esc;
const int MOTOR_PIN = 9; // Connect motor to D9
const int BUTTON_PIN = 2; // Connect button to d2, where the LED is.

int buttonState = HIGH;
int lastButtonState = HIGH;

const int MOTOR_MIN = 1000;
const int MOTOR_NEUT = 1500;
const int MOTOR_LOWEST = 1600;
const int MOTOR_CRUISE = 1750; 
const int MOTOR_MAX = 2000;

bool wheelSpin = false;
bool launched = false;
double GThreshold = 3.0; // H Motor: 7.86Gs. I Motor: 5.86Gs
int readingThreshold = 4; // Number of readings above GThreshold for rocket to be launched
int numOfReadings = 0;

enum rocketState {
                SPIN_UP_TRUE,
                WAITING_FOR_LAUNCH, 
                ROTATION_TO_90, 
                ROTATION_TO_0, 
                WAITING_FOR_DESCENT, 
                DESCENDING, 
                LANDED
                };
rocketState currentState = SPIN_UP_TRUE;
enum testState {
                SPIN_UP,
                TEST_TO_90,
                TEST_TO_0,
                TEST_OFF
                };
testState currentTest = TEST_TO_90;
unsigned long stateTimer = 0;
unsigned long holdTimer = 10000; // 10s, how long to stay at 90 degrees

double maxAlt = 0;
double startingAlt = 0;

// Variables for calculating the amount of rotation needed by the wheel
double currentAngle = 0;
unsigned long lastUpdate = 0;
double alpha = 0.98;

unsigned long prevBeep = 0;

bool buttonOF = false;
/////////////////////////////// REACTION WHEEL VARIABLES END /////////////////////////////

bool ecefEnabled = false; // bool to enable turning off ECEF for debugging
bool rtkEnabled = false; // bool to enable turing off RTK for debugging -> Must be using D9S Corrections Reciever
bool gpsI2C = false;      // bool to enable I2C when true, UART when false
bool usingBuzzer = false; // if this is false, the buzzer will never make a noise. 
bool usingOLED = false; // if false, OLED screen won't be used and setup will be faster.
bool usingLEDloop = true; // if false, LEDs will not light up after setup
bool usingLEDsetup = true; //if false, LEDs will not light up during setup
bool usingPhotoresistor = false; //if false, data from the photoresistor will not be collected

bool testingReactionWheel = false; // if true, wheel will test its back and forth, then the max speed
// END VARIABLES TO EDIT FOR CONFIGURATION

String header = "hh:mm:ss,FltTimer,T(s),T(ms),Hz,T2,T3,T4,T5,T6,totT,5v,VIN(V),HtrS,extT(F) or ADC,extT(C),intT(F),intT(C),Fix Type,RTK,PVT,Sats,Date,Time,Lat,Lon,Alt(Ft),Alt(M),HorizAccuracy(MM),VertAccuracy(MM),VertVel(Ft/S),VertVel(M/S),ECEFstat,ECEFX(M),ECEFY(M),ECEFZ(M),NedVelNorth(M/S),NedVelEast(M/S),NedVelDown(M/S),GndSpd(M/S),Head(Deg),PDOP,kPa,ATM,PSI,C,F,Ft,M,VV(Ft),VV(M),G(y),G(x),G(z),Deg/S(x),Deg/S(y),Deg/S(z),uT(x),uT(y),uT(z),kx mG(y),Current Angle,Phase,PID Output";

void setup() { //////////////////////////////////////////// SETUP ////////////////////////////////////////////
    systemSetUp();
    if (usingBuzzer){
        startUpJingle();
    }

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    esc.attach(MOTOR_PIN, MOTOR_MIN, MOTOR_MAX);
    esc.writeMicroseconds(MOTOR_NEUT); // Sets the motor to neutral
    // There should be a beep to confirm arming
    delay(2500);

    tone(TONE_PIN, 150, 200);
    delay(200);
    tone(TONE_PIN, 250, 200);
    delay(200);
    tone(TONE_PIN, 150, 200);
    
    // PID Setup
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-150, 250); // +/- 500 for the motor limits
    myPID.SetSampleTime(DATA_DELAY); // Sets the PID to update with the DATA_DELAY

} ///////////////////////////////////////////////////////// SETUP ////////////////////////////////////////////

void loop() { ///////////////////////////////////////////// LOOP /////////////////////////////////////////////

    if(millis() - timer >= DATA_DELAY){
        timer = millis();
        updateData();
        if (timer - prevBeep >= 10000) {
            prevBeep = timer;
            tone(TONE_PIN, 400, 200);
            lastButtonState = HIGH;
            buttonOF = !buttonOF;
        }
        if (!testingReactionWheel) {
            buttonState = digitalRead(BUTTON_PIN);
            if (buttonState == LOW && lastButtonState == HIGH && (currentState == SPIN_UP_TRUE || currentState == WAITING_FOR_LAUNCH)) {
                esc.writeMicroseconds(MOTOR_CRUISE);
                if (buttonOF == false) {
                    wheelSpin = true;
                    tone(TONE_PIN, 400, 200);
                    stateTimer = millis();
                }
                else {
                    wheelSpin = false;
                    tone(TONE_PIN, 200, 200);
                    currentState = SPIN_UP_TRUE;
                }
            }

            if (wheelSpin) {
                switch (currentState) {
                    case SPIN_UP_TRUE:
                        esc.writeMicroseconds(MOTOR_CRUISE);
                        if (millis() - stateTimer > 5) {
                            currentState = WAITING_FOR_LAUNCH;
                        }
                    break;
                    case WAITING_FOR_LAUNCH:
                        setPoint = 0.0;
                        if (abs(imu.data.accelX) >= GThreshold) {
                            numOfReadings++;
                            if (numOfReadings >= 1) {
                                tone(TONE_PIN, 400, 200);
                                setPoint = 90.0;
                                stateTimer = millis();
                                currentState = ROTATION_TO_90;
                            }
                        }
                        else {
                            numOfReadings = 0;
                            startingAlt = abs(pressureSensor[5]);
                        }
                    break;

                    case ROTATION_TO_90:
                        if (abs(PIDInput) < 1.0 && millis() - stateTimer > holdTimer) {
                            tone(TONE_PIN, 350, 200);
                            setPoint = 0.0;
                            stateTimer = millis();
                            currentState = ROTATION_TO_0;
                            numOfReadings = 0; // Resets numOfReadings for checking when the rocket is descending
                        }
                    break;

                    case ROTATION_TO_0:
                        if (abs(PIDInput) < 1.0 && millis() - stateTimer > holdTimer) {
                            tone(TONE_PIN, 300, 200);
                            currentState = WAITING_FOR_DESCENT;
                        }

                    break;

                    case WAITING_FOR_DESCENT:
                        if (pressureSensor[6] >= maxAlt) {
                            maxAlt = pressureSensor[6];
                            numOfReadings = 0;
                        }
                        else if (pressureSensor[6] + 1.0 < maxAlt) { // 1.0 is extra space for any readings that may be off
                            numOfReadings++;
                            if (numOfReadings >= readingThreshold) {
                                tone(TONE_PIN, 250, 200);
                                numOfReadings = 0;
                                setPoint = 0.0; // Largely redundent to set this again
                                currentState = DESCENDING;
                            }
                        }
                    break;

                    case DESCENDING:
                        if ((abs(pressureSensor[5]) - startingAlt) + 0.5 <= 0) {
                            numOfReadings++;
                            if (numOfReadings >= readingThreshold) {
                                tone(TONE_PIN, 200, 200);
                                currentState = LANDED;
                            }
                        }
                        else {
                            numOfReadings = 0;
                        }
                    break;

                    case LANDED:
                        esc.writeMicroseconds(MOTOR_NEUT);
                        PIDOutput = 0.0;
                    break;
                }
            }
        }
        else {
            buttonState = digitalRead(BUTTON_PIN);
            if (buttonState == LOW) {
                lastButtonState = LOW;
                tone(TONE_PIN, 400, 200);
                wheelSpin = true;
                stateTimer = millis();
                currentAngle = 0.0;
            }
            if (wheelSpin == true) {
                // esc.writeMicroseconds(MOTOR_CRUISE);
                switch (currentTest) {
                    case SPIN_UP:
                        esc.writeMicroseconds(MOTOR_CRUISE);
                        if (millis() - stateTimer > 5) {
                            currentTest = TEST_TO_90;
                        }
                    case TEST_TO_90:
                        setPoint = 0.0;
                        // if (currentAngle >= 88) {
                        //     currentAngle = 0;
                        //     currentTest = TEST_TO_0;f
                        //     tone(TONE_PIN, 1000, 200);
                        // }
                    break;

                    case TEST_TO_0:
                        setPoint = -90.0;
                        if (currentAngle <= -88) {
                            currentTest = TEST_OFF;
                        }
                    break;

                    case TEST_OFF:
                        tone(TONE_PIN, 600, 200);
                        stopMotor();
                    break;
                }
            }
        }

        // Must calculate the angle Y, as the gyro only gives degree/seconds, this can be done via integration
        unsigned long currentTime = millis();
        float dt = (currentTime - lastUpdate) / 1000.0;
        lastUpdate = currentTime;

        // Getting the acceleration angle will help with stability over time
        double accelerationAngleY = (atan2(imu.data.accelX, imu.data.accelZ) * 180) / PI;

        // Alpha keeps the acclerationAngleY small, but still allows it to influence, increasing accuracy over time
        currentAngle = alpha * (currentAngle + (imu.data.gyroY * dt)) + (1 - alpha) * (accelerationAngleY);

        currentAngle = constrain(currentAngle, -180, 180);
        if (buttonState == LOW) {
            currentAngle = 0;
        }

        PIDInput = currentAngle;
        myPID.Compute();

        if ((currentState != LANDED || testingReactionWheel == true) && wheelSpin == true) {
            driveMotor(PIDOutput);
        }
        else {
            stopMotor();
        }

    }
    if(usingPhotoresistor){
    photocellReading = analogRead(photocellPin);
    }
    // if (rtkEnabled){
    //   if (millis() - timer <= DATA_DELAY - 7){
    //       sparkFunGNSS.checkUblox(); // Check for the arrival of new GNSS data and process it.
    //       sparkFunGNSS.checkCallbacks(); // Check if any GNSS callbacks are waiting to be processed.
    //   }
    // }

} ///////////////////////////////////////////////////////// LOOP /////////////////////////////////////////////

///////// Functions ////////////
// void updateData();
// void driveMotor(double output);
// void stopMotor();

void updateData(){
    systemUpdate();

    data = flightTimer;
    data += ",";
    data += flightTimerString;
    data += ",";
    data += String(timerSec);
    data += ",";
    data += String(timer);
    data += ",";
    data += String(frequencyHz);
    data += ",";
    data += String(timer2);
    data += ",";
    data += String(timer3);
    data += ",";
    data += String(timer4);
    data += ",";
    data += String(timer5);
    data += ",";
    data += String(timer6);
    data += ",";
    data += String(timerTotal);
    data += ",";
    data += String(voltage_5v);
    data += ",";
    data += String(voltage_3v7);
    data += ",";
    data += String(heaterStatus);
    data += ",";

    if(thermExtOrADC){
        data += String(ThermistorExt.getTempF());
        data += ",";
        data += String(ThermistorExt.getTempC());
        data += ",";
    }
    else{
        data += String(photoresistorADCValue);
        data += ",";
        data += "0";
        data += ",";
    }

    data += String(ThermistorInt.getTempF());
    data += ",";
    data += String(ThermistorInt.getTempC());
    data += ",";
    data += fixTypeGPS;
    data += ",";
    data += fixTypeRTK;
    data += ",";
    data += String(PVTstatus);
    data += ",";
    data += String(SIV);
    data += ",";
    data += String(gpsMonth);
    data += "/";
    data += String(gpsDay);
    data += "/";
    data += String(gpsYear);
    data += ",";
    data += String(gpsHour);
    data += ":";
    data += String(gpsMinute);
    data += ":";
    data += String(gpsSecond);
    data += ".";

    if (gpsMillisecond < 10) {
        data += "00";
        data += String(gpsMillisecond);
        data += ",";
    }
    else if (gpsMillisecond < 100) {
        data += "0";
        data += String(gpsMillisecond);
        data += ",";
    }
    else{
        data += String(gpsMillisecond); 
        data += ",";
    }

    // data += lat_int;
    // data += ".";
    // data += lat_frac;
    // data += ",";
    // data += lon_int;
    // data += ".";
    // data += lon_frac;
    // data += ",";
    // data += f_ellipsoid*1000;
    // data += ",";
    // data += f_msl*1000;
    // data += ",";
    // data += f_accuracy*1000;
    // data += ",";

    char paddedNumber[8]; // Buffer to hold the padded number (7 digits + null terminator)
    data += String(gpsLatInt);
    data += ".";
    // Format the number with padded zeros using sprintf()
    sprintf(paddedNumber, "%07ld", gpsLatDec);
    data += String(paddedNumber); // Pad the number with zeros up to 7 digits
    // data += gpsLatDec;
    data += ",";

    data += String(gpsLonInt); 
    data += ".";
    // Format the number with padded zeros using sprintf()
    sprintf(paddedNumber, "%07ld", gpsLonDec);
    data += String(paddedNumber); // Pad the number with zeros up to 7 digits
    // data += gpsLonDec;
    data += ",";

    // data += gpsLatInt;
    // data += ".";
    // data += gpsLatDec;
    // data += ",";
    // data += gpsLonInt;
    // data += ".";
    // data += gpsLonDec;
    // data += ",";
    data += String(gpsAltFt);
    data += ",";
    data += String(gpsAltM);
    data += ",";
    data += String(gpsHorizAcc);
    data += ",";
    data += String(gpsVertAcc);
    data += ",";
    data += String(gpsVertVelFt);
    data += ",";
    data += String(gpsVertVelM);
    data += ",";
    data += String(ecefStatus);
    data += ",";
    data += String(ecefX);
    data += ",";
    data += String(ecefY); 
    data += ",";
    data += String(ecefZ);
    data += ","; 
    data += String(velocityNED[0]);
    data += ",";
    data += String(velocityNED[1]); 
    data += ",";
    data += String(velocityNED[2]);
    data += ","; 
    data += String(gpsGndSpeed);
    data += ",";
    data += String(gpsHeading);
    data += ",";
    data += String(gpsPDOP);
    data += ",";
    data += String(pressureSensor[0]);
    data += ",";
    data += String(pressureSensor[1]);
    data += ",";
    data += String(pressureSensor[2]);
    data += ",";
    data += String(pressureSensor[3]);
    data += ",";
    data += String(pressureSensor[4]);
    data += ",";
    data += String(pressureSensor[5]);
    data += ",";
    data += String(pressureSensor[6]);
    data += ",";
    data += String(vertVelFt);
    data += ",";
    data += String(vertVelM);
    data += ",";
    data += String(imu.data.accelX);
    data += ",";
    data += String(imu.data.accelY);
    data += ",";
    data += String(imu.data.accelZ);
    data += ",";
    data += String(imu.data.gyroX);
    data += ",";
    data += String(imu.data.gyroY);
    data += ",";
    data += String(imu.data.gyroZ);
    data += ",";
    data += String(magData.x);
    data += ",";
    data += String(magData.y);
    data += ",";
    data += String(magData.z);
    data += ",";
    data += String(kxData[0]);
    data += ",";
    data += String(currentAngle);
    data += ",";
    data += String(currentState);
    data += ",";
    data += String(PIDOutput);
  
   if (usingPhotoresistor){
      data += ",";
      data += String(photocellReading);
    }

    data += "\n";

    Serial.println(data);
    
    timer5 = millis() - timer7; ///////////// Timer 6 ///////////// 
    

    timer6 = millis(); ///////////// Timer 5 ///////////// 

    // float Ax2 = 0;  // Example floating-point number
    // int decimalPlaces = 2;         // Set the number of decimal places

    // // Multiply the number by 10^decimalPlaces, round it, then divide back
    // float factor = pow(10, decimalPlaces);  // Factor = 10^2 = 100 for 2 decimals
    // Ax2 = round(imu.data.accelX * factor) / factor;  // Round to 2 decimals

    // // Convert the result to an integer and display as string
    // int intPart = (int)Ax2;  // Get the integer part
    // int decPart = (int)((Ax2 - intPart) * factor);  // Get the decimal part

    // // Now format it into a string manually
    // char Ax[10];  // Make sure the buffer is large enough for the number
    // sprintf(Ax, "%d.%02d", intPart, decPart);  // Format to 2 decimal places

    if (usingOLED){
      float temp = ThermistorInt.getTempC();
      temp *= 100;
      temp = (int)round(temp);
      temp /= 100;
      int alt = pressureSensor[6];
      alt *= 1000;
      alt = round(alt);
      alt /= 1000;
      float pres = LPS22HB.readPressure(Units::PSI);
      pres *= 1000;
      pres = (int)round(pres);
      pres /= 1000;      
      String strOLED = "SD:" + String(sdStatus) + " GPS:" + String(bool(SIV)) +
      "H:" + String(pressureSensor[6]) + "ft" +
      String((int)round(imu.data.accelX)) + "x|" + String((int)round(imu.data.accelY)) + "y|"+ String((int)round(imu.data.accelZ)) + "z" + "\n" +
      String((int)round(imu.data.gyroX)) + "x|" + String((int)round(imu.data.gyroY)) + "y|"+ String((int)round(imu.data.gyroZ)) + "z" + "\n" +
      //"T:" + String(temp) + " C\n"
      "PID:" + String(PIDOutput) + "\n" +
      "State:" + String(currentState) + "\n";
      // "|T:" + String((int)round(ThermistorInt.getTempC())) + 
      // "G:" + String(gpsAltFt);
      OLED.update(strOLED);
    }

    dataAdded += data;
    if (dataCounter == 3){
        if (SD.exists(filename)){
            datalog.print(dataAdded);
            datalog.flush();
        }
        else{
            if (usingBuzzer) tone(TONE_PIN, 400);
            Serial.println("NO SD");
        }
        // datalog.print(dataAdded);
        // datalog.flush();
        dataAdded = "";
    }
    dataCounter++;
    if (dataCounter == 4) dataCounter = 0;

    if (usingXBee)  XBee.println(String(pressureSensor[5]));

    timer6 = millis() - timer6; /////////////////////////

    timerTotal = timer2 + timer3 + timer4 + timer5 + timer6;
}

void driveMotor(double output) {

        if ((int)output >= 10 && (int)output <= -10) {
            CENTER_LED.on();
        }
        else if ((int)output < -10) {
            LEFT_LED.on();
        }
        else {
            RIGHT_LED.on();
        }

    int pulseWidth = MOTOR_CRUISE + (int)output;

    pulseWidth = constrain(pulseWidth, MOTOR_LOWEST, MOTOR_MAX);

    esc.writeMicroseconds(pulseWidth);
}

void stopMotor() {
    esc.writeMicroseconds(MOTOR_NEUT);
    PIDOutput = 0.0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////


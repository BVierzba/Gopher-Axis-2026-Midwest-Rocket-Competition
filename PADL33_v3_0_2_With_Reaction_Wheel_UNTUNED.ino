/************************************************
      MnSGC Ballooning/Rocketry PADL-33 Flight Code
      Created by: Ashton Posey

      Modification Date: 4/6/226
      Modification by: Broc Vierzba
      Modifications:
        - Adjusted OLED screen to provide gyro data and PID information
        - Adjusted rocketState logic
        - Increased the startup speed
        - Re-did PIDInput logic to actually work
        - Changed Kp to 1.0 so PIDOutput gave an output
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

/////////////////////////////// PID VARIABLES /////////////////////////////////
// UNTUNED //
double Kp = 1.0, Ki = 0.0, Kd = 0.0;

double setPoint = 0; // Target Angle
double PIDInput; // Current Angle
double PIDOutput; // Motor Speed

PID myPID(&PIDInput, &PIDOutput, &setPoint, Kp, Ki, Kd, DIRECT);

Servo esc;
const int MOTOR_PIN = 9; // Connect motor to D9

const int MOTOR_MIN = 1000;
const int MOTOR_NEUT = 1500;
const int MOTOR_MAX = 2000;

bool launched = false;
double GThreshold = 3.0; // H Motor: 7.86Gs. I Motor: 5.86Gs
int readingThreshold = 4; // Number of readings above GThreshold for rocket to be launched
int numOfReadings = 0;

enum rocketState {
                WAITING_FOR_LAUNCH, 
                ROTATION_TO_90, 
                ROTATION_TO_0, 
                WAITING_FOR_DESCENT, 
                DESCENDING, 
                LANDED
                };
rocketState currentstate = WAITING_FOR_LAUNCH;
unsigned long stateTimer = 0;
unsigned long holdTimer = 10000; // 10s, how long to stay at 90 degrees

double maxAlt = 0;
double startingAlt = 0;

// Variables for calculating the amount of rotation needed by the wheel
double currentAngle = 0;
unsigned long lastUpdate = 0;
double alpha = 0.98;

unsigned long prevBeep = 0;
/////////////////////////////// PID VARIABLES END /////////////////////////////

bool ecefEnabled = false; // bool to enable turning off ECEF for debugging
bool rtkEnabled = false; // bool to enable turing off RTK for debugging -> Must be using D9S Corrections Reciever
bool gpsI2C = false;      // bool to enable I2C when true, UART when false
bool usingBuzzer = false; // if this is false, the buzzer will never make a noise. 
bool usingOLED = true; // if false, OLED screen won't be used and setup will be faster.
bool usingLEDloop = true; // if false, LEDs will not light up after setup
bool usingLEDsetup = true; //if false, LEDs will not light up during setup
bool usingPhotoresistor = false; //if false, data from the photoresistor will not be collected

bool testingReactionWheel = true; // if true, wheel will test its back and forth, then the max speed
// END VARIABLES TO EDIT FOR CONFIGURATION

String header = "hh:mm:ss,FltTimer,T(s),T(ms),Hz,T2,T3,T4,T5,T6,totT,5v,VIN(V),HtrS,extT(F) or ADC,extT(C),intT(F),intT(C),Fix Type,RTK,PVT,Sats,Date,Time,Lat,Lon,Alt(Ft),Alt(M),HorizAccuracy(MM),VertAccuracy(MM),VertVel(Ft/S),VertVel(M/S),ECEFstat,ECEFX(M),ECEFY(M),ECEFZ(M),NedVelNorth(M/S),NedVelEast(M/S),NedVelDown(M/S),GndSpd(M/S),Head(Deg),PDOP,kPa,ATM,PSI,C,F,Ft,M,VV(Ft),VV(M),G(y),G(x),G(z),Deg/S(x),Deg/S(y),Deg/S(z),uT(x),uT(y),uT(z),kx mG(y),mG(x),mG(z),GPS I2C?,Photoresistor";

void setup() { //////////////////////////////////////////// SETUP ////////////////////////////////////////////
    systemSetUp();
    if (usingBuzzer){
        startUpJingle();
    }

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
    myPID.SetOutputLimits(-500, 500); // +/- 500 for the motor limits
    myPID.SetSampleTime(DATA_DELAY); // Sets the PID to update with the DATA_DELAY

} ///////////////////////////////////////////////////////// SETUP ////////////////////////////////////////////

void loop() { ///////////////////////////////////////////// LOOP /////////////////////////////////////////////

    if(millis() - timer >= DATA_DELAY){
        timer = millis();
        updateData();
        if (timer - prevBeep >= 10000) {
            prevBeep = timer;
            tone(TONE_PIN, 400, 200);
        }
        if (!testingReactionWheel) {
            switch (currentstate) {

                case WAITING_FOR_LAUNCH:
                    if (imu.data.accelZ > GThreshold) {
                        numOfReadings++;
                        if (numOfReadings >= readingThreshold) {
                            setPoint = 90.0;
                            stateTimer = millis();
                            currentstate = ROTATION_TO_90;
                        }
                    }
                    else {
                        numOfReadings = 0;
                        startingAlt = abs(gpsVertVelFt);
                    }
                break;

                case ROTATION_TO_90:
                    RED_LED.on();
                    if (abs(PIDInput) < 1.0 && millis() - stateTimer > holdTimer) {
                        setPoint = 0.0;
                        stateTimer = millis();
                        currentstate = ROTATION_TO_0;
                        numOfReadings = 0; // Resets numOfReadings for checking when the rocket is descending
                    }
                break;

                case ROTATION_TO_0:
                    RED_LED.off();
                    BLUE_LED.on();
                    if (abs(PIDInput) < 1.0 && millis() - stateTimer > holdTimer) {
                        currentstate = WAITING_FOR_DESCENT;
                    }

                break;

                case WAITING_FOR_DESCENT:
                    BLUE_LED.off();
                    GREEN_LED.on();
                    if (pressureSensor[6] >= maxAlt) {
                        maxAlt = pressureSensor[6];
                        numOfReadings = 0;
                    }
                    else if (pressureSensor[6] + 1.0 < maxAlt) { // 1.0 is extra space for any readings that may be off
                        numOfReadings++;
                        if (numOfReadings >= readingThreshold) {
                            numOfReadings = 0;
                            setPoint = 0.0; // Largely redundent to set this again
                            currentstate = DESCENDING;
                        }
                    }
                break;

                case DESCENDING:
                    GREEN_LED.off();
                    RED_LED.on();
                    BLUE_LED.on();
                    if ((abs(gpsVertVelFt) - startingAlt) + 0.5 <= 0) {
                        numOfReadings++;
                        if (numOfReadings >= readingThreshold) {
                            currentstate = LANDED;
                        }
                    }
                    else {
                        numOfReadings = 0;
                    }
                    currentstate = LANDED;
                break;

                case LANDED:
                    GREEN_LED.on();
                    PIDOutput = 0.0;
                break;
            }
        }
        else {
            // Spins both directions at 1/5th power
            delay(5000);
            esc.write(MOTOR_NEUT + 100);
            delay(5000);
            esc.write(MOTOR_NEUT - 100);
            delay(5000);

            // Slowly spins up to max speed over 25 seconds
            for (int i = 0; i < 500; i += 10) {
                esc.write(MOTOR_NEUT + i);
                delay(500);
            }
        }
        
        // Must calculate the angle Y, as the gyro only gives degree/seconds, this can be done via integration
        unsigned long currentTime = millis();
        float dt = (currentTime - lastUpdate) / 1000.0;
        lastUpdate = currentTime;

        // Getting the acceleration angle will help with stability over time
        double accelerationAngleY = atan2(imu.data.accelX, imu.data.accelZ) * 180 / PI;

        // Alpha keeps the acclerationAngleY small, but still allows it to influence, increasing accuracy over time
        currentAngle = alpha * (currentAngle + (imu.data.gyroY * dt)) + (1 - alpha) * (accelerationAngleY);

        PIDInput = currentAngle;
        myPID.Compute();

        if (currentstate != WAITING_FOR_LAUNCH && currentstate != LANDED) {
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
    data += String(kxData[1]);
    data += ",";
    data += String(kxData[2]);
    data += ",";
    data += String(gpsI2C);
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
      "State:" + String(currentstate) + "\n";
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
    int pulseWidth = MOTOR_NEUT + (int)output;

    pulseWidth = constrain(pulseWidth, MOTOR_MIN, MOTOR_MAX);

    esc.writeMicroseconds(pulseWidth);
}

void stopMotor() {
    esc.writeMicroseconds(MOTOR_NEUT);
    PIDOutput = 0.0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////


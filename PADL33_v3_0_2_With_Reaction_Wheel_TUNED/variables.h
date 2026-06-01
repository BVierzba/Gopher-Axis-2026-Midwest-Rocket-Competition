/************************************************
      PADL-33 Global Variables
      Created by: Ashton Posey
************************************************/
//Purpose: Global variables for PADL-33

#define SERIAL_BAUD 500000
#define I2C_CLOCK 400000
#define VOLTAGE_READER_PIN_5V A7
#define VOLTAGE_READER_PIN_3V7 A6
#define EXTERNAL_THERMISTOR_PIN A3
#define INTERNAL_THERMISTOR_PIN A2
#define CHIP_SELECT 10 //Should highlight if you have teensy 3.5/3.6/4.0/4.1 selected
#define HEATER_PIN 8
#define TONE_PIN 6
#define YELLOW_LED_PIN 100 // Data Update LED
#define GREEN_LED_PIN 3
#define BLUE_LED_PIN 4 // Satellites is greater than 0 - GPS is Locked
#define RED_LED_PIN 5

#define OK(ok) (ok ? F("  ->  OK") : F("  ->  ERROR!")) // Convert uint8_t into OK/ERROR

LED YELLOW_LED(YELLOW_LED_PIN);
LED LEFT_LED(GREEN_LED_PIN);
LED CENTER_LED(BLUE_LED_PIN);
LED RIGHT_LED(RED_LED_PIN);

byte greenLedCounter = 0;

float voltage_5v = 999;
float voltage_3v7 = 999;

bool usingKX = false;
//SparkFun_KX132 kxAccel;
SparkFun_KX134 kxAccel; // For the KX134, uncomment this and comment line above
outputData myData; // Struct for the accelerometer's data
bool kxStatus = false;
double kxData[3] = {999, 999, 999};

float kxDataStatus[6] = {999, 998, 997, 996, 995, 994};
int kxDataStatusTimer = 0;

bool heaterStatus = false;
int maxTemp = 87.0;
int minTemp = 85.0;

//createSafeString(data, 700); // Full data string
//createSafeString(dataAdded, 2800); // create an empty string called dataAdded with a mutiple of data
String data;
String dataAdded;
byte dataCounter = 0;
//createSafeString(flightTimer, 10);
//createSafeString(strTimer, 40);
String flightTimer;
String strTimer;

long photoresistorADCValue = 0;

bool thermExtOrADC = true; // true means you're using an external thermistor
Thermistor ThermistorExt(EXTERNAL_THERMISTOR_PIN);
Thermistor ThermistorInt(INTERNAL_THERMISTOR_PIN);

// SdFs SD;
// FsFile datalog;
SdFat SD;
File datalog;
// SdFat32 SD;
// File32 datalog;
// SdExFat SD;
// ExFile datalog;
//File datalog; // Constructed object that handles SD input and output, can input/output i.e read/write for multiple files
char filename[ ] = "SDCARD00.csv"; //Make sure dataFileN matches first and second place of the zeros in terms of the arrays index
String directory = "Data";
const byte dataFileN1 = 6; 
const byte dataFileN2 = 7; 
bool sdStatus = false; // Status of datalog.begin();   
bool sdActive = false; // Status of if there are available file names i.e the SD could be full from files 0-99 

String _print;

unsigned long timer = 0; // millis()/1000
float timerSec = 999;

double pressureBoundary1;
double pressureBoundary2;
double pressureBoundary3;
float h1 = 36152.0;
float h2 = 82345.0;
float T1;
float T2;
float T3;

bool pressureStatus = 0;

float pressureSensor[7] = {999, 999, 999, 999, 999, 999, 999};
//float pressureKPA = 999, pressureATM = 999, pressurePSI = 999, temperatureC = 999, temperatureF = 999, altitudeFt = 999, altitudeM = 999;
float vertVelFt = 0;
float vertVelM = 0;
float msPrevAltFt = 0;
float msPrevTime = 0;

float pressureSensorStatus[6] = {999, 998, 997, 996, 995, 994};
int pressureSensorStatusTimer = 0;

//Flight Timer
//createSafeString(flightTimerString, 40); // create an empty string called msgStr with a capacity if 40 chars
String flightTimerString;
unsigned int flightStartTime = 0;        // Value in millis of when the flight starts
bool flightTimerStatus = false;           // Status for if the flight starts
float altFtStart = 0;                     // Alt of the payload when the payload is turned on, is compared against current alt to start flight timer

bool tempHumStatus = 0;
float temperature = 999;
float humidity = 999;

// Create a new sensor object
sBmm150MagData_t magData;
BMI270 imu;
DFRobot_BMM150_I2C bmm150(&Wire1, UINT8_C(0x10));

float bmiDataStatus[6] = {999, 998, 997, 996, 995, 994};
int bmiDataStatusTimer = 0;

bool colorDistStatus = 0;

int proximity = 0;
//int r = 0, g = 0, b = 0;

// updateFrequency function
unsigned long hzPrevTime = 0;
float frequencyHz = 0;

byte timer2 = 0, timer3 = 0, timer4 = 0, timer5 = 0, timer6 = 0, timer7 = 0, timerTotal = 0;;

////////////////////////////////////////////////////////// GPS //////////////////////////////////////////////////////////

#define UBLOX_SERIAL Serial1

// bool usingM9N = true;                         //sparkfun library = NAV messages (ecef & PVT), tiny GPS = NMEA messages (traditional LLA)

byte gpsStatus = 0;

SFE_UBLOX_GNSS sparkFunGNSS;
dynModel dynamicModel = DYN_MODEL_AIRBORNE4g; //setting dynamic model to 8 (airborne 4g) for rocket, 2g (for pterydactyl is 7)

bool hpGPS = false;
int gpsMonth = 999;
int gpsDay = 999;
int gpsYear = 999;
int gpsHour = 999;
int gpsMinute = 999;
int gpsSecond = 999;
int gpsMillisecond = 999;
int gpsTimeOfWeek = 999;
// double gpsLat = 999;
// double gpsLon = 999;
// double gpsAltM = 999;
// double gpsAltFt = 999;
// double gpsGndSpeed = 999;
// double gpsHeading = 999;
double gpsLat = 999;
double gpsLon = 999;
float gpsAltM = 999;
float gpsAltFt = 999;
float gpsGndSpeed = 999;
float gpsHeading = 999;
int gpsPDOP = 999;

long gpsLatDec = 999;
int gpsLatInt = 999;
long gpsLonDec = 999;
int gpsLonInt = 999;

float gpsVertVelFt = 999;
float gpsVertVelM = 999;
unsigned long gpsPrevTime = 0;
float gpsPrevAltFt = 0;

double velocityNED[3] = {999,999,999};

//High Percision GPS
float altitudeFtGPS = 999;
float latitudeGPS = 999;
float longitudeGPS = 999;
//initializaiton of ECEF variables
double ecefX = 999, ecefY = 999, ecefZ = 999;
//double posAcc = 999;
int SIV = 0;
//double latCalc = 999, longCalc = 999, altCalcFt = 999, altCalcM = 999;
//UTM vairabels:
// double UTMNorthing = 999, UTMEasting = 999;
// char UTMZoneLetter = '?';
// int UTMZoneNum = 999;
int gpsHorizAcc = 999;
int gpsVertAcc = 999;

int gpsYaw = 999;
int gpsPitch = 999;
int gpsRoll = 999;

int setDynModel = 0;

bool ecefStatus = false;

SFE_UBLOX_GNSS myLBand; // NEO-D9S

const uint32_t myLBandFreq = 1556290000; // Uncomment this line to use the US SPARTN 1.8 service

bool rxmStatus = false;
bool pvtStatus = false;
bool lbandStatus = false;

bool PVTstatus = false;

//createSafeString(fixTypeGPS, 30); // create an empty string called msgStr with a capacity if 40 chars
//createSafeString(fixTypeRTK, 7); // create an empty string called msgStr with a capacity if 40 chars
String fixTypeGPS;
String fixTypeRTK;
////////////////////////////////////////////////////////// GPS //////////////////////////////////////////////////////////

ReefwingLPS22HB LPS22HB;

#define XBEE_BAUD 9600      // If the baud of the xbee isn's set to this in XTCU it will not connect. Load the XBee in XTCU and click default.
#define XBee Serial1   // Define the serial port of the XBee
// Nano: Serial or Serial1 for BLE33
//    ( 1,  0)
//DOUT RX  TX DIN

// Teensy: Serial8
//    (34, 35)
//DOUT RX  TX DIN

String xbeeID = "GX26";     // Choose an ID for your XBee - 2-4 character string, A-Z and 0-9 only please
                           // Software ID only, not sent to the XBee by any means.

// XBee Configuration Varibales
String scanChID = "CCCC";  // Leave this, never needs to be changed.
String channelID = "10";   // Channel, similar to a TV.
String networkID = "A100"; // Can have multiple networks on a channel but it's not ideal, important for dynamically changing networks as a payload.

bool usingXBee = false;

// OLED
OLED OLED;   // Initialising OLED screen.
unsigned short OLEDdt = 750; //dt stands for delay time, The time given to the user to read the Screen.
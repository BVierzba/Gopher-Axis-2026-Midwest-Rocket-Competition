/************************************************
      PADL-33 GPS Code
      Created by: Ashton Posey
************************************************/
//Purpose: General functions to run the NEO-M9N, NEO-M8N, & ZED-F9P GPS

////////////////////////////////////// GPS Set Up //////////////////////////////////////
// void gpsSetUp();
// void gpsUpdate();
// void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct);// ZED-F9P Function
// void printRXMCOR(UBX_RXM_COR_data_t *ubxDataStruct); // ZED-F9P Function

/////////////////////////////////// RTK Correction Keys ///////////////////////////////////
// The D9S is a corrections reciever that uses the PointPerfect sattelite system to recieve corrections.
// If you are not using the D9S you are not using RTK and these keys are not required. 
// The f9P works just fine at 10Hz without the RTK
// Current Max Frequency of RTK is 1Hz
// Documentation for getting keys, can be found on the ballooning drive under F9P RTK GPS:
// https://docs.google.com/document/d/1LPn6epG78DDsRGPjsu2ZfjNjDn0iGQSRisbK2AWPjNg/edit?usp=sharing

// You can set the information below after signing up with the u-blox Thingstream portal 
// and adding a new New PointPerfect Thing (L-Band or L-Band + IP)
// https://portal.thingstream.io/app/location-services/things
// In the new PointPerfect Thing, you go to the credentials tab and copy and paste the IP Dynamic Keys here.
//
// The keys are valid from a particular GPS Week Number and Time of Week.
// Looking at the credentials tab, the current key expires 23:59 Feb 11th 2022.
// This means the next key is valid _from_ Midnight Feb 12th 2022.
// That is GPS Week 2196. The GPS Time of Week in seconds is 518400.
// Working backwards, the current key became valid exactly 4 weeks earlier (Midnight Jan 15th 2022).
//
// See: https://www.labsat.co.uk/index.php/en/gps-time-calculator
//
// The keys are given as: 32 hexadecimal digits = 128 bits = 16 Bytes
//
// The next example shows how to retrieve the keys using ESP32 WiFi and MQTT.
// You can cut and paste the keys and GPS week/time-of-week from that example into here.

const uint8_t currentKeyLengthBytes =   16; 
const char currentDynamicKey[] =        "7875bc2e9cb9252cef9070aef9ecd091";
const uint16_t currentKeyGPSWeek =      2297; // Update this when you add new keys
const uint32_t currentKeyGPSToW =       84558;

const uint8_t nextKeyLengthBytes =      16; 
const char nextDynamicKey[] =           "3fce6f457544996e2e6292b260b52cd7";
const uint16_t nextKeyGPSWeek =         2301; // Update this when you add new keys
const uint32_t nextKeyGPSToW =          84558;
/////////////////////////////////// RTK Correction Keys ///////////////////////////////////

void gpsSetUp(){
    if (gpsI2C){
        for (int i=0; i<10; i++){
            if (sparkFunGNSS.begin(Wire)){
                Serial.println(F("u-blox GNSS Online!"));
                if(usingBuzzer) tone(TONE_PIN, 550, 50);
                greenLedCounter++;
                break;
            }
            else {
                Serial.println(F("u-blox GNSS Offline"));
                if (i == 9){
                    if (usingBuzzer) tone(TONE_PIN, 400);
                    digitalWrite(LEDR, LOW);       // will turn the LED on
                    RED_LED.on();
                }
            }
            delay(20);
        }
    }
    else {
        UBLOX_SERIAL.begin(GPS_BAUD);
        for (int i=0; i<10; i++){
            //UBLOX_SERIAL.begin(GPS_BAUD);
            if (sparkFunGNSS.begin(UBLOX_SERIAL)){
                Serial.println(F("u-blox GNSS Online!"));
                if(usingBuzzer) tone(TONE_PIN, 550, 50);
                if (usingOLED){
                  OLED.update("GPS\nOnline!");
                  delay(OLEDdt);
                }
                greenLedCounter++;
                break;
            }
            else {
                Serial.println(F("u-blox GNSS Offline"));
                if (usingOLED){
                  OLED.update("GPS\nOffline...");
                  delay(OLEDdt);
                }
                if (i == 9){
                    if (usingBuzzer) tone(TONE_PIN, 400);
                    digitalWrite(LEDR, LOW);       // will turn the LED on
                    RED_LED.on();
                }
            }
            delay(20);
        }
    }

    if (ecefEnabled) sparkFunGNSS.setI2COutput(COM_TYPE_NMEA | COM_TYPE_UBX); // Turn on both UBX and NMEA sentences on I2C. (Turn off RTCM and SPARTN)
    else sparkFunGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    // sparkFunGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_I2C); // Several of these are on by default on ublox board so let's disable them
    // sparkFunGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_I2C);
    // sparkFunGNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_I2C);
    // sparkFunGNSS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_I2C);
    // sparkFunGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_I2C);
    // sparkFunGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C); // Leave only GGA enabled at current navigation rate

    
    sparkFunGNSS.setNavigationFrequency(GPS_FREQUENCY); //Set output to 5 times a second
    byte rate = sparkFunGNSS.getNavigationFrequency(); //Get the update rate of this module
    Serial.println("Current Frequency: " + String(rate));
    if(rate != GPS_FREQUENCY){
        for(int i=0; i<50; i++){ //reference list of dynamic models can be found at the bottom of this tab. 0 is pedestrian (default), 7 is airborne2g, 8 is airborne4g
            sparkFunGNSS.setNavigationFrequency(GPS_FREQUENCY); //Set output to 5 times a second
            rate = sparkFunGNSS.getNavigationFrequency(); //Get the update rate of this module
            Serial.println("Current Frequency: " + String(rate));
            if(rate == GPS_FREQUENCY) break;
        }
    }

    sparkFunGNSS.setDynamicModel(dynamicModel); //if returns 255 then that is an error, unable to communicate with gps
    gpsStatus = sparkFunGNSS.getDynamicModel();
    Serial.println("Current Airmode: " + String(gpsStatus) + ", attempting to switch to airborne4g/8");
    if (usingOLED){
      OLED.update("Airborne\nMode:\n" + String(gpsStatus) + "\n\nAttempt:\n1");
      delay(OLEDdt);
    }
    if(gpsStatus != 8){
        for(int i=0; i<50; i++){ //reference list of dynamic models can be found at the bottom of this tab. 0 is pedestrian (default), 7 is airborne2g, 8 is airborne4g
            sparkFunGNSS.setDynamicModel(dynamicModel); //if returns 255 then that is an error, unable to communicate with gps
            gpsStatus  = sparkFunGNSS.getDynamicModel();
            if (usingOLED){
              OLED.update("Airborne\nMode:\n" + String(gpsStatus) + "\n\nAttempt:\n" + String(i));
              delay(OLEDdt);
            }
            Serial.println("Current Airmode: " + String(gpsStatus) + ", attempting to switch to airborne4g/8");
            if(gpsStatus == 8) break;
        }
    }

    //dataDelay = 1000 / GPS_FREQUENCY;
    Serial.println("GPS Data Delay = " + String(GPS_DATA_DELAY));
    int mesRate = sparkFunGNSS.setMeasurementRate(GPS_DATA_DELAY);  //Produce a measurement every 1000ms
    Serial.println("Current Navigation Rate: " + String(rate));
    if (mesRate != GPS_DATA_DELAY) {
      for (int i = 0; i < 50; i++) {                  //reference list of dynamic models can be found at the bottom of this tab. 0 is pedestrian (default), 7 is airborne2g, 8 is airborne4g
        sparkFunGNSS.setMeasurementRate(GPS_DATA_DELAY);   //Set output to 5 times a second
        mesRate = sparkFunGNSS.getMeasurementRate();  //Get the update rate of this module
        Serial.println("Current Navigation Rate: " + String(mesRate));
        if (mesRate == GPS_DATA_DELAY) break;
      }
    }

    //if (gpsI2C || gpsType == "M9")
    for (int i = 0; i < 10; i++) sparkFunGNSS.setAutoPVTrate(1);  //Tell the GNSS to send the PVT solution every measurement
    //sparkFunGNSS.setAutoPVTrate(1); //Tell the GNSS to send the PVT solution every measurement
    sparkFunGNSS.setAutoNAVPOSECEFrate(1);  //Tell the GNSS to send each POSECEF solution every 5th measurement
    if (ecefEnabled && gpsI2C) sparkFunGNSS.setAutoNAVPOSECEFrate(1); //Tell the GNSS to send each POSECEF solution every 5th measurement
    sparkFunGNSS.setHighPrecisionMode(false); // Enable High Precision Mode - include extra decimal places in the GGA messages
    //}
    
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // Begin and configure the ZED-F9x
    if (rtkEnabled){
        //sparkFunGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

        //uint8_t ok = sparkFunGNSS.setI2COutput(COM_TYPE_UBX); //Turn off NMEA noise
        
        //uint8_t ok = sparkFunGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_SPARTN); //Be sure SPARTN input is enabled
        //uint8_t ok = sparkFunGNSS.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_SPARTN); //Be sure SPARTN input is enabled
        uint8_t ok = sparkFunGNSS.setPortInput(COM_PORT_UART2, COM_TYPE_UBX | COM_TYPE_RTCM3 | COM_TYPE_SPARTN); //Be sure SPARTN input is enabled

        if (ok) ok = sparkFunGNSS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED); // Set the differential mode - ambiguities are fixed whenever possible
        if (ok) ok = sparkFunGNSS.setVal8(UBLOX_CFG_SPARTN_USE_SOURCE, 1); // use LBAND PMP message

        //if (ok) ok = sparkFunGNSS.setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_COR_I2C, 1); // Enable UBX-RXM-COR messages on I2C
        //if (ok) ok = sparkFunGNSS.setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_COR_UART1, 1); // Enable UBX-RXM-COR messages on I2C
        if (ok) ok = sparkFunGNSS.setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_COR_UART2, 1); // Enable UBX-RXM-COR messages on I2C
        
        //Configure the SPARTN IP Dynamic Keys
        //"When the receiver boots, the host should send 'current' and 'next' keys in one message." - Use setDynamicSPARTNKeys for this.
        //"Every time the 'current' key is expired, 'next' takes its place."
        //"Therefore the host should then retrieve the new 'next' key and send only that." - Use setDynamicSPARTNKey for this.
        // The key can be provided in binary (uint8_t) format or in ASCII Hex (char) format, but in both cases keyLengthBytes _must_ represent the binary key length in bytes.
        for (int i=0; i<5; i++) {
            if (ok) ok = sparkFunGNSS.setDynamicSPARTNKeys(currentKeyLengthBytes, currentKeyGPSWeek, currentKeyGPSToW, currentDynamicKey,
                                                          nextKeyLengthBytes, nextKeyGPSWeek, nextKeyGPSToW, nextDynamicKey);
            delay(20);
        }

        //if (ok) ok = sparkFunGNSS.saveConfiguration(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_MSGCONF); //Optional: Save the ioPort and message settings to NVM
        
        Serial.print(F("GNSS: configuration "));
        Serial.println(OK(ok));
        //sparkFunGNSS.setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata so we can watch the carrier solution go to fixed

        //sparkFunGNSS.setRXMCORcallbackPtr(&printRXMCOR); // Print the contents of UBX-RXM-COR messages so we can check if the PMP data is being decrypted successfully

        //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        // Begin and configure the NEO-D9S L-Band receiver

        //myLBand.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

        for (int i=0; i<50 && !lbandStatus; i++){
              lbandStatus = (myLBand.begin(Wire1, 0x43)) ? true : false;

              if (!lbandStatus){
                  Serial.println("Trying L-Band Setup Again!");
              }

          }

          if (lbandStatus) {
              Serial.println("L-Band     Online!");
          }
          else {
              Serial.println("L-Band     Offline...");
          }

                ok = myLBand.setVal32(UBLOX_CFG_PMP_CENTER_FREQUENCY,   myLBandFreq); // Default 1539812500 Hz
        if (ok) ok = myLBand.setVal16(UBLOX_CFG_PMP_SEARCH_WINDOW,      2200);        // Default 2200 Hz
        if (ok) ok = myLBand.setVal8(UBLOX_CFG_PMP_USE_SERVICE_ID,      0);           // Default 1 
        if (ok) ok = myLBand.setVal16(UBLOX_CFG_PMP_SERVICE_ID,         21845);       // Default 50821
        if (ok) ok = myLBand.setVal16(UBLOX_CFG_PMP_DATA_RATE,          2400);        // Default 2400 bps
        if (ok) ok = myLBand.setVal8(UBLOX_CFG_PMP_USE_DESCRAMBLER,     1);           // Default 1
        if (ok) ok = myLBand.setVal16(UBLOX_CFG_PMP_DESCRAMBLER_INIT,   26969);       // Default 23560
        if (ok) ok = myLBand.setVal8(UBLOX_CFG_PMP_USE_PRESCRAMBLING,   0);           // Default 0
        if (ok) ok = myLBand.setVal64(UBLOX_CFG_PMP_UNIQUE_WORD,        16238547128276412563ull); 

        if (ok) ok = myLBand.setVal8(UBLOX_CFG_I2COUTPROT_UBX, 1);           // Enable UBX output on I2C
        if (ok) ok = myLBand.setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_I2C,   1); // Ensure UBX-RXM-PMP is enabled on the I2C port 
        if (ok) ok = myLBand.setVal8(UBLOX_CFG_UART1OUTPROT_UBX, 0);         // Enable UBX output on UART1
        if (ok) ok = myLBand.setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART1, 0); // Output UBX-RXM-PMP on UART1
        if (ok) ok = myLBand.setVal8(UBLOX_CFG_UART2OUTPROT_UBX, 1);         // Enable UBX output on UART2
        if (ok) ok = myLBand.setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART2, 1); // Output UBX-RXM-PMP on UART2
        if (ok) ok = myLBand.setVal32(UBLOX_CFG_UART1_BAUDRATE,         38400); // match baudrate with ZED default
        if (ok) ok = myLBand.setVal32(UBLOX_CFG_UART2_BAUDRATE,         38400); // match baudrate with ZED default
        
        // Serial.print(F("L-Band: configuration "));
        // Serial.println(OK(ok));

        myLBand.softwareResetGNSSOnly(); // Do a restart

        while (!rxmStatus){
            //sparkFunGNSS.setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata so we can watch the carrier solution go to fixed
            sparkFunGNSS.setRXMCORcallbackPtr(&printRXMCOR); // Print the contents of UBX-RXM-COR messages so we can check if the PMP data is being decrypted successfully
            sparkFunGNSS.checkUblox(); // Check for the arrival of new GNSS data and process it.
            sparkFunGNSS.checkCallbacks(); // Check if any GNSS callbacks are waiting to be processed.
            Serial.println("F9P");
        }
    }

    Serial.println("GPS Status (7): " + String(gpsStatus));
}

void gpsUpdate(){
    PVTstatus = sparkFunGNSS.getPVT();
    if (gpsStatus == 8 && PVTstatus){
        gpsAltM =  sparkFunGNSS.getAltitude() * pow(10.0, -3); //ublox.getAlt_feet();  //older used in OG pterodactyl
        gpsAltFt = gpsAltM * 3.280839895;
        gpsLat = sparkFunGNSS.getLatitude() * pow(10.0, -7); // ublox.getLat();
        gpsLon = sparkFunGNSS.getLongitude() * pow(10.0, -7); //ublox.getLon();
        // Serial.println(gpsLat);
        // Serial.println(gpsLon);

        gpsLatDec = abs(long(gpsLat * pow(10.0, 7))) % 10000000;
        if (gpsLat < 0) gpsLatInt = floor(gpsLat) + 1;
        else gpsLatInt = floor(gpsLat);

        gpsLonDec = abs(long(gpsLon * pow(10.0, 7))) % 10000000;
        if (gpsLon < 0) gpsLonInt = floor(gpsLon) + 1;
        else gpsLonInt = floor(gpsLon);
        
        SIV = sparkFunGNSS.getSIV();

        if (SIV > 0) BLUE_LED.on();
        else BLUE_LED.off();

        gpsMonth = sparkFunGNSS.getMonth();
        gpsDay = sparkFunGNSS.getDay();
        gpsYear = sparkFunGNSS.getYear();

        gpsHour = sparkFunGNSS.getHour() - 5;  // Their time zone is 5h ahead
        if (gpsHour < 0) gpsHour = gpsHour + 24;
        gpsMinute = sparkFunGNSS.getMinute();
        gpsSecond = sparkFunGNSS.getSecond();
        gpsMillisecond = sparkFunGNSS.getMillisecond();

        gpsTimeOfWeek = sparkFunGNSS.getTimeOfWeek();
        //gpsWNO = sparkFunGNSS.time

        gpsGndSpeed = sparkFunGNSS.getGroundSpeed()* pow(10.0, -3); // mm/s to m/s
        gpsHeading = sparkFunGNSS.getHeading() * pow(10.0, -5); // deg * 10^5 to deg
        gpsPDOP = sparkFunGNSS.getPDOP();

        gpsVertVelFt = (gpsAltFt - gpsPrevAltFt) / (timerSec - gpsPrevTime);
        gpsVertVelM = gpsVertVelFt * 0.3047999995367;
        gpsPrevAltFt = gpsAltFt;
        gpsPrevTime = timerSec;

        byte fixType = sparkFunGNSS.getFixType();
        if(fixType == 0) fixTypeGPS = "No Fix";
        else if(fixType == 1) fixTypeGPS = "Dead reckoning";
        else if(fixType == 2) fixTypeGPS = "2D";
        else if(fixType == 3) fixTypeGPS =  "3D";
        else if(fixType == 4) fixTypeGPS = "GNSS + Dead reckoning";
        else if(fixType == 5) fixTypeGPS = "Time only";

        byte RTK = sparkFunGNSS.getCarrierSolutionType();
        if (RTK == 0) fixTypeRTK = "No RTK";
        else if (RTK == 1) fixTypeRTK = "HPF RTK";
        else if (RTK == 2) fixTypeRTK = "HP RTK"; // High po

        velocityNED[0] = sparkFunGNSS.getNedNorthVel() / 1000.0;
        velocityNED[1] = sparkFunGNSS.getNedEastVel() / 1000.0;
        velocityNED[2] = sparkFunGNSS.getNedDownVel() / 1000.0;

        gpsHorizAcc = sparkFunGNSS.getHorizontalAccEst(); // Print the horizontal accuracy estimate
        gpsVertAcc = sparkFunGNSS.getVerticalAccEst();

        // gpsYaw = sparkFunGNSS.getVehicleHeading();
        // gpsPitch = sparkFunGNSS.getVehiclePitch();
        // gpsRoll = sparkFunGNSS.getVehicleRoll();

        sparkFunGNSS.flushPVT();
    }

    if (ecefEnabled){
        // Calling getNAVPOSECEF returns true if there actually is a fresh position solution available.
        ecefStatus = sparkFunGNSS.getNAVPOSECEF();
        if (ecefStatus){
            // updateECEF();
            ecefX = sparkFunGNSS.packetUBXNAVPOSECEF->data.ecefX / 100.0; // convert ecefX to m
            ecefY = sparkFunGNSS.packetUBXNAVPOSECEF->data.ecefY / 100.0; // convert ecefY to m
            ecefZ = sparkFunGNSS.packetUBXNAVPOSECEF->data.ecefZ / 100.0; // convert ecefY to m
            sparkFunGNSS.flushNAVPOSECEF(); //Mark all the data as read/stale so we get fresh data next time
        }
    }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallbackPtr
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  pvtStatus = true;
  double latitude = ubxDataStruct->lat; // Print the latitude
  Serial.print(F("Lat: "));
  Serial.print(latitude / 10000000.0, 7);

  double longitude = ubxDataStruct->lon; // Print the longitude
  Serial.print(F("  Long: "));
  Serial.print(longitude / 10000000.0, 7);

  double altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
  Serial.print(F("  Height: "));
  Serial.print(altitude / 1000.0, 3);

  uint8_t fixType = ubxDataStruct->fixType; // Print the fix type
  Serial.print(F("  Fix: "));
  Serial.print(fixType);
  if (fixType == 0)
    Serial.print(F(" (None)"));
  else if (fixType == 1)
    Serial.print(F(" (Dead Reckoning)"));
  else if (fixType == 2)
    Serial.print(F(" (2D)"));
  else if (fixType == 3)
    Serial.print(F(" (3D)"));
  else if (fixType == 3)
    Serial.print(F(" (GNSS + Dead Reckoning)"));
  else if (fixType == 5)
    Serial.print(F(" (Time Only)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  uint8_t carrSoln = ubxDataStruct->flags.bits.carrSoln; // Print the carrier solution
  Serial.print(F("  Carrier Solution: "));
  Serial.print(carrSoln);
  if (carrSoln == 0)
    Serial.print(F(" (None)"));
  else if (carrSoln == 1)
    Serial.print(F(" (Floating)"));
  else if (carrSoln == 2)
    Serial.print(F(" (Fixed)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  uint32_t hAcc = ubxDataStruct->hAcc; // Print the horizontal accuracy estimate
  Serial.print(F("  Horizontal Accuracy Estimate: "));
  Serial.print(hAcc);
  Serial.print(F(" (mm)"));

  Serial.println();    
}


//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printRXMCOR will be called when new RXM COR data arrives
// See u-blox_structs.h for the full definition of UBX_RXM_COR_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setRXMCORcallbackPtr
//        /                  _____  This _must_ be UBX_RXM_COR_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printRXMCOR(UBX_RXM_COR_data_t *ubxDataStruct)
{
  rxmStatus = true;
  Serial.print(F("UBX-RXM-COR:  ebno: "));
  Serial.print((double)ubxDataStruct->ebno / 8, 3); //Convert to dB

  Serial.print(F("  protocol: "));
  if (ubxDataStruct->statusInfo.bits.protocol == 1)
    Serial.print(F("RTCM3"));
  else if (ubxDataStruct->statusInfo.bits.protocol == 2)
    Serial.print(F("SPARTN"));
  else if (ubxDataStruct->statusInfo.bits.protocol == 29)
    Serial.print(F("PMP (SPARTN)"));
  else if (ubxDataStruct->statusInfo.bits.protocol == 30)
    Serial.print(F("QZSSL6"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  errStatus: "));
  if (ubxDataStruct->statusInfo.bits.errStatus == 1)
    Serial.print(F("Error-free"));
  else if (ubxDataStruct->statusInfo.bits.errStatus == 2)
    Serial.print(F("Erroneous"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  msgUsed: "));
  if (ubxDataStruct->statusInfo.bits.msgUsed == 1)
    Serial.print(F("Not used"));
  else if (ubxDataStruct->statusInfo.bits.msgUsed == 2)
    Serial.print(F("Used"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  msgEncrypted: "));
  if (ubxDataStruct->statusInfo.bits.msgEncrypted == 1)
    Serial.print(F("Not encrypted"));
  else if (ubxDataStruct->statusInfo.bits.msgEncrypted == 2)
    Serial.print(F("Encrypted"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  msgDecrypted: "));
  if (ubxDataStruct->statusInfo.bits.msgDecrypted == 1)
    Serial.print(F("Not decrypted"));
  else if (ubxDataStruct->statusInfo.bits.msgDecrypted == 2)
    Serial.print(F("Successfully decrypted"));
  else
    Serial.print(F("Unknown"));

  Serial.println();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


// // First, let's collect the position data
//     int32_t latitude;
//     int8_t latitudeHp;
//     int32_t longitude;
//     int8_t longitudeHp;
//     int32_t ellipsoid;
//     int8_t ellipsoidHp;
//     int32_t msl;
//     int8_t mslHp;
//     //uint32_t accuracy = sparkFunGNSS.getHorizontalAccuracy();

//     // Defines storage for the lat and lon units integer and fractional parts
//     int32_t lat_int; // Integer part of the latitude in degrees
//     int32_t lat_frac; // Fractional part of the latitude
//     int32_t lon_int; // Integer part of the longitude in degrees
//     int32_t lon_frac; // Fractional part of the longitude

//     float f_ellipsoid;
//     float f_msl;
//     float f_accuracy;

// getHighResLatitude: returns the latitude from HPPOSLLH as an int32_t in degrees * 10^-7
    // getHighResLatitudeHp: returns the high resolution component of latitude from HPPOSLLH as an int8_t in degrees * 10^-9
    // getHighResLongitude: returns the longitude from HPPOSLLH as an int32_t in degrees * 10^-7
    // getHighResLongitudeHp: returns the high resolution component of longitude from HPPOSLLH as an int8_t in degrees * 10^-9
    // getElipsoid: returns the height above ellipsoid as an int32_t in mm
    // getElipsoidHp: returns the high resolution component of the height above ellipsoid as an int8_t in mm * 10^-1
    // getMeanSeaLevel: returns the height above mean sea level as an int32_t in mm
    // getMeanSeaLevelHp: returns the high resolution component of the height above mean sea level as an int8_t in mm * 10^-1
    // getHorizontalAccuracy: returns the horizontal accuracy estimate from HPPOSLLH as an uint32_t in mm * 10^-1

    // If you want to use the high precision latitude and longitude with the full 9 decimal places
    // you will need to use a 64-bit double - which is not supported on all platforms

    // To allow this example to run on standard platforms, we cheat by converting lat and lon to integer and fractional degrees

    // The high resolution altitudes can be converted into standard 32-bit float

    // First, let's collect the position data
    //  latitude = sparkFunGNSS.getHighResLatitude();
    //  latitudeHp = sparkFunGNSS.getHighResLatitudeHp();
    //  longitude = sparkFunGNSS.getHighResLongitude();
    //  longitudeHp = sparkFunGNSS.getHighResLongitudeHp();
    //  ellipsoid = sparkFunGNSS.getElipsoid();
    //  ellipsoidHp = sparkFunGNSS.getElipsoidHp();
    //  msl = sparkFunGNSS.getMeanSeaLevel();
    //  mslHp = sparkFunGNSS.getMeanSeaLevelHp();
    //uint32_t accuracy = sparkFunGNSS.getHorizontalAccuracy();

    // // Calculate the latitude and longitude integer and fractional parts
    // lat_int = latitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
    // lat_frac = latitude - (lat_int * 10000000); // Calculate the fractional part of the latitude
    // lat_frac = (lat_frac * 100) + latitudeHp; // Now add the high resolution component
    // if (lat_frac < 0) // If the fractional part is negative, remove the minus sign
    // {
    //   lat_frac = 0 - lat_frac;
    // }
    // lon_int = longitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
    // lon_frac = longitude - (lon_int * 10000000); // Calculate the fractional part of the longitude
    // lon_frac = (lon_frac * 100) + longitudeHp; // Now add the high resolution component
    // if (lon_frac < 0) // If the fractional part is negative, remove the minus sign
    // {
    //   lon_frac = 0 - lon_frac;
    // }

    // // Calculate the height above ellipsoid in mm * 10^-1
    // f_ellipsoid = (ellipsoid * 10) + ellipsoidHp;
    // // Now convert to m
    // f_ellipsoid = f_ellipsoid / 10000.0; // Convert from mm * 10^-1 to m

    // // Calculate the height above mean sea level in mm * 10^-1
    // f_msl = (msl * 10) + mslHp;
    // // Now convert to m
    // f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m

    // // Convert the horizontal accuracy (mm * 10^-1) to a float
    // //f_accuracy = accuracy;
    // // Now convert to m
    // f_accuracy = f_accuracy / 10000.0; // Convert from mm * 10^-1 to m

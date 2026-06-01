/************************************************
      PADL-33 KX13x 64G IMU Code
      Created by: Ashton Posey
************************************************/
//Purpose: General functions to run the KX13x 64G IMU using a qwiic connector on SDA & SCL
//         Must be enabled in variables.h -> bool usingKX 

////////////////////////////////////// KX 64G Accelerometer System Set Up //////////////////////////////////////
// void imu64gSetUp();
// void imu64gUpdate();

void imu64gSetUp(){
    if(usingKX){
      if (!kxAccel.begin())
      {
          Serial.println("Could not communicate with the the KX13X. Freezing.");
      }
      else {
          kxStatus = true;

          kxAccel.softwareReset(); // Give some time for the accelerometer to reset.
          delay(5);                    // It needs two, but give it five for good measure.

          // Many settings for KX13X can only be
          // applied when the accelerometer is powered down.
          // However there are many that can be changed "on-the-fly"
          // check datasheet for more info, or the comments in the
          // "...regs.h" file which specify which can be changed when.
          kxAccel.enableAccel(false);
          kxAccel.setRange(SFE_KX134_RANGE64G); // 64g for the KX134
          kxAccel.enableDataEngine(); // Enables the bit that indicates data is ready.
          kxAccel.setOutputDataRate(GPS_FREQUENCY); // Default is 50Hz
          kxAccel.enableAccel();
      }
    }
}

void imu64gUpdate(){
    if(usingKX){
        // Check if data is ready.
        if(kxStatus && kxAccel.dataReady()){
            kxAccel.getAccelData(&myData);
            kxData[0] = myData.xData * 1000;
            kxData[1] = myData.yData * 1000;
            kxData[2] = myData.zData * 1000;
        }
        kxDataStatus[kxDataStatusTimer] = kxData[0]; // check if the pressure sensor is actually working
        kxDataStatusTimer++;
        if(kxDataStatusTimer == 6) kxDataStatusTimer = 0;
        if(kxDataStatus[0] == kxDataStatus[1] && kxDataStatus[1] == kxDataStatus[2] && kxDataStatus[2] == kxDataStatus[3] && 
            kxDataStatus[3] == kxDataStatus[4] && kxDataStatus[4] == kxDataStatus[5] && kxDataStatus[5] == kxDataStatus[0]){
            if (usingBuzzer) tone(TONE_PIN, 400);
            Serial.println("KXL ERROR!");
        }
    }
}
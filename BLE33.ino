/************************************************
      PADL-33 BLE33 Sensors Code
      Created by: Ashton Posey
************************************************/
//Purpose: General functions to run the pressure sensor, 9 dof imu, humidity sensor, & color/distance sensor.

////////////////////////////////////// Pressure Sensor Set Up //////////////////////////////////////
void pressureSensorSetUp();
float PSI_to_Ft(float const &pressurePSI);
void pressureRead(float *pressureSensor);
void pressureSensorUpdate();
void imuSetUp();
void imuUpdate();
// void humiditySensorSetUp();
// void colorDistanceSensorSetUp();

void pressureSensorSetUp(){
    LPS22HB.begin();
    
    for (int i=0; i<10; i++){
        if (LPS22HB.connected()){
            Serial.println("Pressure Sensor Online!");
            if(usingBuzzer) tone(TONE_PIN, 550, 50);

            if (usingOLED){
              OLED.update("Pressure\nSensor\nOnline!");
              delay(OLEDdt);
            }

            pressureStatus = true;
            greenLedCounter++;
            pressureRead(pressureSensor);
            altFtStart = pressureSensor[5];
            LPS22HB.setODR(Rate::RATE_25_HZ);
            
            T1 = 59-.00356*h1;
            T2 = -70;
            T3 = -205.05 + .00164*h2;

            pressureBoundary1 = (2116 * pow(((T1+459.7)/518.6),5.256));
            pressureBoundary2 = (473.1*exp(1.73-.000048*h2)); // does exp function work??
            pressureBoundary3 = (51.97*pow(((T3 + 459.7)/389.98),-11.388)); 

            msPrevAltFt = pressureSensor[5];
            break;
        }
        else {
            LPS22HB.begin();
            Serial.println("LPS22HB Pressure Sensor Offline...");
            if (usingOLED){
              OLED.update("Pressure\nSensor\nOffline!");
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
    delay(20);
}

float PSI_to_Ft(float const &pressurePSI){
    float pressurePSF = (pressurePSI*144.0);
    float altitudeFt = -999.9;

    if (pressurePSF > pressureBoundary1)  altitudeFt = (459.7 + 59.0 - 518.6*pow( (pressurePSF/2116.0), (1.0/5.256)) ) / 0.00356; // altitude is less than 36,152 ft ASL
    else if (pressurePSF <= pressureBoundary1 && pressurePSF > pressureBoundary2) altitudeFt = (1.73 - log(pressurePSF/473.1)) / 0.000048; // altitude is between 36,152 and 82,345 ft ASL
    else if (pressurePSF <= pressureBoundary2) altitudeFt = (459.7 - 205.5 - 389.98 * pow((pressurePSF / 51.97), (1.0 / -11.388))) / -0.00164; // altitude is greater than 82,345 ft ASL

    return altitudeFt;
}

void pressureRead(float *pressureSensor){
    // read the sensor value
    if (!pressureStatus) {
        for(int i = 0; i<7; i++) pressureSensor[i] = 999;
        return;
    }
    pressureSensor[0] = LPS22HB.readPressure() / 10.0;
    pressureSensor[1] = pressureSensor[0] * 0.00986923;
    pressureSensor[2] = pressureSensor[0] * 0.145038;

    pressureSensor[3] = LPS22HB.readTemperature();
    pressureSensor[4] = (pressureSensor[3] * 9.0/5.0) + 32.0;
  
    pressureSensor[5] = PSI_to_Ft(pressureSensor[2]);
    pressureSensor[6] = pressureSensor[5] * 0.3048;

    // pressureSensor[7] = {pressureKPA, pressureATM, pressurePSI, temperatureC, temperatureF, altitudeFt, altitudeM};
    return;
}

void pressureSensorUpdate(){
    if (pressureStatus){
        pressureRead(pressureSensor);

        pressureSensorStatus[pressureSensorStatusTimer] = pressureSensor[5]; // check if the pressure sensor is actually working
        pressureSensorStatusTimer++;
        if (pressureSensorStatusTimer == 6) pressureSensorStatusTimer = 0;
        if (pressureSensorStatus[0] == pressureSensorStatus[1] && pressureSensorStatus[1] == pressureSensorStatus[2] && pressureSensorStatus[2] == pressureSensorStatus[3] && 
            pressureSensorStatus[3] == pressureSensorStatus[4] && pressureSensorStatus[4] == pressureSensorStatus[5] && pressureSensorStatus[5] == pressureSensorStatus[0])  {
            if (usingBuzzer) tone(TONE_PIN, 400);
            digitalWrite(LEDR, LOW);       // will turn the LED on
            RED_LED.on();
            Serial.println("BARO ERROR!");
        }

        setHeaterState();

        vertVelFt = (pressureSensor[5] - msPrevAltFt) / (timerSec - msPrevTime);
        vertVelM = vertVelFt * 0.3047999995367;
        msPrevAltFt = pressureSensor[5];
        msPrevTime = timerSec;

        if (pressureSensor[5] - altFtStart > 100 && !flightTimerStatus) {
            flightStartTime = timerSec;
            flightTimerStatus = true;
        }
        if (flightTimerStatus){ //flightTimerString = flightTimeStr(int(timerSec) - flightStartTime);
            // /////////////// UPDATE FLIGHT TIMER ///////////////
            long timer8 = int(timerSec) - flightStartTime; // Format: "00:00:00" -> h:m:s
            strTimer = String(timer8/3600);
            flightTimerString = strTimer;
            flightTimerString += ":";
            timer8 %= 3600;
            strTimer = String(timer8/600);
            flightTimerString += strTimer;
            timer8 %= 600;
            strTimer = String(timer8/60);
            flightTimerString += strTimer;
            flightTimerString += ":";
            timer8 %= 60;
            strTimer = String(timer8/10);
            flightTimerString += strTimer;
            strTimer = String(timer8 % 10);
            flightTimerString += strTimer;
            /////////////// UPDATE FLIGHT TIMER ///////////////
        }
    }
}


////////////////////////////////////// IMU Set Up //////////////////////////////////////
void imuSetUp(){
    for (int i=0; i<10; i++){
        if (imu.beginI2C(UINT8_C(0x68), Wire1) == BMI2_OK){
            Serial.println("BMI 270 Accelerometer & Gyroscope Online!");
            if(usingBuzzer) tone(TONE_PIN, 550, 50);
            if (usingOLED){
              OLED.update("Accel &\nGyro\nOnline!");
              delay(OLEDdt);
            }
            greenLedCounter++;
            break;
        }
        else {
            Serial.println(F("BMI 270 Accelerometer & Gyroscope Offline..."));
            if (usingOLED){
              OLED.update("Accel &\nGyro\nOffline...");
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
    delay(20);

    //imu.setAccelPowerMode(BMI2_POWER_OPT_MODE);
    imu.setAccelODR(BMI2_ACC_ODR_25HZ);
    imu.setGyroODR(BMI2_GYR_ODR_25HZ);

    // The accelerometer and gyroscope can be configured with multiple settings
    // to reduce the measurement noise. Both sensors have the following settings
    // in common:
    // .range       - Measurement range. Lower values give more resolution, but
    //                doesn't affect noise significantly, and limits the max
    //                measurement before saturating the sensor
    // .odr         - Output data rate in Hz. Lower values result in less noise,
    //                but lower sampling rates.
    // .filter_perf - Filter performance mode. Performance oprtimized mode
    //                results in less noise, but increased power consumption
    // .bwp         - Filter bandwidth parameter. This has several possible
    //                settings that can reduce noise, but cause signal delay
    // 
    // Both sensors have different possible values for each setting:
    // 
    // Accelerometer values:
    // .range       - 2g to 16g
    // .odr         - Depends on .filter_perf:
    //                  Performance mode: 12.5Hz to 1600Hz
    //                  Power mode:       0.78Hz to 400Hz
    // .bwp         - Depends on .filter_perf:
    //                  Performance mode: Normal, OSR2, OSR4, CIC
    //                  Power mode:       Averaging from 1 to 128 samples
    // 
    // Gyroscope values:
    // .range       - 125dps to 2000dps (deg/sec)
    // .ois_range   - 250dps or 2000dps (deg/sec) Only relevant when using OIS,
    //                see datasheet for more info. Defaults to 250dps
    // .odr         - Depends on .filter_perf:
    //                  Performance mode: 25Hz to 3200Hz
    //                  Power mode:       25Hz to 100Hz
    // .bwp         - Normal, OSR2, OSR4, CIC
    // .noise_perf  - Similar to .filter_perf. Performance oprtimized mode
    //                results in less noise, but increased power consumption
    // 
    // Note that not all combinations of values are possible. The performance
    // mode restricts which ODR settings can be used, and the ODR restricts some
    // bandwidth parameters. An error code is returned by setConfig, which can
    // be used to determine whether the selected settings are valid.
    int8_t err = BMI2_OK;

    // Set accelerometer config 
    bmi2_sens_config accelConfig;
    accelConfig.type = BMI2_ACCEL;
    accelConfig.cfg.acc.odr = BMI2_ACC_ODR_25HZ;
    accelConfig.cfg.acc.bwp = BMI2_ACC_CIC_AVG8;
    accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    accelConfig.cfg.acc.range = BMI2_ACC_RANGE_16G;
    err = imu.setConfig(accelConfig);

    // Set gyroscope config
    bmi2_sens_config gyroConfig;
    gyroConfig.type = BMI2_GYRO;
    gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_25HZ;
    gyroConfig.cfg.gyr.bwp = BMI2_GYR_CIC_MODE; // CIC mode does 7 data points in order to integrate for a better result
    gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    gyroConfig.cfg.gyr.ois_range = BMI2_GYR_OIS_2000;
    gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_2000;
    gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    err = imu.setConfig(gyroConfig);

    //delay(250);
    delay(20);

    for (int i=0; i<10; i++){
        if (!bmm150.begin()){ // False means true for some reason
            Serial.println("BMM 150 Magnetometer Online!");
            if(usingBuzzer) tone(TONE_PIN, 550, 50);
            if (usingOLED){
              OLED.update("Mag\nOnline!");
              delay(OLEDdt);
            }
            bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
            //bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
            bmm150.setRate(BMM150_DATA_RATE_25HZ);
            //bmm150.setMeasurementXYZ();
            greenLedCounter++;
            break;
        }
        else {
            Serial.println(F("BMM 150 Magnetometer Offline..."));
            if (usingOLED){
              OLED.update("Mag\nOffline!");
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
    delay(20);

    // while(bmm150.begin()){
    //     Serial.println("bmm150 init failed, Please try again!");
    //     delay(250);
    //     if (usingBuzzer) tone(TONE_PIN, 400);
    // } 
    // Serial.println("bmm150 init success!");

    // bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    // //bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    // bmm150.setRate(BMM150_DATA_RATE_25HZ);
    // //bmm150.setMeasurementXYZ();
}

void imuUpdate(){
    imu.getSensorData();
    // bmiDataStatus[bmiDataStatusTimer] = imu.data.accelX;; // check if the pressure sensor is actually working
    // bmiDataStatusTimer++;
    // // if (bmiDataStatusTimer == 6) bmiDataStatusTimer = 0;
    // // if (bmiDataStatus[0] == bmiDataStatus[1] && bmiDataStatus[1] == bmiDataStatus[2] && bmiDataStatus[2] == bmiDataStatus[3] && 
    // //     bmiDataStatus[3] == bmiDataStatus[4] && bmiDataStatus[4] == bmiDataStatus[5] && bmiDataStatus[5] == bmiDataStatus[0])  {
    // //     if (usingBuzzer) tone(TONE_PIN, 400);
    // //     Serial.println("IMU ERROR!");
    // // }
    
    magData = bmm150.getGeomagneticData();
    //float compassDegree = bmm150.getCompassDegree();
}

////////////////////////////////////// Temperature & Humidity Sensor Set Up //////////////////////////////////////
void humiditySensorSetUp(){
    if (HS300x.begin()) {
    Serial.println("Temperature & Humidity Sensor Online!");
    tempHumStatus = 1;
    }
    else {
      Serial.println("Temperature & Humidity Sensor Offline...");
      beep();
    }
}
////////////////////////////////////// Color & Distance Sensor Set Up //////////////////////////////////////
void colorDistanceSensorSetUp(){
    if (APDS.begin()) {
        Serial.println("Color & Distance Sensor Online!");
        colorDistStatus = 1;
    }
    else {
      Serial.println("Color & Distance Sensor Offline...");
      beep();
    }

    Serial.println();

    //delay(250);
    delay(20);
}
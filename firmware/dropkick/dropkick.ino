/* 
 * This file is part of the Kick distribution (https://github.com/rrainey/sidekick
 * Copyright (c) 2021 Riley Rainey
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

//#include <Adafruit_BME680.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <SD.h>

#define APP_STRING  "Sidekick, version 0.20"
#define LOG_VERSION 1
#define NMEA_APP_STRING "$PVER,\"Sidekick, version 0.20\",20"

/**
 * Fatal error codes (to be implemented)
 * 
 * 1 - no SD card
 * 2 - altitude sensor fail
 * 3 - IMU sensor fail
 */

/*
 * Operating mode
 */
#define OPS_FLIGHT       0 // normal mode (NOT YET IMPLEMENTED)
#define OPS_GROUND_TEST  1 // for testing; uses horizontal movement as an analogue to altitude changes

int nOpMode = OPS_GROUND_TEST;

#define TEST_SPEED_THRESHOLD_KTS  6.0

/*
 * Automatic jump logging
 * Generate a unique log file for each jump.
 * Log file contains GPS NMEA CSV records along with extra sensor data records in quasi-NMEA format
 * 
 * State 0: WAIT - gather baseline surface elevation information; compute HDOT_fps
 * 
 * State 1: IN_FLIGHT (enter this state when HDOT_fpm indicates >= 200 fpm climb), 
 *                   enable GPS (future versions), compute HDOT_fps, start logging if not already)
 *                   
 * State 2: LANDED1 (enter when altitude is within 1000 feet of baseline ground alt and 
 *                   HDOT_fpm < 50 fpm, start timer 1, log data)
 *                   
 * State 3: LANDED2  like state 2 - if any conditions are vioated, return to state 1(IN_FLIGHT), 
 *                   go to state 0 when timer 1 reaches 60 seconds, disable GPS (future versions), log data otherwise
 */

#define STATE_WAIT       0
#define STATE_IN_FLIGHT  1
#define STATE_LANDED_1   2
#define STATE_LANDED_2   3

int nAppState;

#define BLINK_STATE_OFF     0
#define BLINK_STATE_LOGGING 1
#define BLINK_STATE_BATTERY 2

int blinkState = BLINK_STATE_OFF;

bool bBatteryAlarm = false;
float measuredBattery_volts;

/*
 * LiPoly battery is rated at 3.7V
 */
#define LOWBATT_THRESHOLD 3.55

/*
 * Estimated MSL altitude, based on standard day pressure @ sea level
 */
int nH_feet = 0;

/*
 * Estimated rate of climb (fps)
 */
int nHDOT_fps = 0;

/*
 * Estimated ground elevation, ft
 * 
 * Computed while in WAIT state.
 */
int nHGround_feet = 0;

/*
 * I2C connection to the BME688 pressure/temp sensor
 */
//Adafruit_BME680 bme;

/*
 * I2C connection to the MPU-6050 IMU
 */
Adafruit_MPU6050 mpu;

bool mpu6050Present = false;

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp = dps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = dps.getPressureSensor();

/*
 * Adalogger M0 hardware definitions
 * 
 * See https://learn.adafruit.com/adafruit-feather-m0-adalogger/pinouts
 */
#define VBATPIN       A7
#define RED_LED       13
#define GREEN_SD_LED   8
#define SD_CHIP_SELECT 4

Adafruit_GPS GPS(&Wire);

File logFile;

char logpath[32];

/*
 * Records last millis() time when timers were updated in
 * the main loop.
 */
uint32_t lastTime_ms = 0;

/*
 * Set up timers
 * 
 * Timer 1: used in landing state machine (OFF initially)
 * 
 * Timer 2: periodic check of battery state
 * 
 * Timer 3: blink controller for RED LED  (OFF initially)
 * 
 * Timer 4: BME sensor logging interval timer  (OFF initially)
 * 
 * Timer 5: Periodic SD-card log file flushing  (OFF initially)
 */

#define TIMER1_INTERVAL_MS 30000

bool bTimer1Active = false;
int32_t timer1_ms = 0;

#define TIMER2_INTERVAL_MS 30000

bool bTimer2Active = true;
int32_t timer2_ms = TIMER2_INTERVAL_MS;

#define TIMER3_ON_INTERVAL_MS     750
#define TIMER3_OFF_INTERVAL_1_MS  750 // off interval when signaling battery low
#define TIMER3_OFF_INTERVAL_2_MS  (3000 - TIMER3_ON_INTERVAL_MS) // off interval for flight mode

bool bTimer3Active = false;
int32_t timer3_ms = 0;

#define TIMER4_INTERVAL_MS 25  // 40Hz

bool bTimer4Active = false;
int32_t timer4_ms = 0;

#define TIMER5_INTERVAL_MS 10000

bool bTimer5Active = false;
int32_t timer5_ms = 0;

int redLEDState = LOW;

// For debugging
bool printNMEA = false;

/**
 * Control RED (blinking) LED
 * This LED is locted to the left of the USB connector on
 * the Adalogger
 */
void setBlinkState( int newState ) {

  switch ( blinkState ) {

  case BLINK_STATE_OFF:
    // Was off, now on?
    if (newState != BLINK_STATE_OFF ) {
      bTimer3Active = true;
      timer3_ms = TIMER3_ON_INTERVAL_MS;
      redLEDState = HIGH;
    }
    break;
    
  case BLINK_STATE_LOGGING:
    if (newState == BLINK_STATE_BATTERY) {
      bTimer3Active = true;
      timer3_ms = TIMER3_ON_INTERVAL_MS;
      redLEDState = HIGH;
    }
    else if (newState == BLINK_STATE_OFF) {
        bTimer3Active = false;
        redLEDState = LOW;
     }
     break;
     
  case BLINK_STATE_BATTERY:
    if (newState == BLINK_STATE_OFF) {
      bTimer3Active = false;
      redLEDState = LOW;
    }
    else {
      // update state, but let blinking logic handle the transition
    }
    break;
  }

  // Update state and LED
  blinkState = newState;
  digitalWrite( RED_LED, redLEDState );
}

char * generateLogname(char *gname) 
{
    char * result = NULL;
    int i;
    for (i=0; true; i++) {
      sprintf (gname, "log%05d.txt", i);
   
      if (!SD.exists(gname)) {
          result = gname;
          break;
      }
    }

    return result;
}

void updateTestStateMachine() {

  /**
   * State machine appropriate for ground testing
   * TODO: support flight operating mode, OPS_FLIGHT
   */

  switch (nAppState) {

  case STATE_WAIT:
    if (GPS.speed >= TEST_SPEED_THRESHOLD_KTS) {

      Serial.println("Switching to STATE_IN_FLIGHT");
      
      // open log file
      generateLogname( logpath );
      logFile = SD.open( logpath, FILE_WRITE );

      logFile.println( NMEA_APP_STRING );

      // log data; jump to 5HZ logging
      //GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
      //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

      // Activate altitude / battery sensor logging
      bTimer4Active = true;
      timer4_ms = TIMER4_INTERVAL_MS;

      // Activate periodic log file flushing
      startLogFileFlushing();

      // Activate "in flight" LED blinking
      setBlinkState ( BLINK_STATE_LOGGING );
      
      nAppState = STATE_IN_FLIGHT;
    }
    break;

  case STATE_IN_FLIGHT:
    {

      if (GPS.speed < TEST_SPEED_THRESHOLD_KTS) {
        Serial.println("Switching to STATE_LANDED_1");
        nAppState = STATE_LANDED_1;
        timer1_ms = TIMER1_INTERVAL_MS;
        bTimer1Active = true;
      }
    }
    break;

  case STATE_LANDED_1:
    {

      if (GPS.speed >= TEST_SPEED_THRESHOLD_KTS) {
        Serial.println("Switching to STATE_IN_FLIGHT");
        nAppState = STATE_IN_FLIGHT;
        bTimer1Active = false;
      }
      else if (bTimer1Active && timer1_ms <= 0) {
        GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
        bTimer4Active = false;
        Serial.println("Switching to STATE_WAIT");
        setBlinkState ( BLINK_STATE_OFF );
        nAppState = STATE_WAIT;
        bTimer1Active = false;
        
        stopLogFileFlushing();
        logFile.close();
        
      }
      
    }
    break;

  case STATE_LANDED_2:
    {
      
      if (GPS.speed >= TEST_SPEED_THRESHOLD_KTS) {
        nAppState = STATE_IN_FLIGHT;
        bTimer1Active = false;
      }
      else if (bTimer1Active && timer1_ms <= 0) {
        GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
        bTimer4Active = false;
        setBlinkState ( BLINK_STATE_OFF );
        nAppState = STATE_WAIT;
        Serial.println("Switching to STATE_WAIT");
        bTimer1Active = false;
        bTimer5Active = false;

        stopLogFileFlushing();
        logFile.close();
      }
    }
    break;
  }
}

void setup() {

  blinkState = BLINK_STATE_OFF;
  
  //bTimer2Active = true;
  //timer2_ms = TIMER2_INTERVAL_MS;

  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);

  lastTime_ms = millis();

  // Wait (maximim of 30 seconds) for hardware serial to appear
  while (!Serial) {
    if (millis() - lastTime_ms > 30000) {
      break;
    }
  }
  
  Serial.begin(115200);

  Serial.println(APP_STRING);
  
  GPS.begin(0x42);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGAGSA);

  // For test operating mode, set update rate to 1HZ
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // include antenna status in stream
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

/*

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
*/

  delay(500);

  if (!SD.begin( SD_CHIP_SELECT )) {

    Serial.println("SD card initialization failed!");

    while (1);

  }

  delay(500);

  Wire.setClock( 400000 );

  Serial.println("Initialize peripheral ICs");

  if (mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire, 1 )) {
    Serial.println("MPU6050 present");

    mpu6050Present = true;

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  else {
    Serial.println("Failed to find MPU6050 chip");
    mpu6050Present = false;
  }

  delay(500);

  if (! dps.begin_I2C(DPS310_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Failed to find DPS310 chip");
    while (1) yield();
  }
  Serial.println("DPS310 present");

  dps.configurePressure(DPS310_16HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_16HZ, DPS310_16SAMPLES);

  /*
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  */
  //bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  //bme.setGasHeater(320, 150); // 320*C for 150 ms

  Serial.println("Switching to STATE_WAIT");
  nAppState = STATE_WAIT;

  /*
   * Set to 'true' do do some quick debugging of landing flow.
   */
  if (false) {
    Serial.println("Executing dummy flight run");
    Serial.println("Switching to STATE_IN_FLIGHT");
        
    // open log file
    generateLogname( logpath );
    logFile = SD.open( logpath, FILE_WRITE );

    logFile.println( NMEA_APP_STRING );

    logFile.flush();

    // log data; jump to 5HZ logging
    //GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

    // Activate altitude / battery sensor logging
    bTimer4Active = true;
    timer4_ms = TIMER4_INTERVAL_MS;

    // Activate periodic log file flushing
    startLogFileFlushing();

    // Activate "in flight" LED blinking
    setBlinkState ( BLINK_STATE_LOGGING );

    // force battery test timer to fire
    timer2_ms = 0;
    
    nAppState = STATE_IN_FLIGHT;
  }
  
  lastTime_ms = millis();
}

void loop() {

  char lastNMEA[MAXLINELENGTH];
  
  uint32_t curTime_ms = millis();

  uint32_t deltaTime_ms = curTime_ms - lastTime_ms;

  if ( deltaTime_ms > 0 ) {

    /*
     * Update active timer countdowns
     */
    if (bTimer1Active) {
      timer1_ms -= deltaTime_ms;
    }
  
    if (bTimer2Active) {
      timer2_ms -= deltaTime_ms;
    }
  
    if (bTimer3Active) {
      timer3_ms -= deltaTime_ms;
    }
  
    if (bTimer4Active) {
      timer4_ms -= deltaTime_ms;
    }

    if (bTimer5Active) {
      timer5_ms -= deltaTime_ms;
    }
    
    lastTime_ms = curTime_ms;

  }

  while ( GPS.available() > 0 ) {
  
    char c = GPS.read();
  
    /*
     * Received a NMEA record terminator?  Process it.
     */
    if ( GPS.newNMEAreceived() ) {
  
      strcpy( lastNMEA, GPS.lastNMEA() );

      if (logFile) {
        logFile.print( lastNMEA );
        flushLog();
      }
  
      if ( printNMEA ) {
        Serial.print( lastNMEA );
      }
  
      /*
       * Parse this latest arriving NMEA record. This will update appropriate state
       * variables in the GPS object. Calling GPS.lastNMEA() also has the effect of clearing the
       * flag indicating a new record arrived.
       */
      if (!GPS.parse( lastNMEA )) {
        // message not useful to us, or (less likely) had invalid checksum
      }
  
    }

  }

  /*
   * Processing tasks below are outside of the
   * GPS NMEA processing loop.
   */

   updateTestStateMachine();

   /*
    * 
    */

    sensors_event_t temp_event, pressure_event;
  
    if (dps.temperatureAvailable()) {
      dps_temp->getEvent(&temp_event);
      /*
      Serial.print(F("Temperature = "));
      Serial.print(temp_event.temperature);
      Serial.println(" *C");
      Serial.println();
      */
    }
  
    // Reading pressure also reads temp so don't check pressure
    // before temp!
    if (dps.pressureAvailable()) {
      
      dps_pressure->getEvent(&pressure_event);

      if (nAppState == STATE_WAIT) {
        nHGround_feet = dps.readAltitude() * 3.28084;
      }
      else {
        logFile.print("$PENV,");
        logFile.print(millis());
        logFile.print(",");
        logFile.print(pressure_event.pressure);
        logFile.print(",");
        logFile.print(dps.readAltitude());
        logFile.println();
      }
      
    }

  /*
   * RED LED Blink Logic
   * 
   * The RED LED will blink using different patterns to indicate
   * one of three states: Constant off, ON/OFF at 1.5Hz to indicates a low battery.
   * A 3-second blink is used to indicate flight mode.
   */
  if (bTimer3Active && timer3_ms <= 0) {
    
    redLEDState = (redLEDState == HIGH) ? LOW : HIGH;
    digitalWrite(RED_LED, redLEDState);

    if ( redLEDState == HIGH ) {
      timer3_ms = TIMER3_ON_INTERVAL_MS;
    }
    else {
      timer3_ms = bBatteryAlarm ? TIMER3_OFF_INTERVAL_1_MS : TIMER3_OFF_INTERVAL_2_MS;
    }
    
  }

  /*
   * Every 30 seconds, measure the battery state.
   * Blink red LED if low.
   */
  if (bTimer2Active && timer2_ms <= 0) {
    
    timer2_ms = TIMER2_INTERVAL_MS;
    
    measuredBattery_volts = analogRead(VBATPIN);
    measuredBattery_volts *= 2;    // we divided by 2, so multiply back
    measuredBattery_volts *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredBattery_volts /= 1024; // convert to voltage

    if ( measuredBattery_volts <= LOWBATT_THRESHOLD ) {
      bBatteryAlarm = true;
      setBlinkState ( BLINK_STATE_BATTERY );
    }
    else {
      if ( bBatteryAlarm ) {
        setBlinkState( (nAppState != STATE_WAIT) ? BLINK_STATE_LOGGING : BLINK_STATE_OFF );
      }
      bBatteryAlarm = false;
    }
  }

  /*
   * Log BME sensor information
   */
  if (bTimer4Active && timer4_ms <= 0) {
 
    //BMESample();
    IMU();

    timer4_ms = TIMER4_INTERVAL_MS;
  }
  
}

/*
void BMESample() {

  if (nAppState != STATE_WAIT) {
    
    if (! bme.performReading()) {
      Serial.println("Failed to perform BME888 reading :(");
      return;
    }

    if (logFile ) {
    
      logFile.print("$PENV,");
      logFile.print(millis());
      logFile.print(",");
      logFile.print(bme.temperature);
      logFile.print(",");
      logFile.print(bme.pressure / 100.0);
      logFile.print(",");
      logFile.print(bme.humidity);
      logFile.print(",");
      logFile.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
      logFile.print(",");
      logFile.print(bme.gas_resistance / 1000.0);
      logFile.print(",");
      logFile.println(measuredBattery_volts);

    }
  }
}
*/

/*
void PressureSample() {

  if (nAppState != STATE_WAIT) {
    
    if (! bme.performReading()) {
      Serial.println("Failed to perform BME888 reading :(");
      return;
    }

    if (logFile ) {
    
      logFile.print("$PENV,");
      logFile.print(millis());
      logFile.print(",");
      logFile.print(bme.temperature);
      logFile.print(",");
      logFile.print(bme.pressure / 100.0);
      logFile.print(",");
      logFile.print(bme.humidity);
      logFile.print(",");
      logFile.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
      logFile.print(",");
      logFile.print(bme.gas_resistance / 1000.0);
      logFile.print(",");
      logFile.println(measuredBattery_volts);

    }
  }
}
*/

void startLogFileFlushing()
{
  if (! bTimer5Active) {
    timer5_ms = TIMER5_INTERVAL_MS;
    bTimer5Active = true;
  }
}

void stopLogFileFlushing()
{
  if (bTimer5Active) {
    timer5_ms = 0;
    bTimer5Active = false;
  }

  if ( logFile ) {
    logFile.flush();
  }
}

void flushLog() {
    
  if (bTimer5Active && timer5_ms <= 0) {
    
    //Serial.println( "Log flushing" );

    timer5_ms = TIMER5_INTERVAL_MS;

    if ( logFile ) {
      logFile.flush();
    }
    
  }
}

void IMU() {

  if (mpu6050Present) {
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
  
    Serial.print(g.gyro.x); // rad per sec
    Serial.print(",");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print(g.gyro.z);
    Serial.println();
  
    if (logFile) {
      logFile.print("$PIMU,");
      logFile.print(millis());
      
      logFile.print(",");
      logFile.print(a.acceleration.x);  // m/s^2
      logFile.print(",");
      logFile.print(a.acceleration.y);
      logFile.print(",");
      logFile.print(a.acceleration.z);
      
      logFile.print(",");
      logFile.print(g.gyro.x); // rad per sec
      logFile.print(",");
      logFile.print(g.gyro.y);
      logFile.print(",");
      logFile.print(g.gyro.z);
      logFile.println();
    }
  }
}
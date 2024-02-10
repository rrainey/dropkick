/* 

  Test peripheral interfaces that we'll use with the Tempo hardware

  This application is an amalgam of several open source Arduino projects.
  It was used to test several different processor board with the Peakick sensor array.

  Elements based on code by Kris Winer
  "01/14/2022 Copyright Tlera Corporation"
  "Library may be used freely and without limit with proper attribution."
  see https://github.com/kriswiner/ICM42688

*/

#include <Arduino.h>
#include <stdio.h>
#include <avr/dtostrf.h>

#include "Adafruit_ZeroTimer.h"
#include "ICM42688.h"
#include "bmp3.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS myGNSS;

#define GPS_I2C_ADDR      0x42  // SAM-M10Q
#define ICM42688_ADDR     0x69

// "true" for extended Serial debugging messages
#define SerialDebug false

volatile bool alarmFlag = false;  // for RTC alarm interrupt

#if defined(ESP32)

#include <ESP32Time.h>
#include "ESP32TimerInterrupt.h"

// Sparkfun ESP32 Thing Plus C
#define RED_LED           13
#define GREEN_LED         12
#define CLKOUT            14        // ESP32 GPIO14 on the Sparkfun ESP32 Thing Plus C

// Unused in the current code, but I experimented with supplying the IMU clock from the ESP32
// using the ESP32 ledc API.
#define CPU_SUPPLIES_IMU_CLKIN  false
#define LEDC_CLKIN_CHANNEL    7     // channel (chosen randomly)
#define LEDC_CLKIN_RESOLUTION 8     // 8-BIT COUNTER (0..255)
#define LEDC_CLKIN_BASE_FREQ 32768  // Hz
#define LEDC_DUTY_CYCLE      128    // 50% duty cycle on a 8-bit counter

// ESP32 Thing Plus C/peackick-specific interrupt line connections
#define ICM42688_intPin1 32  // INT/INT1
#define ICM42688_intPin2 14  // INT2/CLKIN
#define MMC5983MA_intPin 15
#define BMP390_intPin    33
#endif

#if defined(STM32F4xx)
// Sparkfun STM-32 (DEV-17712)
#define RED_LED           13
#define GREEN_LED         12
#define CLKOUT            5        

#define CPU_SUPPLIES_IMU_CLKIN  false

// ESP32 Thing Plus C/peackick-specific interrupt line connections
#define ICM42688_intPin1 6  // INT/INT1
#define ICM42688_intPin2 5  // INT2/CLKIN
#define MMC5983MA_intPin 9
#define BMP390_intPin    10

#define IRAM_ATTR

#include <STM32RTC.h>

static STM32RTC::Hour_Format hourFormat = STM32RTC::HOUR_24;
static STM32RTC::AM_PM period = STM32RTC::AM;

STM32RTC& rtc = STM32RTC::getInstance();

void rtc_SecondsCB(void *data)
{
  UNUSED(data);
  alarmFlag = true;
}

#endif

// A small hack to automatically identify the Sparkfun SAMD21 board
//#if defined(_SAMD51J20A_)
#define DROPKICK_V4_SAM
//#endif

#if defined(DROPKICK_V4_SAM)

/**
 * DROPKICK V4 SAM SAMD21G18A 
 * 
 * BMP390    INT       D6
 * ICM42688  INT/INT1  D11
 * ICM42688  INT2      D12
 * 
 */
#define RED_LED           13
#define GREEN_LED         12
#define CLKOUT            5         // as a D51 output line     

#define CPU_SUPPLIES_IMU_CLKIN  false

// SAMD51 Thing Plus C/peackick-specific interrupt line connections
#define ICM42688_intPin1 digitalPinToInterrupt(11)   // INT/INT1
#define ICM42688_intPin2 digitalPinToInterrupt(12)   // INT2/CLKIN as an interrupt line
#define BMP390_intPin    digitalPinToInterrupt(6)

#define IRAM_ATTR

// This example can have just about any frequency for the callback
// automatically calculated!
float freq = 1000.0; // 1 KHz

// timer tester
Adafruit_ZeroTimer zerotimer = Adafruit_ZeroTimer(3);

void TC3_Handler() {
  Adafruit_ZeroTimer::timerHandler(3);
}

void TimerCallback0(void)
{
  alarmFlag = true;
}

#endif

#define bmp3_check_rslt(func, r)                     \
    if (r != BMP3_OK) {                              \
        Serial.println("Error returned from " func); \
    }

// global constants for 9 DoF fusion and AHRS
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError =
    pi * (40.0f /
          180.0f);  // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift =
    pi *
    (0.0f /
     180.0f);  // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;  // compute beta
float zeta =
    sqrtf(3.0f / 4.0f) *
    GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick
                    // scheme usually set to a small or zero value
float pitch, yaw, roll;  // absolute orientation
float a12, a22, a31, a32,
    a33;  // rotation matrix coefficients for Euler angles and gravity
          // components
float deltat = 0.0f,
      sum = 0.0f;  // integration interval for both filter schemes
uint32_t lastUpdate = 0,
         firstUpdate = 0;  // used to calculate integration interval
uint32_t Now = 0;          // used to calculate integration interval
float lin_ax, lin_ay,
    lin_az;  // linear acceleration (acceleration with gravity component
             // subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f,
                 0.0f};  // vector to hold integral error for Mahony method

extern void IRAM_ATTR MadgwickQuaternionUpdate(float ax, float ay, float az,
                                               float gx, float gy, float gz,
                                               float mx, float my, float mz);


/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G
      GFS_15_625DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS,
 GFS_500DPS, GFS_1000DPS, GFS_2000DPS AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz,
 AODR_50AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz,
      AODR_1kHz, AODR_2kHz, AODR_4kHz, AODR_8kHz, AODR_16kHz, AODR_32kHz
      GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz,
 GODR_1kHz, GODR_2kHz, GODR_4kHz, GODR_8kHz, GODR_16kHz, GODR_32kHz
*/
uint8_t Ascale = AFS_4G, Gscale = GFS_250DPS, AODR = AODR_200Hz,
        GODR = GODR_200Hz, aMode = aMode_LN, gMode = gMode_LN;

// must be set to match accel/gyro samples
float fSampleInterval_sec = 1 / 200.0f;

float aRes, gRes;  // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0.0f, 0.0f, 0.0f},
      gyroBias[3] = {0.0f, 0.0f, 0.0f};  // offset biases for the accel and gyro
int16_t accelDiff[3] = {0, 0, 0},
        gyroDiff[3] = {0, 0, 0};  // difference betwee ST and normal values
float STratio[7] = {
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f};    // self-test results for the accel and gyro
int16_t ICM42688Data[7];  // Stores the 16-bit signed sensor output
float imuTemp_C;  // Stores the real internal gyro temperature in degrees
                     // Celsius
float ax, ay, az, gx, gy,
    gz;  // variables to hold latest accel/gyro data values

volatile uint16_t imuISRCount = 0;

ICM42688 imu(&Wire);


#define SEALEVELPRESSURE_HPA (1013.25)

volatile bool pressureSampleAvailable = false;

void IRAM_ATTR BMPSampleReadyHandler() { pressureSampleAvailable = true; }

/*
 * BMP390 globals
 */
int8_t rslt;
uint16_t settings_sel;
struct bmp3_dev dev;
struct bmp3_data data = {0};
struct bmp3_settings settings = {0};
struct bmp3_status status = {{0}};

// ESP32 RTC API
uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;

#if defined(ESP32)
// UTC timezone
ESP32Time RTC(0);
ESP32Timer ITimer0(0);

// five second status reports to Serial
#define ESP32_TIMER_INTERVAL_USEC (5000 * 1000)
#endif

bool IRAM_ATTR TimerHandler(void* timerNo) {
    alarmFlag = true;
    return true;
}

void IRAM_ATTR myinthandler1() {
    ++imuISRCount;
}


void IRAM_ATTR alarmMatch() { alarmFlag = true; }

// number of IMU data read interrupts processed / interval
unsigned long imuIntCount = 0;
// track total number of times we missed processing after an IMU interrupt 
unsigned long imuISROverflow = 0;
// track number of loop() calls / interval
unsigned long loopCount = 0;
// track number of invalid samples received in FIFO
unsigned long imuInvalidSamples = 0;
// FIFO header does not match expected bit pattern
unsigned long imuFifoEmpty = 0;
// total number of packets processed from FIFO
unsigned long fifoTotal = 0;
// total number of passes through loop()
unsigned long loopTotal = 0;

bool ledState = false;
bool greenLedState = false;

float pressure_hPa = 1000.0f;
float bmpTemperature_degC = 15.0f;
float bmpAltitude_m = 0.0f;

float mx = 0.0001f, my=0.0f, mz=0.0f;

void setup() {

    delay(2000);

    Serial.begin(115200);
    while (!Serial) {
    }

    // Configure led
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);

#if !defined(ESP32)
    pinMode(ICM42688_intPin1, INPUT);
    pinMode(ICM42688_intPin2, INPUT);
    pinMode(BMP390_intPin, INPUT);
#endif

    digitalWrite(RED_LED, HIGH);

    Wire.begin();           // set master mode
    Wire.setClock(100000);  // I2C frequency at 100 kHz

    myGNSS.begin();

    myGNSS.setUART1Output(0);
    myGNSS.setUART2Output(0);

    //myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
    myGNSS.setI2COutput(COM_TYPE_NMEA);
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

    // Idle reporting will be at 0.5 Hz
    myGNSS.setMeasurementRate(2000);
    myGNSS.setNavigationRate(1);

    if (myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g) == false) 
    {
        Serial.println(F("Warning: setDynamicModel failed"));
    }
    else
    {
        Serial.println(F("GNSS Dynamic Platform Model set to AIRBORNE2g"));
    }

    //This will pipe all NMEA sentences to the serial port so we can see them
    //myGNSS.setNMEAOutputPort(Serial);

    myGNSS.setNavigationFrequency(1);

    Serial.println("ICM42688 ");
    byte ICM42688ID = imu.getChipID(); 


    Serial.println("BMP390");
    bmp3_begin(&dev, 0x76, &Wire);
    uint8_t BMP390ID = dev.chip_id;

    bool allSensorsAcknowledged =
        ICM42688ID == 0x47 && BMP390ID == 0x60;

    if (allSensorsAcknowledged) {
        Serial.println("All peripherals are operating");
        Serial.println(" ");

        digitalWrite(RED_LED, LOW);

        imu.reset();

        // set sensor resolutions for self test
        aRes = 4.0f / 32768.0f;
        gRes = 250.0f / 32768.0f;

        imu.selfTest(accelDiff, gyroDiff, STratio);
        Serial.println("Accel Self Test:");
        Serial.print("Ax diff: ");
        Serial.print(accelDiff[0] * aRes * 1000.0f);
        Serial.println(" mg");
        Serial.print("Ay diff: ");
        Serial.print(accelDiff[1] * aRes * 1000.0f);
        Serial.println(" mg");
        Serial.print("Az diff: ");
        Serial.print(accelDiff[2] * aRes * 1000.0f);
        Serial.println(" mg");
        Serial.println("Should be between 50 and 1200 mg");

        Serial.println("Gyro Self Test:");
        Serial.print("Gx diff: ");
        Serial.print(gyroDiff[0] * gRes);
        Serial.println(" dps");
        Serial.print("Gy diff: ");
        Serial.print(gyroDiff[1] * gRes);
        Serial.println(" dps");
        Serial.print("Gz diff: ");
        Serial.print(gyroDiff[2] * gRes);
        Serial.println(" dps");
        Serial.println("Should be > 60 dps");

        Serial.print("Ax ratio: ");
        Serial.print(STratio[1] * 100.0f, 0);
        Serial.println(" %");
        Serial.print("Ay ratio: ");
        Serial.print(STratio[2] * 100.0f, 0);
        Serial.println(" %");
        Serial.print("Az ratio: ");
        Serial.print(STratio[3] * 100.0f, 0);
        Serial.println(" %");
        Serial.println("Should be between 50 and 150%");

        Serial.println("Gyro Self Test:");
        Serial.print("Gx ratio: ");
        Serial.print(STratio[4] * 100.0f, 0);
        Serial.println(" %");
        Serial.print("Gy ratio: ");
        Serial.print(STratio[5] * 100.0f, 0);
        Serial.println(" %");
        Serial.print("Gz ratio: ");
        Serial.print(STratio[6] * 100.0f, 0);
        Serial.println(" %");
        Serial.println("Should be between 50 and 150%");
        delay(2000);

        // get sensor resolutions for user settings, only need to do this once
        aRes = imu.getAres(Ascale);
        gRes = imu.getGres(Gscale);

        // Configure IMU

        imu.init(Ascale, Gscale, AODR, GODR, aMode, gMode, CPU_SUPPLIES_IMU_CLKIN);

        Serial.println(
            "Sampling accel and gyro offset biases: keep device flat and "
            "motionless");
        delay(1000);

        imu.offsetBias(accelBias, gyroBias);
        Serial.println("accel biases (mg)");
        Serial.println(1000.0f * accelBias[0]);
        Serial.println(1000.0f * accelBias[1]);
        Serial.println(1000.0f * accelBias[2]);
        Serial.println("gyro biases (dps)");
        Serial.println(gyroBias[0]);
        Serial.println(gyroBias[1]);
        Serial.println(gyroBias[2]);
        delay(1000);

        Serial.println("");
        Serial.println("Magnetometer Calibration");

        digitalWrite(GREEN_LED, HIGH);


        // initialize BMP390 per datasheet, section 3.4.1 for "Drone"
        rslt = bmp3_init(&dev);
        bmp3_check_rslt("bmp3_init", rslt);

        settings.int_settings.drdy_en = BMP3_ENABLE;
        settings.press_en = BMP3_ENABLE;
        settings.temp_en = BMP3_ENABLE;

        settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
        settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
        settings.odr_filter.odr = BMP3_ODR_12_5_HZ;
        settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
      
        settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
                       BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
                       BMP3_SEL_DRDY_EN | BMP3_SEL_IIR_FILTER;

        rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
        bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

        attachInterrupt(BMP390_intPin, BMPSampleReadyHandler, RISING);

        settings.op_mode = BMP3_MODE_NORMAL;
        rslt = bmp3_set_op_mode(&settings, &dev);
        bmp3_check_rslt("bmp3_set_op_mode", rslt);

        digitalWrite(RED_LED, HIGH);
    } else {
        if (ICM42688ID != 0x6A) Serial.println(" ICM42688 not functioning!");
        if (BMP390ID != 0x60) Serial.println(" BMP390 not functioning!");
    }

    /* Set up the RTC alarm interrupt */

#if defined(ESP32)
    if (ITimer0.attachInterruptInterval(ESP32_TIMER_INTERVAL_USEC,
                                        TimerHandler)) {
        Serial.print(F("Starting RTC report interval clock; millis() = "));
        Serial.println(millis());
    } else {
        Serial.println(F("Can't set RTC."));
    }

    RTC.setTime(0, 0, 0, 1, 1, 2024);  // 1st Jan 2024 00:00:00

#endif

#if defined(SAMD51_THING_PLUS)
    TC.startTimer(1000000, myISR); // one second
#endif

#if defined(DROPKICK_V4_SAM)
    uint16_t divider  = 1;
    uint16_t compare = 0;
    tc_clock_prescaler prescaler = TC_CLOCK_PRESCALER_DIV1;

    // 1 Hz interrupt
    divider = 1024;
    prescaler = TC_CLOCK_PRESCALER_DIV1024;
    compare = (48000000/1024);

    zerotimer.enable(false);
    zerotimer.configure(prescaler,       // prescaler
            TC_COUNTER_SIZE_16BIT,       // bit width of timer/counter
            TC_WAVE_GENERATION_MATCH_PWM // frequency or PWM mode
            );

    zerotimer.setCompare(0, compare);
    zerotimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, TimerCallback0);
    zerotimer.enable(true);
#endif

#if defined(STM32F405xx)
        rtc.begin(hourFormat);

        rtc.setHours(0, period);
        rtc.setMinutes(0);
        rtc.setSeconds(0);

        rtc.setDay(1);
        rtc.setMonth(1);
        rtc.setYear(2024);

        rtc.attachSecondsInterrupt(rtc_SecondsCB);
#endif

    // activate CLKIN line as clock for IMU
    // Generate a 32.768K pps square wave on the IMU CLKIN pin
#if (CPU_SUPPLIES_IMU_CLKIN == true)
        int result = ledcSetup(LEDC_CLKIN_CHANNEL, 
                               LEDC_CLKIN_BASE_FREQ,
                               LEDC_CLKIN_RESOLUTION);
        ledcAttachPin(CLKOUT, LEDC_CLKIN_CHANNEL);
        Serial.print(F("ledcSetup() returned "));
        Serial.println(result);
        ledcWrite(LEDC_CLKIN_CHANNEL, LEDC_DUTY_CYCLE);
#endif

    imu.readIntStatus();  // clear ICM42688 data ready interrupt

    // ICM42688 INT1/INT interrupt 
    attachInterrupt(ICM42688_intPin1, myinthandler1, RISING); 

    imu.enableFifoMode( 3 );

    digitalWrite(GREEN_LED, LOW);
}

void loop() {

    ++loopCount;
    ++loopTotal;


#ifdef UNUSED_TAKE_SAMPLES_FROM_REGISTERS
    // ICM42688 has new sample?
    if (imuISRCount) {
        if (imuISRCount > 1) {
            ++imuISROverflow;
        }
        imuISRCount = 0;
        ++imuIntCount;

        deltat = fSampleInterval_sec;

        imu.readData(ICM42688Data);  // INT1 cleared on any read

        // Now we'll calculate the acceleration value into actual g's
        ax = (float)ICM42688Data[1] * aRes - accelBias[0];  
        ay = (float)ICM42688Data[2] * aRes - accelBias[1];
        az = (float)ICM42688Data[3] * aRes - accelBias[2];

        // Calculate the gyro value into actual degrees per second
        gx = (float)ICM42688Data[4] * gRes - gyroBias[0];
        gy = (float)ICM42688Data[5] * gRes - gyroBias[1];
        gz = (float)ICM42688Data[6] * gRes - gyroBias[2];

        // signs and axes if mag values corrected for IC placement on the
        // peakick V1 board
        MadgwickQuaternionUpdate(ay, ax, az, gy * pi / 180.0f, gx * pi / 180.0f,
                                 gz * pi / 180.0f, -mx, my, -mz);
    }
#endif

    if (imuISRCount > 0) {

        if (imuISRCount > 1) {
            ++imuISROverflow;
        }
        imuISRCount = 0;
        ++imuIntCount;

        // Read status bits to clear interrupt (see definition of INT_CONFIG0)
        uint8_t intStatus = imu.readIntStatus();

        deltat = fSampleInterval_sec;

        uint8_t status;
        icm42688::fifo_packet3 sampleBuf[140], *pBuf;
        uint16_t packetCount;

        status = imu.readFiFo(sampleBuf, &packetCount);
        if (status != ICM42688_RETURN_OK) {
            Serial.println("readFifo encountered an error");
        }

        fifoTotal += packetCount;

        for (int i = 0; i < packetCount; ++i) {
            pBuf = &sampleBuf[i];

            // valid sample?

            if ((pBuf->header & ICM42688_FIFO_HEADER_MSG) == 0) {
              if ((pBuf->header & (ICM42688_FIFO_HEADER_ACCEL | ICM42688_FIFO_HEADER_GYRO)) == 
                (ICM42688_FIFO_HEADER_ACCEL | ICM42688_FIFO_HEADER_GYRO)) {
                if (!(PACKET3_SAMPLE_MARKED_INVALID(pBuf->gx) ||
                      PACKET3_SAMPLE_MARKED_INVALID(pBuf->gy) ||
                      PACKET3_SAMPLE_MARKED_INVALID(pBuf->gz) ||
                      PACKET3_SAMPLE_MARKED_INVALID(pBuf->ax) ||
                      PACKET3_SAMPLE_MARKED_INVALID(pBuf->ay) ||
                      PACKET3_SAMPLE_MARKED_INVALID(pBuf->az))) {
                    ax = (float)pBuf->ax * aRes - accelBias[0];
                    ay = (float)pBuf->ay * aRes - accelBias[1];
                    az = (float)pBuf->az * aRes - accelBias[2];

                    // Convert the gyro value into degrees per second
                    gx = (float)pBuf->gx * gRes - gyroBias[0];
                    gy = (float)pBuf->gy * gRes - gyroBias[1];
                    gz = (float)pBuf->gz * gRes - gyroBias[2];

                    imuTemp_C = ((float)pBuf->temp / 2.07f) + 25.0f;

                    // signs and axes if mag values corrected for IC placement
                    // on the peakick V1 board
                    MadgwickQuaternionUpdate(ay, ax, az, gy * pi / 180.0f,
                                             gx * pi / 180.0f, gz * pi / 180.0f,
                                             -mx, my, -mz);
                } else {
                    imuInvalidSamples++;
                }
                } else {
                  imuInvalidSamples++;
                }
            } else {
                imuFifoEmpty++;
            }
        }
    }

    if ( pressureSampleAvailable ) {
        pressureSampleAvailable = false;

        rslt = bmp3_get_status(&status, &dev);
        bmp3_check_rslt("bmp3_get_status", rslt);

        /* Read temperature and pressure data iteratively based on data ready
         * interrupt */
        if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE)) {
            /*
             * First parameter indicates the type of data to be read
             * BMP3_PRESS_TEMP : To read pressure and temperature data
             * BMP3_TEMP       : To read only temperature data
             * BMP3_PRESS      : To read only pressure data
             */
            rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
            bmp3_check_rslt("bmp3_get_sensor_data", rslt);

            /* NOTE : Read status register again to clear data ready interrupt
             * status */
            rslt = bmp3_get_status(&status, &dev);
            bmp3_check_rslt("bmp3_get_status", rslt);

            pressure_hPa = data.pressure / 100.0f;
            bmpTemperature_degC = data.temperature;
            // altitude = 44330.0f * (1.0 - pow(Pressure / SEALEVELPRESSURE_HPA,
            // 0.1903));
        }
    }

    if (alarmFlag) {  // update RTC output (serial display) whenever the RTC
                      // alarm condition is achieved

        alarmFlag = false;

#ifdef notdef

        if (SerialDebug) {
            Serial.println("RTC:");
            Day = RTC.getDay();
            Month = RTC.getMonth();
            Year = RTC.getYear();
            Seconds = RTC.getSecond();
            Minutes = RTC.getMinute();
            Hours = RTC.getHour();
            if (Hours < 10) {
                Serial.print("0");
                Serial.print(Hours);
            } else
                Serial.print(Hours);
            Serial.print(":");
            if (Minutes < 10) {
                Serial.print("0");
                Serial.print(Minutes);
            } else
                Serial.print(Minutes);
            Serial.print(":");
            if (Seconds < 10) {
                Serial.print("0");
                Serial.println(Seconds);
            } else
                Serial.println(Seconds);

            Serial.print(Month);
            Serial.print("/");
            Serial.print(Day);
            Serial.print("/");
            Serial.println(Year);
            Serial.println(" ");
        }
#endif

        if (SerialDebug) {
            char buf[128];

            sprintf(buf, "a = {%5.1f, %5.1f, %5.1f} mg", 1000.0f * ax,
                    1000.0f * ay, 1000.0f * az);
            Serial.println(buf);

            sprintf(buf, "g = {%5.1f, %5.1f, %5.1f} deg/s", gx, gy, gz);
            Serial.println(buf);

            sprintf(buf, "M = {%5.1f, %5.1f, %5.1f} mG", 1000.0f * mx,
                    1000.0f * my, 1000.0f * mz);
            Serial.println(buf);

            sprintf(buf, "q = {%5.3f, %5.3f, %5.3f, %5.3f}", q[0], q[1], q[2],
                    q[3]);
            Serial.println(buf);
        }

        if (SerialDebug) {
            // Serial.print("Altimeter temperature = "); Serial.print(
            // Temperature, 2); Serial.println(" C"); // temperature in degrees
            // Celsius
            Serial.print("Altimeter temperature = ");
            Serial.print(9.0f * bmpTemperature_degC / 5.0f + 32.0f, 2);
            Serial.println(" F");
            Serial.print("Altimeter pressure = ");
            Serial.print(pressure_hPa, 2);
            Serial.println(" mbar");
            Serial.print("Altitude = ");
            Serial.print(bmpAltitude_m, 2);
            Serial.println(" feet");
            Serial.print("imuISRCount = ");
            Serial.print(imuISRCount);
            Serial.println("");
            Serial.print("  loopCount = ");
            Serial.print(loopCount);
            Serial.println("");
        }

        // Print temperature in degrees Centigrade
        if (SerialDebug) {
            Serial.print("Gyro temperature is ");
            Serial.print(imuTemp_C, 1);
            Serial.println(" degC");
        }

        a12 = 2.0f * (q[1] * q[2] + q[0] * q[3]);
        a22 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        a31 = 2.0f * (q[0] * q[1] + q[2] * q[3]);
        a32 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
        a33 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
        pitch = -asinf(a32);
        roll = atan2f(a31, a33);
        yaw = atan2f(a12, a22);
        pitch *= 180.0f / pi;
        yaw *= 180.0f / pi;
        yaw += 0.0f;  // no magnetic declination correction for now
        if (yaw < 0) yaw += 360.0f;  // Ensure yaw stays between 0 and 360
        roll *= 180.0f / pi;
        lin_ax = ax + a31;
        lin_ay = ay + a32;
        lin_az = az - a33;

        
#ifdef SPRINTF_HAS_FLOAT_SUPPORT
        sprintf(
            pbuf,
            "Yaw      Pitch    Roll (deg) Press(hPa)\n%6.2f   %6.2f   %6.2f     %7.1f",
            yaw, pitch, roll, pressure_hPa);
#else
        char pbuf[128];
        char xp[16], yp[16], zp[16], ap[16];
        sprintf(
            pbuf,
            "Yaw      Pitch    Roll (deg) Press(hPa)\n%s   %s   %s     %s",
            dtostrf(yaw,6,2,xp), dtostrf(pitch,6,2,yp), dtostrf(roll,6,2,zp), 
            dtostrf(pressure_hPa,7,1,ap));
#endif
        Serial.println(pbuf);

        sprintf(pbuf,
                "loopCount  IntCount ISROverflow  AvgFIFO  invalidFIFO\n %8d   %7d    %8d  %7d   %6d\n---",
                loopCount, imuIntCount, imuISROverflow, fifoTotal/loopTotal, imuInvalidSamples);
        Serial.println(pbuf);

        imuIntCount = 0;
        loopCount = 0;

        ledState = !ledState;
        digitalWrite(RED_LED, (ledState ? HIGH : LOW));
    }

    greenLedState = !greenLedState;
    digitalWrite(GREEN_LED, (greenLedState ? HIGH : LOW));
}
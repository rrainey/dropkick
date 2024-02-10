/* 
 * 
 * ICM42688 class with interrupt, FIFO support, and enhanced
 * error reporting in a few critical locations.
 *
 * Based on code created by Kris Winer
 * 01/14/2022 Copyright Tlera Corporation
 * Library may be used freely and without limit with attribution.
 * 
 * References
 * [1] ICM-422688-P datasheet, Version 1.8; https://invensense.tdk.com/download-pdf/icm-42688-p-datasheet/
 * [2] https://github.com/kriswiner/ICM42688
 *
 */

#ifndef ICM42688_h
#define ICM42688_h

#include "Arduino.h"
#include <Wire.h>

/* ICM42688 registers
https://media.digikey.com/pdf/Data%20Sheets/TDK%20PDFs/ICM-42688-P_DS_Rev1.2.pdf
*/
// User Bank 0
#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_DRIVE_CONFIG              0x13
#define ICM42688_INT_CONFIG                0x14
#define ICM42688_FIFO_CONFIG               0x16
#define ICM42688_TEMP_DATA1                0x1D
#define ICM42688_TEMP_DATA0                0x1E
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_ACCEL_DATA_X0             0x20
#define ICM42688_ACCEL_DATA_Y1             0x21
#define ICM42688_ACCEL_DATA_Y0             0x22
#define ICM42688_ACCEL_DATA_Z1             0x23
#define ICM42688_ACCEL_DATA_Z0             0x24
#define ICM42688_GYRO_DATA_X1              0x25
#define ICM42688_GYRO_DATA_X0              0x26
#define ICM42688_GYRO_DATA_Y1              0x27
#define ICM42688_GYRO_DATA_Y0              0x28
#define ICM42688_GYRO_DATA_Z1              0x29
#define ICM42688_GYRO_DATA_Z0              0x2A
#define ICM42688_TMST_FSYNCH               0x2B
#define ICM42688_TMST_FSYNCL               0x2C
#define ICM42688_INT_STATUS                0x2D
#define ICM42688_FIFO_COUNTH               0x2E
#define ICM42688_FIFO_COUNTL               0x2F
#define ICM42688_FIFO_DATA                 0x30
#define ICM42688_APEX_DATA0                0x31
#define ICM42688_APEX_DATA1                0x32
#define ICM42688_APEX_DATA2                0x33
#define ICM42688_APEX_DATA3                0x34
#define ICM42688_APEX_DATA4                0x35
#define ICM42688_APEX_DATA5                0x36
#define ICM42688_INT_STATUS2               0x37   
#define ICM42688_INT_STATUS3               0x38   
#define ICM42688_SIGNAL_PATH_RESET         0x4B
#define ICM42688_INTF_CONFIG0              0x4C
#define ICM42688_INTF_CONFIG1              0x4D
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_GYRO_CONFIG1              0x51
#define ICM42688_GYRO_ACCEL_CONFIG0        0x52
#define ICM42688_ACCEL_CONFIG1             0x53
#define ICM42688_TMST_CONFIG               0x54
#define ICM42688_APEX_CONFIG0              0x56
#define ICM42688_SMD_CONFIG                0x57
#define ICM42688_FIFO_CONFIG1              0x5F
#define ICM42688_FIFO_CONFIG2              0x60
#define ICM42688_FIFO_CONFIG3              0x61
#define ICM42688_FSYNC_CONFIG              0x62
#define ICM42688_INT_CONFIG0               0x63
#define ICM42688_INT_CONFIG1               0x64
#define ICM42688_INT_SOURCE0               0x65
#define ICM42688_INT_SOURCE1               0x66
#define ICM42688_INT_SOURCE3               0x68
#define ICM42688_INT_SOURCE4               0x69
#define ICM42688_FIFO_LOST_PKT0            0x6C
#define ICM42688_FIFO_LOST_PKT1            0x6D
#define ICM42688_SELF_TEST_CONFIG          0x70
#define ICM42688_WHO_AM_I                  0x75 // should return 0x47
#define ICM42688_REG_BANK_SEL              0x76

/*
 * INT_SOURCE0 bits
 */
#define ICM42688_INT_SOURCE0_RESET_DONE_INT1_EN  (1<<4)
#define ICM42688_INT_SOURCE0_UI_DRDY_INT1_EN     (1<<3)
#define ICM42688_INT_SOURCE0_FIFO_THS_INT1_EN    (1<<2)
#define ICM42688_INT_SOURCE0_FIFO_FULL_INT1_EN   (1<<1)
#define ICM42688_INT_SOURCE0_UI_AGC_RDY_INT1_EN  (1<<0)

/*
 * PWR_MGMT0 bits
 */
#define ICM42688_PWR_MGMT0_TEMP_DISABLE      (1<<5)
#define ICM42688_PWR_MGMT0_GYRO_IDLE_MASK    (1<<4)  // MASK TO ZERO TO POWER OFF
#define ICM42688_PWR_MGMT0_GYRO_MODE_LN      (3<<2)
#define ICM42688_PWR_MGMT0_ACCEL_MODE_LN     (3<<0)

/*
 * ICM42688_FIFO_CONFIG1
 * Register Name: FIFO_CONFIG1
 */
#define BIT_FIFO_CONFIG1_RESUME_PARTIAL_RD_POS  6
#define BIT_FIFO_CONFIG1_RESUME_PARTIAL_RD_MASK (0x1 << BIT_FIFO_CONFIG1_RESUME_PARTIAL_RD_POS)

/* FIFO_WM_GT_TH */
#define BIT_FIFO_CONFIG1_WM_GT_TH_POS  5
#define BIT_FIFO_CONFIG1_WM_GT_TH_MASK (0x1 << BIT_FIFO_CONFIG1_WM_GT_TH_POS)

typedef enum {
	ICM426XX_FIFO_CONFIG1_WM_GT_TH_EN  = (0x1 << BIT_FIFO_CONFIG1_WM_GT_TH_POS),
	ICM426XX_FIFO_CONFIG1_WM_GT_TH_DIS = (0x0 << BIT_FIFO_CONFIG1_WM_GT_TH_POS),
} ICM426XX_FIFO_CONFIG1_WM_GT_t;

/* FIFO_HIRES_EN */
#define BIT_FIFO_CONFIG1_HIRES_POS  4
#define BIT_FIFO_CONFIG1_HIRES_MASK (0x1 << BIT_FIFO_CONFIG1_HIRES_POS)

typedef enum {
	ICM426XX_FIFO_CONFIG1_HIRES_EN  = (0x1 << BIT_FIFO_CONFIG1_HIRES_POS),
	ICM426XX_FIFO_CONFIG1_HIRES_DIS = (0x0 << BIT_FIFO_CONFIG1_HIRES_POS),
} ICM426XX_FIFO_CONFIG1_HIRES_t;

/* FIFO_TMST_FSYNC_EN */
#define BIT_FIFO_CONFIG1_TMST_FSYNC_POS  3
#define BIT_FIFO_CONFIG1_TMST_FSYNC_MASK (0x1 << BIT_FIFO_CONFIG1_TMST_FSYNC_POS)

typedef enum {
	ICM426XX_FIFO_CONFIG1_TMST_FSYNC_EN  = (0x1 << BIT_FIFO_CONFIG1_TMST_FSYNC_POS),
	ICM426XX_FIFO_CONFIG1_TMST_FSYNC_DIS = (0x0 << BIT_FIFO_CONFIG1_TMST_FSYNC_POS),
} ICM426XX_FIFO_CONFIG1_TMST_FSYNC_t;

/* FIFO_TEMP_EN */
#define BIT_FIFO_CONFIG1_TEMP_POS  2
#define BIT_FIFO_CONFIG1_TEMP_MASK (0x1 << BIT_FIFO_CONFIG1_TEMP_POS)

typedef enum {
	ICM426XX_FIFO_CONFIG1_TEMP_EN  = (0x1 << BIT_FIFO_CONFIG1_TEMP_POS),
	ICM426XX_FIFO_CONFIG1_TEMP_DIS = (0x0 << BIT_FIFO_CONFIG1_TEMP_POS),
} ICM426XX_FIFO_CONFIG1_TEMP_t;

/* FIFO_GYRO_EN */
#define BIT_FIFO_CONFIG1_GYRO_POS  1
#define BIT_FIFO_CONFIG1_GYRO_MASK (0x1 << BIT_FIFO_CONFIG1_GYRO_POS)

typedef enum {
	ICM426XX_FIFO_CONFIG1_GYRO_EN  = (0x1 << BIT_FIFO_CONFIG1_GYRO_POS),
	ICM426XX_FIFO_CONFIG1_GYRO_DIS = (0x0 << BIT_FIFO_CONFIG1_GYRO_POS),
} ICM426XX_FIFO_CONFIG1_GYRO_t;

/* FIFO_ACCEL_EN*/
#define BIT_FIFO_CONFIG1_ACCEL_POS  0
#define BIT_FIFO_CONFIG1_ACCEL_MASK 0x1

typedef enum {
	ICM426XX_FIFO_CONFIG1_ACCEL_EN  = 0x01,
	ICM426XX_FIFO_CONFIG1_ACCEL_DIS = 0x00,
} ICM426XX_FIFO_CONFIG1_ACCEL_t;

typedef enum {
	ICM426XX_FIFO_CONFIG_BYPASS        = (0 << 6),
	ICM426XX_FIFO_CONFIG_STREAM        = (1<<6),
  ICM426XX_FIFO_CONFIG_STOP_ON_FULL  = (2<<6),
  ICM426XX_FIFO_CONFIG_STOP_ON_FULL2 = (3<<6),
} ICM426XX_FIFO_CONFIG_FIFO_MODE_t;


// User Bank 1
#define ICM42688_SENSOR_CONFIG0            0x03
#define ICM42688_GYRO_CONFIG_STATIC2       0x0B
#define ICM42688_GYRO_CONFIG_STATIC3       0x0C
#define ICM42688_GYRO_CONFIG_STATIC4       0x0D
#define ICM42688_GYRO_CONFIG_STATIC5       0x0E
#define ICM42688_GYRO_CONFIG_STATIC6       0x0F
#define ICM42688_GYRO_CONFIG_STATIC7       0x10
#define ICM42688_GYRO_CONFIG_STATIC8       0x11
#define ICM42688_GYRO_CONFIG_STATIC9       0x12
#define ICM42688_GYRO_CONFIG_STATIC10      0x13
#define ICM42688_XG_ST_DATA                0x5F
#define ICM42688_YG_ST_DATA                0x60
#define ICM42688_ZG_ST_DATA                0x61
#define ICM42688_TMSTAL0                   0x63
#define ICM42688_TMSTAL1                   0x64
#define ICM42688_TMSTAL2                   0x62
#define ICM42688_INTF_CONFIG4              0x7A
#define ICM42688_INTF_CONFIG5              0x7B
#define ICM42688_INTF_CONFIG6              0x7C

// User Bank 2
#define ICM42688_ACCEL_CONFIG_STATIC2      0x03
#define ICM42688_ACCEL_CONFIG_STATIC3      0x04
#define ICM42688_ACCEL_CONFIG_STATIC4      0x05
#define ICM42688_XA_ST_DATA                0x3B
#define ICM42688_YA_ST_DATA                0x3C
#define ICM42688_ZA_ST_DATA                0x3D

// User Bank 4
#define ICM42688_APEX_CONFIG1              0x40
#define ICM42688_APEX_CONFIG2              0x41
#define ICM42688_APEX_CONFIG3              0x42
#define ICM42688_APEX_CONFIG4              0x43
#define ICM42688_APEX_CONFIG5              0x44
#define ICM42688_APEX_CONFIG6              0x45
#define ICM42688_APEX_CONFIG7              0x46
#define ICM42688_APEX_CONFIG8              0x47
#define ICM42688_APEX_CONFIG9              0x48
#define ICM42688_ACCEL_WOM_X_THR           0x4A
#define ICM42688_ACCEL_WOM_Y_THR           0x4B
#define ICM42688_ACCEL_WOM_Z_THR           0x4C
#define ICM42688_INT_SOURCE6               0x4D
#define ICM42688_INT_SOURCE7               0x4E
#define ICM42688_INT_SOURCE8               0x4F
#define ICM42688_INT_SOURCE9               0x50
#define ICM42688_INT_SOURCE10              0x51
#define ICM42688_OFFSET_USER0              0x77
#define ICM42688_OFFSET_USER1              0x78
#define ICM42688_OFFSET_USER2              0x79
#define ICM42688_OFFSET_USER3              0x7A
#define ICM42688_OFFSET_USER4              0x7B
#define ICM42688_OFFSET_USER5              0x7C
#define ICM42688_OFFSET_USER6              0x7D
#define ICM42688_OFFSET_USER7              0x7E
#define ICM42688_OFFSET_USER8              0x7F

#define ICM42688_ADDRESS           0x68   // Address of ICM42688 accel/gyro when ADO = 0

#define AFS_2G  0x03
#define AFS_4G  0x02
#define AFS_8G  0x01
#define AFS_16G 0x00 // default

#define GFS_2000DPS   0x00   // default
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
#define GFS_125DPS    0x04
#define GFS_62_50DPS  0x05
#define GFS_31_25DPS  0x06
#define GFS_15_625DPS 0x07

// Low Noise mode
#define AODR_32kHz    0x01   
#define AODR_16kHz    0x02
#define AODR_8kHz     0x03
#define AODR_4kHz     0x04
#define AODR_2kHz     0x05
#define AODR_1kHz     0x06  // default
//Low Noise or Low Power modes
#define AODR_500Hz    0x0F
#define AODR_200Hz    0x07
#define AODR_100Hz    0x08
#define AODR_50Hz     0x09
#define AODR_25Hz     0x0A
#define AODR_12_5Hz   0x0B
// Low Power mode
#define AODR_6_25Hz   0x0C  
#define AODR_3_125Hz  0x0D
#define AODR_1_5625Hz 0x0E

#define GODR_32kHz  0x01   
#define GODR_16kHz  0x02
#define GODR_8kHz   0x03
#define GODR_4kHz   0x04
#define GODR_2kHz   0x05
#define GODR_1kHz   0x06 // default
#define GODR_500Hz  0x0F
#define GODR_200Hz  0x07
#define GODR_100Hz  0x08
#define GODR_50Hz   0x09
#define GODR_25Hz   0x0A
#define GODR_12_5Hz 0x0B

#define aMode_OFF 0x01
#define aMode_LP  0x02
#define aMode_LN  0x03

#define gMode_OFF 0x00
#define gMode_SBY 0x01
#define gMode_LN  0x03

// FIFO is empty
#define ICM42688_FIFO_HEADER_MSG                     (1<<7)
// packet contains ACCEL data
#define ICM42688_FIFO_HEADER_ACCEL                   (1<<6)
// packet contains GYRO data
#define ICM42688_FIFO_HEADER_GYRO                    (1<<5)
// 20-bit resolution in FIFO packet
#define ICM42688_FIFO_HEADER_20                      (1<<4)
#define ICM42688_FIFO_HEADER_TIMESTAMP_FSYNC_MASK    (3<<2)
#define ICM42688_FIFO_HEADER_ODR_ACCEL               (1<<1)
#define ICM42688_FIFO_HEADER_ODR_GYRO                (1<<0)

namespace icm42688 {

// FIFO data read formats (see [1], section 6)

typedef struct {
    uint8_t header;
    uint8_t ax_high;
    uint8_t ax_low;
    uint8_t ay_high;
    uint8_t ay_low;
    uint8_t az_high;
    uint8_t az_low;
    uint8_t gx_high;
    uint8_t gx_low;
    uint8_t gy_high;
    uint8_t gy_low;
    uint8_t gz_high;
    uint8_t gz_low;
    uint8_t temp;
    uint8_t timestamp_high;
    uint8_t timestamp_low;
} __attribute__((packed))  fifo_xfer_packet3;

typedef struct  {
    uint8_t header;
    uint8_t ax_high;    // AX[19:12]
    uint8_t ax_mid;     // AX[11:4]
    uint8_t ay_high;
    uint8_t ay_mid;
    uint8_t az_high;
    uint8_t az_mid;
    uint8_t gx_high;
    uint8_t gx_mid;
    uint8_t gy_high;
    uint8_t gy_mid;
    uint8_t gz_high;
    uint8_t gz_mid;
    uint8_t temp_high;
    uint8_t temp_low;
    uint8_t timestamp_high;
    uint8_t timestamp_low;
    uint8_t axgx_low;       // upper 4-bits AX[3:0], lower 4-bits GX[3:0]
    uint8_t aygy_low;
    uint8_t azgz_low;
} __attribute__((packed)) _fifo_xfer_packet4;

typedef struct fifo_packet3 {
    uint8_t header;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int8_t temp;
    uint16_t timestamp;
};

typedef struct fifo_packet4 {
    uint8_t header;
    int32_t ax; // unused[31:20], AX[19:0]
    int32_t ay; // unused[31:20], AY[19:0]
    int32_t az; // unused[31:20], AZ[19:0]
    int32_t gx; // unused[31:20], GX[19:0]
    int32_t gy; // unused[31:20], GY[19:0]
    int32_t gz; // unused[31:20], GZ[19:0]
    int16_t temp;
    uint16_t timestamp;
};

#define PACKET4_DATA_SIGN_BIT       (1<<19)
#define PACKET4_SIGN_EXTEND(x)      (long)(((x) & PACKET4_DATA_SIGN_BIT) ? (x) | 0xFFF0000 : (x))

// Invalid data readings are flagged where FIFO_HOLD_LAST_DATA_EN is disabled; 
// see [1] section 14.34
#define PACKET4_SAMPLE_MARKED_INVALID(x) ((x) == -524288)
#define PACKET3_SAMPLE_MARKED_INVALID(x) ((x) == -32768)

// Packet4 Accel to G 
#define PACKET4_AtoG(x)   (PACKET4_SIGN_EXTEND(x)/8192f)
// Packet 4 Gyro reading to deg/sec
#define PACKET4_GtoDPS(x)   (PACKET4_SIGN_EXTEND(x)/131f)

#define HEADER_MSG                   (1<<7)
#define HEADER_ACCEL                 (1<<6)
#define HEADER_GYRO                  (1<<5)
#define HEADER_20                    (1<<4)
#define HEADER_TIMESTAMP_FSYNC_MASK  (3<<2)
#define HEADER_TIMESTAMP_FSYNC_NONE  (0<<2)
#define HEADER_TIMESTAMP_FSYNC_ODR   (2<<2)
#define HEADER_TIMESTAMP_FSYNC_FSYNC (3<<2)
#define HEADER_ODR_ACCEL             (1<<1)
#define HEADER_ODR_GYRO              (1<<0)
};

#define ICM42688_RETURN_OK  0
#define ICM42688_RETURN_ERR 0xff


class ICM42688 {
   public:
    ICM42688(TwoWire* i2c);

    // Configure the device for operation
    void init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR,
              uint8_t aMode, uint8_t gMode, bool CLKIN);

    float getAres(uint8_t Ascale);
    float getGres(uint8_t Gscale);

    // return the chip identifier reported by the device; 0x47 for ICM-42688-P
    uint8_t getChipID(void);

    void offsetBias(float* dest1, float* dest2);
    void reset(void);

    // return interrupt status register bits
    uint8_t readIntStatus(void);
    
    void selfTest(int16_t* accelDiff, int16_t* gyroDiff, float* ratio);

    // Read a sample when not in FIFO mode
    void readData(int16_t* destination);


    void setTiltDetect(void);
    void setWakeonMotion(void);

    // Configure the device for the desired FIFO packet mode.
    //
    // Call after calling init(). Currently only modes 3 and 4 are supported.
    // returns 0 on success, 0xff on any error
    uint8_t enableFifoMode(uint8_t mode);
    //uint8_t disableFifoMode(void);

    // Read and unpack all packets available in FIFO
    // pPacketCount reflect number of packets read
    // returns ICM42688_RETURN_OK or ICM42688_RETURN_ERR if not configured for FIFO mode 3
    uint8_t readFiFo(icm42688::fifo_packet3 *pBuf,  uint16_t *pPacketCount);

    // Read and unpack all packets available in FIFO
    // returns ICM42688_RETURN_OK or ICM42688_RETURN_ERR if not configured for FIFO mode 4
    uint8_t readFiFo(icm42688::fifo_packet4 *pBuf, uint16_t *pPacketCount);

    uint16_t APEXStatus();

    // returns 1 if byte was read successfully
    uint8_t readByte(uint8_t subAddress, uint8_t *pData);

    // returns number of bytes actually read
    uint8_t readBytes(uint8_t subAddress, uint8_t count, uint8_t * pData);
    
    // return 1 if byte written, 0 otherwise
    uint8_t writeByte(uint8_t regAddr, uint8_t data);

    // Deprecated call
    void writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);

    // Deprecated call
    uint8_t readByte(uint8_t devAddr, uint8_t subAddress);

    // Deprecated call
    void readBytes(uint8_t devAddr, uint8_t subAddress, uint8_t count, uint8_t * dest);


   protected:
    float       _aRes, _gRes;
    // Wire interface used for I/O
    TwoWire*    _i2c;
    // I2C device address
    uint8_t     _i2c_address;

    // values: 0-4; only "3" supported currently; "0" not in FIFO mode
    uint8_t     _fifo_packet_config;

    // number of bytes expected in each FIFO packet, based on the configuration we set
    uint8_t     _fifo_packet_size;
};

#endif

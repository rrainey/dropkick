/* 
 * 
 * ICM42688 class with interrupt, FIFO support, and enhanced
 * error reporting in a few critical locations.
 *
 * Based on code created by Kris Winer
 * 01/14/2022 Copyright Tlera Corporation
 * Library may be used freely and without limit with attribution.
 */
#include "ICM42688.h"

ICM42688::ICM42688( TwoWire * i2c )
{

  _i2c = i2c;
  _fifo_packet_config = 0;
  _i2c_address = ICM42688_ADDRESS;
}


uint8_t ICM42688::getChipID()
{
  uint8_t temp = readByte(ICM42688_ADDRESS, ICM42688_WHO_AM_I);
  Serial.print("ChipID ");
  Serial.println(temp);
  return temp;
}


float ICM42688::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
    case AFS_2G:
         _aRes = 2.0f/32768.0f;
         return _aRes;
         break;
    case AFS_4G:
         _aRes = 4.0f/32768.0f;
         return _aRes;
         break;
    case AFS_8G:
         _aRes = 8.0f/32768.0f;
         return _aRes;
         break;
    case AFS_16G:
    default:
         _aRes = 16.0f/32768.0f;
         return _aRes;
         break;
  }
}

float ICM42688::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
     case GFS_15_625DPS:
          _gRes = 15.625f/32768.0f;
          return _gRes;
          break;
    case GFS_31_25DPS:
          _gRes = 31.25f/32768.0f;
          return _gRes;
          break;
    case GFS_62_50DPS:
          _gRes = 62.5f/32768.0f;
          return _gRes;
          break;
    case GFS_125DPS:
          _gRes = 125.0f/32768.0f;
          return _gRes;
          break;
    case GFS_250DPS:
          _gRes = 250.0f/32768.0f;
          return _gRes;
          break;
    case GFS_500DPS:
          _gRes = 500.0f/32768.0f;
          return _gRes;
          break;
    case GFS_1000DPS:
         _gRes = 1000.0f/32768.0f;
         return _gRes;
         break;
    case GFS_2000DPS:
    default:
          _gRes = 2000.0f/32768.0f;
         return _gRes;
         break;
  }
}


void ICM42688::reset()
{
  // reset device
  writeByte( ICM42688_DEVICE_CONFIG,  0x01);
  delay(1);
}


uint8_t ICM42688::readIntStatus()
{
  uint8_t temp;
  uint8_t status = readByte(ICM42688_INT_STATUS, &temp);  
  if (status != 1) {
    Serial.println("DRStatus read failed");
  }
  return temp;
}


void ICM42688::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR, uint8_t aMode, uint8_t gMode, bool CLKIN)
{
  writeByte( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0

  writeByte( ICM42688_PWR_MGMT0,  gMode << 2 | aMode); // set accel and gyro modes
  delay(1);

  writeByte( ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR); // set accel ODR and FS
  
  writeByte( ICM42688_GYRO_CONFIG0,  Gscale << 5 | GODR); // set gyro ODR and FS
  
  //RBR: used defaults since we're using LN data rates
  //writeByte( ICM42688_GYRO_ACCEL_CONFIG0,  0x44); // set gyro and accel bandwidth to ODR/10
 
   // interrupt handling
  writeByte( ICM42688_INT_CONFIG, 0x18 | 0x03 );      // push-pull, pulsed, active HIGH interrupts

  uint8_t temp;
  uint8_t status = readByte(ICM42688_INT_CONFIG1, &temp);

  temp &= ~(1<<4);              // clear bit 4 to allow async interrupt reset (required for proper interrupt operation)
  if (AODR <= AODR_4kHz || GODR <= GODR_4kHz) {
    temp |= (1<<5) | (1<<6);    // shorten interrupt pulse/deassertion duration for higher data rates (required; see 14.5)
  }
  writeByte( ICM42688_INT_CONFIG1, temp);
  writeByte( ICM42688_INT_SOURCE0, ICM42688_INT_SOURCE0_UI_DRDY_INT1_EN );
  
  // Use external clock source
  if(CLKIN) {
    writeByte( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0

    writeByte( ICM42688_INTF_CONFIG1, 0x95); // enable RTC

    writeByte( ICM42688_REG_BANK_SEL, 0x01); // select register bank 1

    writeByte( ICM42688_INTF_CONFIG5, 0x04); // use CLKIN as clock source
  }

    writeByte( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
}


void ICM42688::selfTest(int16_t * accelDiff, int16_t * gyroDiff, float * ratio)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelSTest[3] = {0, 0, 0}, gyroSTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

  writeByte( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
 
  // GYRO AND ACCEL TO LOW-NOIS MODE
  writeByte( ICM42688_PWR_MGMT0,  
    ICM42688_PWR_MGMT0_GYRO_MODE_LN | ICM42688_PWR_MGMT0_ACCEL_MODE_LN );
  delay(1);

  writeByte( ICM42688_ACCEL_CONFIG0, AFS_4G << 5 | AODR_1kHz);  // FS = 2
  
  writeByte( ICM42688_GYRO_CONFIG0,  GFS_250DPS << 5 | GODR_1kHz);  // FS = 3
 
  writeByte( ICM42688_GYRO_ACCEL_CONFIG0,  0x44); // set gyro and accel bandwidth to ODR/10

  readData(temp);
  accelNom[0] = temp[1];
  accelNom[1] = temp[2];
  accelNom[2] = temp[3];
  gyroNom[0]  = temp[4];
  gyroNom[1]  = temp[5];
  gyroNom[2]  = temp[6];
  
  writeByte( ICM42688_SELF_TEST_CONFIG, 0x78); // accel self test
  delay(100); // let accel respond
  readData(temp);
  accelSTest[0] = temp[1];
  accelSTest[1] = temp[2];
  accelSTest[2] = temp[3];

  writeByte( ICM42688_SELF_TEST_CONFIG, 0x07); // gyro self test
  delay(100); // let gyro respond
  readData(temp);
  gyroSTest[0] = temp[4];
  gyroSTest[1] = temp[5];
  gyroSTest[2] = temp[6];

  writeByte( ICM42688_SELF_TEST_CONFIG, 0x00); // normal mode

  accelDiff[0] = accelSTest[0] - accelNom[0];
  if(accelDiff[0] < 0) accelDiff[0] *= -1;        // make sure difference values are positive
  accelDiff[1] = accelSTest[1] - accelNom[1];
  if(accelDiff[1] < 0)accelDiff[1] *= -1;
  accelDiff[2] = accelSTest[2] - accelNom[2];
  if(accelDiff[2] < 0) accelDiff[2] *= -1;
  gyroDiff[0] = gyroSTest[0] - gyroNom[0];
  if(gyroDiff[0] < 0) gyroDiff[0] *= -1;
  gyroDiff[1] = gyroSTest[1] - gyroNom[1];
  if(gyroDiff[1] < 0) gyroDiff[1] *= -1;
  gyroDiff[2] = gyroSTest[2] - gyroNom[2];
  if(gyroDiff[2] < 0) gyroDiff[2] *= -1;
  
  writeByte( ICM42688_REG_BANK_SEL, 0x01); // select register bank 1

  temp[4] = readByte(ICM42688_ADDRESS, ICM42688_XG_ST_DATA); // gyro self-test output generated during manufacturing tests
  temp[5] = readByte(ICM42688_ADDRESS, ICM42688_YG_ST_DATA);
  temp[6] = readByte(ICM42688_ADDRESS, ICM42688_ZG_ST_DATA);

  writeByte( ICM42688_REG_BANK_SEL, 0x02); // select register bank 2

  temp[1] = readByte(ICM42688_ADDRESS, ICM42688_XA_ST_DATA);  // accel self-test output generated during manufacturing tests
  temp[2] = readByte(ICM42688_ADDRESS, ICM42688_YA_ST_DATA);
  temp[3] = readByte(ICM42688_ADDRESS, ICM42688_ZA_ST_DATA);

  ratio[1] = accelDiff[0] / (1310.0f * powf(1.01f, temp[1] - 1) + 0.5f);
  ratio[2] = accelDiff[1] / (1310.0f * powf(1.01f, temp[2] - 1) + 0.5f);
  ratio[3] = accelDiff[2] / (1310.0f * powf(1.01f, temp[3] - 1) + 0.5f);
  ratio[4] = gyroDiff[0]  / (2620.0f * powf(1.01f, temp[4] - 1) + 0.5f);
  ratio[5] = gyroDiff[1] /  (2620.0f * powf(1.01f, temp[5] - 1) + 0.5f);
  ratio[6] = gyroDiff[2] /  (2620.0f * powf(1.01f, temp[6] - 1) + 0.5f);
  
  writeByte( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
}


void ICM42688::offsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};
    
  for (int ii = 0; ii < 128; ii++)
  {
    readData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    delay(50);
  }

  dest1[0] = sum[1]*_aRes/128.0f;
  dest1[1] = sum[2]*_aRes/128.0f;
  dest1[2] = sum[3]*_aRes/128.0f;
  dest2[0] = sum[4]*_gRes/128.0f;
  dest2[1] = sum[5]*_gRes/128.0f;
  dest2[2] = sum[6]*_gRes/128.0f;

  if(dest1[0] > 0.8f)  {dest1[0] -= 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest1[0] < -0.8f) {dest1[0] += 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest1[1] > 0.8f)  {dest1[1] -= 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest1[1] < -0.8f) {dest1[1] += 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest1[2] > 0.8f)  {dest1[2] -= 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
  if(dest1[2] < -0.8f) {dest1[2] += 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
/*
  // load offset biases into offset registers (optional, comment out if not desired)
  temp[0] = (int16_t) (-dest1[0] / 0.00048828125f); // Ax 0.5 mg resolution
  temp[1] = (int16_t) (-dest1[1] / 0.00048828125f); // Ay
  temp[2] = (int16_t) (-dest1[2] / 0.00048828125f); // Az
  temp[3] = (int16_t) (-dest2[0] / 0.03125f);       // Gx 1/32 dps resolution
  temp[4] = (int16_t) (-dest2[1] / 0.03125f);       // Gy
  temp[5] = (int16_t) (-dest2[2] / 0.03125f);       // Gz

  writeByte( ICM42688_REG_BANK_SEL, 0x04); // select register bank 4

  writeByte( ICM42688_OFFSET_USER5,  temp[0] & 0x00FF); // lower Ax byte
  writeByte( ICM42688_OFFSET_USER6,  temp[1] & 0x00FF); // lower Ay byte
  writeByte( ICM42688_OFFSET_USER8,  temp[2] & 0x00FF); // lower Az byte
  writeByte( ICM42688_OFFSET_USER2,  temp[4] & 0x00FF); // lower Gy byte
  writeByte( ICM42688_OFFSET_USER3,  temp[5] & 0x00FF); // lower Gz byte
  writeByte( ICM42688_OFFSET_USER0,  temp[3] & 0x00FF); // lower Gx byte
  writeByte( ICM42688_OFFSET_USER4,  (temp[0] & 0x0F00) >> 4 | (temp[5] & 0x0F00) >> 8); // upper Ax and Gz bytes
  writeByte( ICM42688_OFFSET_USER7,  (temp[2] & 0x0F00) >> 4 | (temp[1] & 0x0F00) >> 8); // upper Az and Ay bytes
  writeByte( ICM42688_OFFSET_USER1,  (temp[4] & 0x0F00) >> 4 | (temp[3] & 0x0F00) >> 8); // upper Gy and Gx bytes
  
  writeByte( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
  */
}


void ICM42688::readData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readBytes(ICM42688_ADDRESS, ICM42688_TEMP_DATA1, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;   
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;  
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;  
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ; 
}

uint8_t ICM42688::enableFifoMode(uint8_t mode) {

  uint8_t status;
  uint8_t data;
  uint8_t actual;

  _fifo_packet_config = 0;

  if (mode <=2 || mode > 4) {
    return ICM42688_RETURN_ERR;
  }

  /*
   * Enable Acc, gyro, and temp sensors in FIFO.
   * Enable threshold interrupts.
   * Partial reads are enabled (since Wire API can't read more than 32 bytes in one chunk)
   */
  actual = readByte(ICM42688_FIFO_CONFIG1, &data);
  if (actual != 1) {
    Serial.println("assertion error: cannot read ICM42688_FIFO_CONFIG1");
    data = 0;
  }
  data |= (uint8_t)ICM426XX_FIFO_CONFIG1_ACCEL_EN;
	data |= (uint8_t)ICM426XX_FIFO_CONFIG1_GYRO_EN;
  data |= (uint8_t)ICM426XX_FIFO_CONFIG1_TEMP_EN;
  data |= (uint8_t)ICM426XX_FIFO_CONFIG1_WM_GT_TH_EN;
  data |= (uint8_t)BIT_FIFO_CONFIG1_RESUME_PARTIAL_RD_MASK;

  //data |= (uint8_t)ICM426XX_FIFO_CONFIG1_TMST_FSYNC_EN;

  switch (mode) {
    case 3:
      data &= ~((uint8_t)ICM426XX_FIFO_CONFIG1_HIRES_EN);
      _fifo_packet_size = 16;
      _fifo_packet_config = 3;
      break;
    case 4:
      data |= (uint8_t)ICM426XX_FIFO_CONFIG1_HIRES_EN;
      _fifo_packet_size = 20;
      _fifo_packet_config = 4;
      break;
    default:
      return ICM42688_RETURN_ERR;
  }
  writeByte(ICM42688_FIFO_CONFIG1, data);

  // set FIFO watermark to 2 records
  uint16_t watermark = 2 /** _fifo_packet_size*/;
  writeByte(ICM42688_FIFO_CONFIG2, watermark & 0xFF);
  writeByte(ICM42688_FIFO_CONFIG3, watermark >> 8);

  // switch to FIFO watermark and RESET for INT1
  writeByte(ICM42688_INT_SOURCE0, 
    ICM42688_INT_SOURCE0_RESET_DONE_INT1_EN | ICM42688_INT_SOURCE0_FIFO_THS_INT1_EN);

  // enable FIFO streaming (i.e., data now comes via FIFO)
  writeByte(ICM42688_FIFO_CONFIG, ICM426XX_FIFO_CONFIG_STREAM);

  return ICM42688_RETURN_OK;
}

uint8_t ICM42688::readFiFo(icm42688::fifo_packet3 *pBuf, uint16_t *pPacketCount) {
  uint8_t actual; 
  uint16_t bytes;
  uint8_t buffer[2];
  icm42688::fifo_xfer_packet3 buf;
  icm42688::fifo_xfer_packet3 *p3;
  icm42688::fifo_packet3 *p3Dest;

  if (_fifo_packet_config != 3) {
    return ICM42688_RETURN_ERR;
  }

  actual = readBytes(ICM42688_FIFO_COUNTH, 2, buffer);
  if (actual != 2) {
    return ICM42688_RETURN_ERR;
  }
  bytes = ((uint16_t)buffer[0] <<8) | buffer[1];

  if (bytes == 0) {
    *pPacketCount = 0;
    return ICM42688_RETURN_OK;
  }

  uint16_t packets = bytes / _fifo_packet_size;
  uint16_t actualPackets = 0;

  if (packets > 139) {
    Serial.print("assertion error: greater than 139 packets in FIFO: ");
    Serial.println(packets);
    packets = 130;
  }

  p3Dest = pBuf;

  for (int i = 0; i < packets; ++i) {
      p3 = &buf;

      // It's tempting to read the whole available FIFO in a single request, but the standard
      // Arduino Wire class is limited to 32 bytes per request -- so, we'll instead read
      // one packet at a time.
      actual = readBytes(ICM42688_FIFO_DATA, _fifo_packet_size, (uint8_t*) p3);
      if (actual == _fifo_packet_size) {

        p3Dest->ax = ((int16_t)p3->ax_high << 8) | p3->ax_low;
        p3Dest->ay = ((int16_t)p3->ay_high << 8) | p3->ay_low;
        p3Dest->az = ((int16_t)p3->az_high << 8) | p3->az_low;
        p3Dest->gx = ((int16_t)p3->gx_high << 8) | p3->gx_low;
        p3Dest->gy = ((int16_t)p3->gy_high << 8) | p3->gy_low;
        p3Dest->gz = ((int16_t)p3->gz_high << 8) | p3->gz_low;
        p3Dest->timestamp =
            (uint16_t)((uint16_t)p3->timestamp_high << 8 | p3->timestamp_low);
        p3Dest->header = p3->header;
        p3Dest->temp = p3->temp;

        ++p3Dest;

        ++ actualPackets;
      }
      else {
          Serial.print("read request of a FIFO packet failed: requested ");
          Serial.print( _fifo_packet_size );
          Serial.print(", actual ");
          Serial.print( actual );

          // experimental: just return what we have after an error
          if (actual == 0) {
            *pPacketCount = actualPackets;
            return ICM42688_RETURN_OK;
          }
      }
  }

  *pPacketCount = actualPackets;

  return ICM42688_RETURN_OK;
}


void ICM42688::setTiltDetect()
{
  writeByte( ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
  writeByte( ICM42688_APEX_CONFIG4, 0x00); // immediately interrupt on tilt

  writeByte( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
  writeByte( ICM42688_APEX_CONFIG0, 0x02); // DMP power save off, DMP ODR at 50 Hz
  writeByte( ICM42688_SIGNAL_PATH_RESET, 0x20); // DMP memory reset
  delay(1);
  writeByte( ICM42688_SIGNAL_PATH_RESET, 0x40); // DMP enable
  writeByte( ICM42688_APEX_CONFIG0, 0x12); // Tilt detect enable
  delay(2000); // wait for tilt to stabilize

  writeByte( ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
  writeByte( ICM42688_INT_SOURCE7, 0x08); // route tilt interrupt to INT2

  writeByte( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
}


void ICM42688::setWakeonMotion()
{
  writeByte( ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
  writeByte( ICM42688_ACCEL_WOM_X_THR, 0x50); // 80 x 3.9 mg is ~312 mg
  writeByte( ICM42688_ACCEL_WOM_Y_THR, 0x50); // 80 x 3.9 mg is ~312 mg
  writeByte( ICM42688_ACCEL_WOM_Z_THR, 0x50); // 80 x 3.9 mg is ~312 mg

  writeByte( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
  writeByte( ICM42688_INT_SOURCE4, 0x07);  // route wake on motion for any axis to INT2
  writeByte( ICM42688_SMD_CONFIG, 0x05);   // differential mode, WOM interrupt ORed with other APEX interrupts, enable WOM function (bit 0)
}


uint16_t ICM42688::APEXStatus()
{
  uint8_t status2 =  readByte(ICM42688_ADDRESS, ICM42688_INT_STATUS2);
  uint8_t status3 =  readByte(ICM42688_ADDRESS, ICM42688_INT_STATUS3); 
  uint16_t temp = ((uint16_t) status2 << 8) | status3; 
  return temp;
}

// return 1 of byte read, 0 otherwise
uint8_t ICM42688::readByte(uint8_t subAddress, uint8_t *pData)
{

  _i2c->beginTransmission(_i2c_address);    // Initialize the Tx buffer
  _i2c->write(subAddress);                  // Put slave register address in Tx buffer
  _i2c->endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t count = _i2c->requestFrom(_i2c_address, (uint8_t) 1);  // Read one byte from slave register address  
  if (count) {
    *pData = _i2c->read();                      // Fill Rx buffer with result
  }
  return count;
}

// On error, all available data bytes will be copied from the Wire object.
// Returns actual count of bytes read
uint8_t ICM42688::readBytes(uint8_t subAddress, uint8_t count, uint8_t * pData)
{  
  _i2c->beginTransmission(_i2c_address);   // Initialize the Tx buffer
  _i2c->write(subAddress);            // Put slave register address in Tx buffer
  _i2c->endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t actual = _i2c->requestFrom(_i2c_address, count);  // Read bytes from slave register address 
  for (uint8_t i=0; i<actual; ++i) {
        pData[i++] = _i2c->read();
  }
  return actual;
}

// return 1 if byte was written; zero otherwise
uint8_t ICM42688::writeByte(uint8_t regAddr, uint8_t data)
{
  uint8_t count;
  _i2c->beginTransmission(_i2c_address);  // Initialize the Tx buffer
  count = _i2c->write(regAddr);           // Put slave register address in Tx buffer
  if (count == 1) {
    count = _i2c->write(data);            // Put data in Tx buffer
  }
  count = _i2c->endTransmission();        // send it
  return count;
}

uint8_t ICM42688::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data = 0;                         
  _i2c->beginTransmission(address);
  _i2c->write(subAddress);
  _i2c->endTransmission(false);
  _i2c->requestFrom(address, (uint8_t) 1);
  data = _i2c->read();
  return data;
  
}

void ICM42688::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  _i2c->beginTransmission(address);   // Initialize the Tx buffer
  _i2c->write(subAddress);            // Put slave register address in Tx buffer
  _i2c->endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  _i2c->requestFrom(address, count);  // Read bytes from slave register address 
  while (_i2c->available()) {
        dest[i++] = _i2c->read(); 
  }   // Put read results in the Rx buffer
}

// Legacy calling signature
void ICM42688::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
  _i2c->beginTransmission(devAddr);  // Initialize the Tx buffer
  _i2c->write(regAddr);           // Put slave register address in Tx buffer
  _i2c->write(data);                 // Put data in Tx buffer
  _i2c->endTransmission();           // Send the Tx buffer
}
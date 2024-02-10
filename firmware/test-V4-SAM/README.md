# TEST-V4-SAM

## A, Arduino sketch designed to strees test the Dropkick SAM V4 board.
This sketch is designed for checkout and stress testing of the
Dropkick SAM V4 board.  This revised board design switched to newer GNSS, IMU, and barometric pressure sensors, so this sketh will not work with earlier variants.

Arduino board configuration settings:

```json
{
    "configuration": "opt=fastest,usbstack=arduino,debug=off",
    "board": "adafruit:samd:adafruit_trinket_m0",
    "sketch": "test-V4-SAM.ino",
    "port": "COM14"
}
```

Your USB port assignment will vary, of course.

## Notes and conclusions
The Dropkick board uses I2C to communicate with all sensors. The newer "peakick" board project does the same.  Both boards exhibit intermittent issues when trying to interact
with sensors at higher sampling rates, probably due to intermittent I2C bus issues.  I did spend some time tuning the I2C bus configuration of both boards, mostly by optimizing the pull-up resistor values.  After tuning, the reliability improved, but intermittent issues persist. Based on that, I have decided that the completely new Tempo board design will use an SPI rather than I2C for the ICM42688 IMU (see https://github.com/rrainey/tempo).

## Credits
This sketch use a custom ICM42688 library.  This IMU driver code is based upon code published by Kris Winer, copyrighted by Tlera Corporation.  I added extensions to add support for the ICM42688 FIFO, interrupts, and allow for better error reporting.  The original code was marked as 
"Library may be used freely and without limit with attribution".  Thank you, Kris and Tlera.  (see https://github.com/kriswiner/ICM42688)

This sketch also includes BMP3-series driver reference code from Bosch Sensortec GmbH. See [BMP3-LICENSE](./BMP3-LICENSE) and https://github.com/boschsensortec/BMP3-Sensor-API for more information.
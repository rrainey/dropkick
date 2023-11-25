# Dropkick CHANGELOG

## 2023-10-15 CAM V5 Assembly
- assembled and tested first v5-cam board - defective CAM receiver part

## 2023-10-01 CAM V5 Board
- Use internal antenna for the CAM-M8C GNSS Module
- revise CAM-M8C layout to get a cleaner ground plane
- add 30 ohm resistors on CAM-M8C I2C lines to reduce noise
- revise MPU-6050 PCB layout to more closely match the datasheet
- add header lines for SWD programming, switch to 2.0 mm spacing on header

## 2023-09-10 SAM V4 PCB Board
I decided to revise the SAM variant PCB to use more modern ICs:

- Change GNSS to the u-blox SAM-M10Q (the M10Q is pin compatible with the M8Q)
- Change IMU to TDK InvenSense ICM-42688
- Add hardware interrupt lines from the IMU
- Change barometric sensor to the Bosch BMP-390
- Switch to a USB-C connector

## 2022-09-03
I have resolved the buffering issue in the GNSS reporting stream and continue to work on developing analysis tools using Octave.

## 2022-08-18
I have collected the first log files from several jumps. GPS data is usable, although there are intermittent u-blox buffering errors
 interspersed in these logs. I have been exploring the feasibility of pose reconstruction from IMU log data.
 I am using GNU Octave for this investigation. The code resides in the ./analysis subdirectory.  It should be considered entirely experimental
 at this stage - don't trust the accuracy of any of that code based on where I am today.
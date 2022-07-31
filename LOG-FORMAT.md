# Log File Format

A Dropkick log is a CSV text file. It is composed of NMEA 183 standard GPS records
 interspersed with the extension records described below.

 A new log file is automatically created and opened when the device detects that a climbout to jump altitude has started.  Logging continues in that same file until the jumper reaches the ground.

 ## Time base

 Some NMEA records include a UTC timestamp computed by the GNSS receiver. A native clock in an Arduino application is the
 millis() system call.  It is a unsigned 32-bit value reflecting the number of milliseconds
 elapsed since boot time.  Both time values
 will appear in a log file.  These two timelines are designed to be correlated using a $PTH record, described below.

## $PVER Record

A single instance of this record appears at the start of a log.  It documents
 the software version used to create the log.

### Comma-separated Fields

 | Description   |                                        |
 |---------------|----------------------------------------|
| $PVER         | Record identifier                      |
| id string | ID of this app version   |
| version number       | an integer version number of the application        |

### Example 

`$PVER,"Dropkick, version 0.52",52`

## $PIMU Record

PIMU record logs sensor information from the [MPU6050 IMU IC](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf).  The MPU6050 is mounted on the
Dropkick PCB such that the postive X-Axis projects out the SD card slot, the positive Y-Axis projects out the face opposite the USB port, and the positive Z-Axis projects out the "top" of the enclosure.  Since the device might be carried by a skydiver in almost any orientation, it would be up to analysis software to discern the body orientation inferred by actual readings.

### Comma-separated Fields

| Description   |                                        |
|---------------|----------------------------------------|
| $PIMU         | Record identifier                      |
| millis() timestamp | Time of sample in milliseconds    |
| X-accel       | expressed in meters per second squared          |
| Y-accel       | expressed in meters per second squared          |
| Z-accel       | expressed in meters per second  squared       |
| X-rate       | raw X rotation rate; expressed in radians per second         |
| Y-rate       | raw Y rotation rate; expressed in radians per second         |
| Z-rate       | raw Z rotation rate; expressed in radians per second         |

### $PIMU Example
`$PIMU,13925127,7.01,5.31,0.49,-0.00,0.01,0.11`

## $PENV Record

This record logs pressure information captured from the [DPS310 sensor IC](https://www.infineon.com/dgdl/Infineon-DPS310-DataSheet-v01_02-EN.pdf?fileId=5546d462576f34750157750826c42242) and a resistor ladder used to monitor the VBATT battery line voltage level.

### Comma-separated Fields
| Description   |                                        |
|---------------|----------------------------------------|
| $PENV         | Record identifier                      |
| millis() timestamp | Time of sample in milliseconds    |
| static air  pressure       | expressed in hPa         |
| estimated altitude | based on static air pressure reading for a standard day; expressed in feet         |
| VBATT voltage      | battery voltage level (3.5V - 3.8V typ.); sampled once every 30 seconds      |

### $PENV Example

`$PENV,13925040,984.62,791.18,3.79`

## $PTH Record

 A PTH record is used to correlate Arduino millis() time with the GNSS computed time of day
  clock information that appears in NMEA records.  A PTH record will appear immediately following each NMEA GGA or
  GGL record.  The timestamp present in the record reflects the millis() time at the arrival
  of the first character of the NMEA record.

### $PTH Example

`$PTH,13925296`


## Reporting Rates

Sensor records are written to the file at these rates:

| Record Type      |  Reporting Rate |
|:----------------:|---------------------------------:|
|  GNSS position report            |       2 Hz |
|  PIMU            |      40 Hz |
|  PENV            |       4 Hz    |
|  PTH             | follows each GGA and VTG record|

Valid for version 52 of the Dropkick firmware.
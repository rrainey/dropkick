# Log File Format

A Dropkick log is a CSV text file. It is composed of NMEA 183 standard GPS records, or "sentences" using NMEA terminilogy,
 interspersed with the extension sentences described below.

 A new log file is automatically created and opened when the device detects that a climbout to jump altitude has started.  Logging continues in that same file 
 until a short time after the jumper reaches the ground.

 ## Time base

 Some NMEA sentences include a UTC timestamp computed by the GNSS receiver. A native clock in an Arduino application is the
 millis() system call.  It is a unsigned 32-bit value reflecting the number of milliseconds
 elapsed since boot time.  Both time values
 will appear in a log file.  These two timelines are designed to be correlated using information in a $PTH sentence, described below.

## NMEA Checksums

As of version 55/155, all sentences end with a three character NMEA checksum sequence ("*HH", where HH is the hex represntation of the checksum byte).  These  are omitted from the examples shown below for clarity.

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

### For Dropkick boards

PIMU record logs sensor information from the [MPU6050 IMU IC](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf).  The MPU6050 is mounted on the
Dropkick PCB such that the postive X-Axis projects out the SD card slot, the positive Y-Axis projects out the face opposite the USB port, and the positive Z-Axis projects out the "top" of the enclosure.  Since the device might be carried by a skydiver in almost any orientation, it would be up to analysis software to discern the body orientation inferred by actual readings.

### For Tempo boards

The ICM42688-V IMU is installed on Tempo Boards. Values are reported in Body Axes, shown below.

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

![Dropkick axes](images/imu-axes.png)
Dropkick board Body Axes

![Tempo Frames](images/tempo-v1-frames.png)
Template board Axes, including the Case (or Body) Axis definitions

## $PIM2 Record (Tempo boards only)

This is the real-time orientation of the jumper expressed as a [Quaternion](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation).

The application maintains this orientation quaternion using a 200Hz sample
stream from the IMU. A startup value of [1,0,0,0] is used,
and orientation quaternion is updated to reflect all changes in the body axes from
that original orientation.

This raw orientation quaternion must be transformed into some world frame of reference (North-East-Down, for example) in order for it to be useful in analysis of a jump.  I have several approaches to this in mind, but it remains
as future work in this project.


### Comma-separated Fields

| Description   |                                        |
|---------------|----------------------------------------|
| $PIM2         | Record identifier                      |
| millis() timestamp | Time of sample in milliseconds    |
| W       |    quaternion w component, nondimensional      |
| X      |    quaternion x component, nondimensional        |
| Y      |    quaternion y component, nondimensional     |
| Z     |   quaternion z component, nondimensional   |


### $PIM2 Example

`$PIM2,13925127,1.0000,0.0000,0.0000,0.0000`

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

## $PSFC Record

This records the estimated surface altitude of the landing area. This value is currently computed directly from the static air pressure assuming a [standard air pressure
lapse rate](https://en.wikipedia.org/wiki/Atmospheric_pressure).

This value can be subtraced from the altitude reports in $PENV sentences to obtain an estimated height above ground level (AGL)

### Comma-separated Fields
| Description   |                                        |
|---------------|----------------------------------------|
| $PSFC         | Record identifier                      |
| estimated surface altitude | Expressed in feet, MSL. Note that the device rarely rests on the ground surface while operating -- no effort is made to take into account the resting height of the device for these samples.   |


### $PSFC Example

`$PSFC,880`

# $PST Record

This records records application state changes. This atate machine is used to
identify when to start and stop loggin of each jump. 

The application defines WAIT, FLIGHT, JUMPING, and LANDED1 states.

### Comma-separated Fields

| Description   |                                        |
|---------------|----------------------------------------|
| $PSFC         | Record identifier                      |
| millis() timestamp | Time of state change in milliseconds   |
| New State | WAIT, FLIGHT, JUMPING, or LANDED1  |


### $PST Example

`$PST,1000,FLIGHT`


## Sentence Reporting Rates

Sensor records are written to the file at these rates:

| Sentence Type      |  Reporting Rate |
|:----------------:|:---------------------------------|
|  PVER            | appears as the first line of a log file |
|  PSFC            | follows the PVER sentence |
|  GNSS position report            |       2 Hz |
|  PIMU            |      40 Hz |
|  PIM2            | follows each $PIMU sentence|
|  PENV            |       4 Hz    |
|  PTH             | follows each GGA and VTG record|
|  PST             | at each internal state change in the logger |

Valid for version 55 and later
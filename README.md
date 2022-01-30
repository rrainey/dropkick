
# Dropkick
**An experimental logger for skydivers**

This is a stand-alone, battery-powered data logger designed to be
carried by a skydiver during their jump.  It includes a GPS, IMU, and altitude sensors. Data from all sensors is logged onto a Micro-SD card. The logged data can be reviewed post-jump by to-be-designed software.

![](images/dropkick-03.png)

## The Global Chip Shortage
**January 2022** - The global semiconductor shortage is very real and impacting my progress on this project. GPS and IMU chips, in particular, are hard to find. For my own use, I'm currently planning to cannibalize these chips from existing boards to get an operational prototype.  If you are trying to repproduce my work, though, you are on your own.


## Directory Structure
* **pcb** - KiCad 6 PCB project including Gerber production files
* **enclosure** - Fusion 360 enclosure; designed to be 3D-printed
* **firmware** - Arduino-based firmware for the microcontroller
* **images** - supporting images and videos

![](images/dropkick-ANIMATION.avi)

## Specifications

Dimensions: 58mm x 51mm x 20mm

Power: rechargeable 500mAh Li-Polymer battery

Storage: removable microSD card

Log file format: one log file for each jump; raw log format is an extension of NMEA0183. Details to follow.

USB micro-B connector (for recharging and programming)

Bootloader programmed via a 10-pin J-Link compatible connector; routine Arduino programming via the USB connector.

## Enclosure Notes
The enclosure is designed to be 3D printed. Tolerances between the upper enclosure, lower enclosure, and the PCB
are quite tight - printing precision will be important. Since 3D printing technologies will vary in their precision, 
you may need to scale or otherwise modify the objects to adequately match the final measurements in the original design.

For example, I printed the prototype enclosure on a MakerBot Replicator 2 (PLA) printer.  Using the default print settings
I know that an object will shrink typically by about 1% post-printing.  I set the scale to 101% to compensate for that 
in the MakerBot Desktop tooling and the end result works well.

## Credits
This PCB design is based on several Adafruit Feather and Stemma QT sensor boards.  Adafruit maintains a family of excellent
microcontroller and sensor evalation boards and companion software.  Support them. They deserve it.

The enclosure for this project was designed using the personal edition of Fusion 360. Autodesk supports the Maker community
through access to this epic design tool.  Try it out.


## Disclaimer
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

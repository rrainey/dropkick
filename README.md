
# Dropkick
**An experimental logger for skydivers**

This is a stand-alone, battery-powered data logger designed to be
carried by a skydiver during their jump.  It includes a GPS, IMU, and altitude sensors. Data from all sensors is logged onto a Micro-SD card. The logged data can be reviewed post-jump by to-be-designed software.

![](images/dropkick-03.png)

### The Global Chip Shortage
**April 2022** - The global semiconductor shortage is very real and impacting my progress on this project. GPS and IMU chips, in particular, are hard to find. For my own use, I'm currently planning to cannibalize these chips from existing boards to get an operational prototype.  If you are trying to reproduce my work, though, you are on your own.


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

## PCB Notes
My prototype circuit boards were supplied by either OSHPark or PCBgogo.

## Solder Reflow

The [u-blox Hardware Integration Manual](https://www.u-blox.com/en/docs/UBX-16018358) provides some detailed requirements for soldering the GNSS module to a PCB:

> ### Soldering paste

>Use of "no clean" soldering paste is highly recommended, as it does not require cleaning after the soldering process has taken place. The paste in the example below meets these criteria.

> * Soldering paste: OM338 SAC405 / Nr.143714 (Cookson Electronics)

> * Alloy specification: Sn 95.5/ Ag 4/ Cu 0.5 (95.5% tin/ 4% silver/ 0.5% copper)

> * Melting temperature: 217 °C

> * Stencil thickness: 120 um

> The final choice of the soldering paste depends on the approved manufacturing procedures. The paste-mask geometry for applying soldering paste should meet the recommendations. 

> ### Reflow soldering

> A convection-type soldering oven is highly recommended over the infrared-type radiation oven. Convection-heated ovens allow precise control of the temperature, and all parts will heat up evenly, regardless of material properties, thickness of components and surface color. As a reference, see the "IPC-7530 Guidelines for temperature profiling for mass soldering (reflowand wave) processes”, published in 2001.

> ### Preheat phase

>During the initial heating of component leads and balls, residual humidity will be dried out. Note that this preheat phase will not replace prior baking procedures.

> * Temperature rise rate: max. 3 °C/s. If the temperature rise is too rapid in the preheat phase it maycause excessive slumping.

> * Time: 60 - 120 s. If the preheat is insufficient, rather large solder balls tend to generate. Conversely, if performed excessively, fine balls and large balls will be generated in clusters.

> * End temperature: 150 - 200 °C. If the temperature is too low, non-melting tends to be caused inareas containing large heat capacity. 

> ### Heating and Reflow Phase

> The temperature rises above the liquidus temperature of 217 °C. Avoid a sudden rise in temperatureas the slump of the paste could become worse.

> * Limit time above 217 °C liquidus temperature: 40 - 60s

> * Peak reflow temperature: 245 °C

SAC405 solder paste is relatively expensive. I have been unable to locate SAC405 solder paste in hobbyist-friendly quantities, so I opted to use SAC305 for these prototypes. The melting qualities are quite similar, and it retains the essential "no clean" properties required by U-blox -- at the cost of [potentially having slightly less sound solder connections](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.517.4830&rep=rep1&type=pdf#:~:text=The%20North%20American%20industry%20generally,comparable%20to%20that%20of%20SAC405.).

I own a very basic infrared-type reflow oven, a Puhui T962. Even though infrared ovens are not recommended by U-blox, I decided to use it and pay careful attention to the programming of heating profile. First, I did a series to timing experiments with the standard profiles comparing them against the u-blox requirements. It turned out that my oven does not have a standard heating profile that would comply with these requirements. In particular, the closest standard profile held the oven temperature above 217 °C for roughly 90 seconds. That's far longer than the 40 to 60 seconds stated in the requirements. I had to develop a custom profile to match.

I programmed this profile into the reflow oven:
![](images/reflow-profile.PNG)

## Enclosure Notes
The enclosure is designed to be 3D printed. Tolerances between the upper enclosure, lower enclosure, and the PCB
 are quite tight - printing precision will be important. Since 3D printing technologies will vary in their precision, 
 you may need to scale or otherwise modify the objects to adequately match the final measurements in the original design.

For example, I printed the prototype enclosure on a MakerBot Replicator 2 (PLA) printer.  Using the default print settings
 I know that an object will shrink typically by about 1% post-printing.  I set the scale to 101% to compensate for that 
 in the MakerBot Desktop tooling and the end result works well.

## Credits

This PCB design is based on several Adafruit Feather and Stemma QT sensor boards.  Adafruit maintains a family of
 excellent microcontroller and sensor evalation boards and companion software.  Support them. They deserve it.

The enclosure for this project was designed using the personal edition of Fusion 360. Autodesk supports the Maker community through access to this epic design tool.  Try it out.


## Disclaimer
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

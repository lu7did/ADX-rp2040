# RDX rp2040 Digital Transceiver (RDX_rp2040)

# Overview to Version 2.0 (Alpha)

A brief story of the project start with the excelent ADX Transceiver from Barb (WB2CBA) which can be found at

* Github site [link](http://www.github.com/WB2CBA/ADX).
* ADX transceiver blog [link](https://antrak.org.tr/blog/adx-arduino-digital-transceiver)

The ADX transceiver is powered by an Arduino Nano (ADX) or Arduino Uno (ADX_UNO) boards using both the 
ATMEL ATMEGA382p processor.

In order to leverage the capabilities of the transceiver with a powerful processor such as the Raspberry Pi Pico
which uses the rp2040 architecture this project was started.

Then a map between the Arduino board I/O and the rp2040 I/O was made showing some differences needs to be addressed
which requires additional circuitry.

Once the hardware platform was defined the firmware was ported using the ADX_UnO_V1.3 firmware as a baseline, the
porting didn't introduce any new feature or function, just the minimum number of changes to the code to accomodate
the architecture differences between both platforms. The firmware version emerging from that initial effort
can be found as [ADX-rp2040](https://github.com/lu7did/ADX-rp2040/tree/master/src/ADX-rp2040) firmware version

Continuing with the roadmap of the project an experimental firmware able to operate as an autonomous transceiver by decoding and
generating FT8 signals without the usage of an external program such as WSJT-X was created, documentation of the
features on this project are documented here.

```
*New in release 2.0 *

* Initial experimental release, for evaluation purposes only
* Automatic FT8 operation.
* TFT LCD 480x320 support.
* Autocalibration mode has been added check the appropriate section on how to enable and operate.
```



# Hardware

Same hardware than the supported by the ADX-rp2040 firmware with the following additions.

*	Audio amplifier (see modifications).
*	TFT LCD IL99488 480x320 board (see wiring). 


# Firmware
## Build environment

Same build environment than the one used by the ADX-rp2040 firmware plus:


## Pre-requisites and libraries

Same build environment than the one used by the ADX-rp2040 firmware plus the following libraries:

*	[TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) Library by Bodmer
*	[TFT_eWidget](https://github.com/Bodmer/TFT_eWidget) Library by Bodmer 

Code excerpts gathered from manyfold sources to recognize here, large pieces of code were extracted from former projects

* [PixiePi](https://github.com/lu7did/PixiePi).
* [Pixino](https://github.com/lu7did/Pixino).
* [OrangeThunder](https://github.com/lu7did/OrangeThunder).

## Code structure

### Overall FT8 decoding logic

![Alt Text](../../docs/RDX-rp2040-FT8.png "FT8 decoding cycle")


### FT8 protocol finite state machine

![Alt Text](../../docs/RDX-rp2040-FSM.png "FT8 protocol finite state machine")

Experimental, yet to be documented.

## Automatic calibration (autocalibration)

Starting on version 2.0 build(23) and higher a new capability to perform an automatic calibration of the Si5351 VFO has been added.

### Enabling

The firmware allows the automatic calibration of the Si5351 dds using the following procedures.

### Operation

When started the firmware will look during the setup stage if the *DOWN* pushbutton is pressed, if so all the on-board LEDs will
be lit with the exception of the TX LED indicating a waiting pattern, the autocalibration procedure will start as soon as the push
 button is released.

If the board is powered off before the push button is released the previous calibration stored in EEPROM (flash memory) will be reset
to zero.

The calibration can be monitored either by the LED pattern exhibited or thru the USB serial port (Arduino IDE Serial Monitor), once
the calibration is completed the results will be written in EEPROM (flash memory) as in the manual calibration in order to be used
on sucessive starting cycles. While the calibration is being performed the TX LED will blink once per second, the rest of the
board LEDs will mark how large is currently the difference in the calibration mode:

```
         WSPR,JS8,FT4,FT8 lit       error > 75 Hz
         WSPR,JS8,FT4     lit       error > 50 Hz
         WSPR,JS8         lit       error > 25 Hz
         WSPR             lit       error > 10 Hz
         All LED off                error < 10 Hz  (final convergence might take few seconds more)
```

When monitoring the calibration thru the USB Serial monitor the messages will look like:
```
Autocalibration procedure started
Current cal_factor=0
Current cal_factor=0, reset
Si5351 clock setup f 1000000 MHz
n(12) cal(1000000) Hz dds(1000071) Hz err (71) Hz factor(0)
n(12) cal(1000000) Hz dds(1000074) Hz err (74) Hz factor(500)
.............[many messages here]................
n(11) cal(1000000) Hz dds(1000001) Hz err (1) Hz factor(71500)
n(10) cal(1000000) Hz dds(1000001) Hz err (1) Hz factor(71500)
n(9) cal(1000000) Hz dds(1000001) Hz err (1) Hz factor(71500)
n(8) cal(1000000) Hz dds(1000002) Hz err (2) Hz factor(71500)
n(8) cal(1000000) Hz dds(1000000) Hz err (0) Hz factor(72000)
n(7) cal(1000000) Hz dds(1000001) Hz err (1) Hz factor(72000)
n(6) cal(1000000) Hz dds(1000001) Hz err (1) Hz factor(72000)
n(5) cal(1000000) Hz dds(1000001) Hz err (1) Hz factor(72000)
n(4) cal(1000000) Hz dds(1000001) Hz err (1) Hz factor(72000)
n(3) cal(1000000) Hz dds(1000001) Hz err (1) Hz factor(72000)
n(2) cal(1000000) Hz dds(1000001) Hz err (1) Hz factor(72000)
n(1) cal(1000000) Hz dds(1000000) Hz err (0) Hz factor(72000)
Calibration procedure completed cal_factor=72000
Turn power-off the ADX board to start

```

Upon finalization a message will be sent thru the serial monitor and the TX led will stop to  blink, the board power needs to be cycled
to restart the operation.

While the autocalibration is performed the progress is also indicated at the TFT LCD GUI.

*	The meter (upper right) will show progress in Hz difference.
*	The text scroll will exhibit progress messages.
*	The footer will be showing the label "AutoCal".

```
                                     *** Warning ***

Calibration time might vary depending on the unique factory characteristics of the Si5351 chipset being used. Upon finalization the power needs to
be recycled for the board to restart.
```

## Time synchronization

To operate using FT8 the transmission and reception must be synchronized in time among all operator with a tolerance of less than 2 seconds. The rp2040 lacks a 
real time clock (RTC), it keeps track of the time quite precisely but starting from the boot moment, which in turn might happen at any arbitrary time, therefore 
rendering the board unusable for FT8 decoding and emitting pursposes.

There are several strategies to address this problem:

*	Using an external RTC board that can be synchronized with an external clock source.
*	Using a GPS receiver to synchronize the time.
*	Using the NTP protocol over the Internet to synchronize with a time server.
*	Some manual way to synchronize the time.

At this point the later strategy is the selected, if not because it's the simplest and quickest to implement. Upon startup the rp2040 board starts it's internal
clock is set arbitrarly. However, if the UP button is found pressed while performing the initial firmware setup the processing is held (all LEDs blinking
signals that situation). The button can be held pressed until the top of the minute and when released the internal clock is set to 00:00:00 and therefore
left synchronized.

To operate FT8 the actual time isn't needed, other administrative pursposes such as a log might require that, but the protocol itself needs to identify within a 1 sec
precision the seconds 0,15,30 and 45 of each minute; once synchronized the internal clock is precise enough to do that.

The synchronization is volatile and therefore needs to be performed everytime the board is powered, but it can be done with any celular phone or other precise time
source (synchronized with a time server) where the second 00 of each minute can be precisely spot.

```
Warning

Although the internal clock is synchronized few microseconds after the release of the UP button the actual synchronization is an eye-hand coordination
that could take some hundred milliseconds up to over a second; in some cases the synchronization isn't good enough, and that can be seen as a difficulty
to properly decode signals or have a reduced sensitiviy to small signals. In that case the best cure is to repeat the synchronization.
However a simple method is to use a clock which actually is digital but has an analog format, when the seconds handle crosses the "1" of the "12" mark the button
must be released, this will account for some differences in the reaction time to do that and thus enhance the synchronization process. 
```

## GUI

The disposition of the LCD is just to make the hardware development more amenable, but it should be placed on top of the transceiver in some form of "sandwich" configuration.

![Alt Text](../../docs/RDX-rp2040-GUI.jpg "RDX GUI Prototype")

The main areas of the GUI are:

*	Icons.
	Icons are meant to be used to activate or de-activate a given function such as WiFi, TCP/IP terminal, OTA firmware update, mount/extract a SD card, create an ADIF log and others.
	When the function isn't active it's shown as crossed (as they are most at this time).
* 	Meter.
	The meter is meant to display signal strenght (S-Units), power (in Watts), SWR or rx level. At this point only the S-meter is implemented to show a level proportional
	to the energy in the passband, it will be calibrated approximately to S-units.
* 	Display area.
	The display area shows several controls.
	*	Buttons.
		There are four buttons.
		*	TX
			When touched it will activate the transmission (similar to press the hardware TX button) and show as inverse, reversing it when touched again. 
			It will also inverse if the board is placed in transmission mode by the firmware or when the TX is activated by pressing the TX button.
		*	CQ/Call
			When touched will inverse and start sending CQ calls, eventually answering them and performing one full automated QSO until touched again (when the
			Manual/Auto control is in Manual). When selecting a particular CQ call from the text area it will be shown as "Call" while the QSO is attempted.
		*	Manual/Auto
			When in Manual the firmware will call CQ when pressing the CQ button or will answer a call if selected from the text display, in Auto mode it will
			call CQ periodically and attempt to answer the first CQ call heard.
		*	Band
			Shows the current band, the firmware support the 40,30,20 and 10 meters band, it will circulate amont them by pressing the button. Tha band change can 
			also be made by the standard ADX hardware procedure and changes made this way reflected in the value of the button. Also changes in the band performed
			by the cursors will be reflected.
	*	Cursors.
		The left cursor will decrease the current band and the right cursor increase it. Changes made will be reflected in the board LED and in the Band button.
	*	Frequency display.
		The frequency display will reflect the standard FT8 frequency of the selected band.
* 	Text area.
	This area will reflect several QSO lines using a color scheme to identify the type of it.
	*	Black on White. 3rd party QSO.
	*	Black on Yellow, CQ from our station or answering to another station.
	*	White on Red, QSO in progress.
	*	Black on Green, CQ from another station.
	When a CQ call from other station is selected by the pencil the transceiver is placed in "Call" mode and an attempt to perform a QSO is made.
* 	Waterfall.
	This area will show a waterfall representation of the passband updated every second.
* 	Footer.
	This area will show configuration information such as firmware level, callsign, grid locator, time and IP address.


```
Warning

The band settings on the firmware needs to be made consistent by using the profer filter on the board and antenna as there is no way for the firmware
to validate neither the proper filter nor a reasonable SWR level when the TX is activated.
```


# Hardware

The hardware required by this transceiver derives directly from the ADX Transceiver (WB2CBA), the implementation can take basically two forms:

* Build a hand wired version of the circuit.
* Build an ADX transceiver and replace the Arduino Nano with the ADX2PDX daughter board created by Barb (WB2CBA), see below.

## ADX_rp2040 circuit


The circuit used is esentially the ADX transceiver with the minimum set of modifications to accomodate a Raspberry pico (rp2040 processor) instead of an
 Arduino Nano (ATMEGA328p processor).
The following diagram has been originally conceived by Dhiru (VU3CER) and put together by Barb (WB2CBA):

![Alt Text](../../docs/PDX_V1.0_Schematic.jpg "PDX Schematic")
PDX_V1.0_Schematic.jpg

Check additionally mods required and TFT support requirements detailed below.

The receiver, Si5351 clock, RF driver and final stages are identical to the standard ADX Transceiver, whilst changes are made around the rp2040 processor to
accomodate the different signaling and voltages used.


### rp2040 pinout assignment

Same as the ADX-rp2040 project


### Power supply

Same as the ADX-rp2040 project

### Receiver

The receiver sub-system is identical than the ADX Transceiver.

### SWR protection

A Zener Diode (D10,1N4756) located where the board TP3 is defined would prevent a situation of high SWR to damage the finals.

### RF Power 

The RF power (driver and finals) is identical than the ADX Transceiver.

### Low Pass Filter

The Low Pass Filter (actually more than that) is needed to suppress unwanted spurious responses and also to achieve high efficiency class E operation.
The design is identical than the ADX Transceiver.


### ADX2PDX daughter board

Same as the ADX-rp2040 project


## RDX-rp2040 modifications

The following modifications applies to both the schematic of the RDX transceiver or the ADX2PDX daughterboard.
```

 * Build a small class A audio amplifier.
 * Connect the input of the amplifier to RXA.
 * Connect the output of the amplifier to GPIO26 (ADC0) pin 31 of the rp2040 board.

```
A suitable circuit can be seen in the following schematic

![Alt Text](../../docs/RDX-rp2040-AF.png "Audio amplifier")


## TFT LCD display support

The firmware supports a TFT LCD IL9488 480x320 display where a GUI is presented allowing the operation of the transceiver, the LCD is optional
but it greatly enhances the autonomous FT8 operation allowing to see the activity on the channel and operate either to call CQ or answer to
an on-going call.

The actual wiring of the TFT board needs to connect the pinout to the Raspberry Pico (rp2040) processor as indicated in the following
diagram:

![Alt Text](../../docs/RDX-rp2040-TFT.png "TFT LCD wiring")


## ADX2PDX daughter board prototype fixes

Same as the ADX-rp2040 project

## ADX2PDX PCB

Same as the ADX-rp2040 project


# Testing

Only preliminar testing has been performed as it is just an alpha version of the firmware for preliminary evaluation purposes,
functions will be tested as the implementation evolves.

# Pending

* Organize and add functionality for icons
* Hardware interface to SD-Card/Export
* File system (Flash based)
* File system (SD card based)
* ADIF generation

## low priority roadmap
* Develop or adopt a PCB layout design.
* CAT support (TS840).
* WSPR beacon.
* Support for QUAD multifilter board
* CW operation.
* GPS support & time alignment
* Support for Si4732 chipset
* Support for smaller display 
* Support for ATU reset
* SWR indicator & control (as HW support is introduced)
* Filter support (as HW support is introduced)
* Support Si4732 based receiver (as HW support is introduced)

## rp2040-w specific
* WiFi support
* mDNS implementation (rdx.local resolution)
* NTP support and clock alignment
* Configuration terminal
* OTA firmware update
* Web based configuration tool
* Web based ADIF export tool
* USB based file system

# Done
* Port automatic calibration from ADX-rp2040
* progress bar for RX/TX (green/red)
* display dialog multiband
* integrate meter (S-Meter and Power)
* document ft8 FSM (UML)
* include multiband support
* integrate scroll text 
* improve ft8 FSM (organize)
* Manual/Auto control
* CQ control
* TX control

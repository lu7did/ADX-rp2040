# RDX rp2040 Digital Transceiver (RDX_rp2040)

# Overview to Version 2.0

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
the architecture differences between both platforms.

This is a special experimental firmware able to operate as an autonomous transceiver by decoding and
generating FT8 signals without the usage of an external program such as WSJT-X

```
*New in release 2.0 *

* Initial experimental release, for evaluation purposes only
```



# Hardware

Same hardware than the supported by the ADX-rp2040 firmware. 


# Firmware
## Build environment

Same build environment than the one used by the ADX-rp2040 firmware.

## Pre-requisites and libraries

Same build environment than the one used by the ADX-rp2040 firmware.

Code excerpts gathered from manyfold sources to recognize here, large pieces of code were extracted from former projects

* [PixiePi](https://github.com/lu7did/PixiePi).
* [Pixino](https://github.com/lu7did/Pixino).
* [OrangeThunder](https://github.com/lu7did/OrangeThunder).

## Code structure

Experimental, yet to be documented.


# Hardware

The hardware required by this transceiver derives directly from the ADX Transceiver (WB2CBA), the implementation can take basically two forms:

* Build a hand wired version of the circuit.
* Build an ADX transceiver and replace the Arduino Nano with the ADX2PDX daughter board created by Barb (WB2CBA), see below.

## ADX_rp2040 circuit

The circuit used is esentially the ADX transceiver with the minimum set of modifications to accomodate a Raspberry pico (rp2040 processor) instead of an
 Arduino Nano (ATMEGA328p processor).
The following diagram has been originally conceived by Dhiru (VU3CER) and put together by Barb (WB2CBA):

![Alt Text](docs/PDX_V1.0_Schematic.jpg "PDX Schematic")
PDX_V1.0_Schematic.jpg

The receiver, Si5351 clock, RF driver and final stages are identical to the standard ADX Transceiver, whilst changes are made around the rp2040 processor to
accomodate the different signaling and voltages used.


### rp2040 pinout assignment
For circuit design and future expansion several assignmentes has been made on the rp2040 pinout for the following assignment assignment


![Alt Text](docs/rp2040_pinout.jpg "rp2040 pinout")


### Power supply

The Raspberry Pi Pico operates with +3.3V logic as opposed to the Arduino Nano used by the ADX transceiver, still it has an internal +5V/+3.3V regulator which
is used by the design. 
The standard +12Vcc voltage is used to feed only the final amplifier (3xBS170), a +12Vcc/+5Vcc regulator is used to obtain the voltage to feed the Raspberry
Pico (VSYS) pin, the +3.3Vcc obtained from the board is then used to feed the receiver logic, the Si5351 clock, the 74ACT244 RF driver and the CD2003GP based 
receiver as well as the miscellaneous circuitry such as LED indicators, switches and signal comparator.

### Switches
Like the standard ADX transceiver the design carries three (3) push (normally open) switches to signal:

* **UP** change mode, up band in band setting mode and up frequency in CW mode. Also used to signal the start of the calibration mode on start-up (see below).
* **DOWN** change mode, down band in band setting mode and down frequency in CW mode. Also used to signal start of serial configuration terminal on start-up (see below).
* **TX** manual transmit, manually set the transceiver in transmission mode, also keyer in CW mode.

The UP/DOWN buttons can be used, if pressed simultaneously to place the transceiver in **Band change** mode, when pressed at the start they can be used
to place the transceiver in calibration mode, this behavior is identical to the ADX transceiver with it's origiinal firmware.
```
¡WARNING!
The three resistors pulling up the switch voltage from +5Vcc in the ADX transceiver **needs to be omitted (not populated)**
on the PDX transceiver as they will feed the corresponding GPIO pins with +5Vcc instead of +3.3Vcc and might result in the
**damage** of the processor. The firmware uses internal pull up resistors to replace them.
```

### LED

Like the standard ADX transceiver the design carries four (4) LED to signal the transceiver state:

* **WSPR LED**, to signal WSPR mode, band1 in band setting operation, calibration mode and terminal mode. Also frequency indicator in CW mode.
* **JS8 LED**, to signal JS8 mode, band2 in band setting operation, end of calibration mode and terminal mode. Also frequency indicator in CW mode.
* **FT4 LED**, to signal FT4 mode, band3 in band setting operation, end of calibration mode and terminal mode. Also frequency indicator in CW mode.
* **FT8 LED**, to signal FT8 mode, band4 in band setting operation, end of calibration mode and terminal mode. Also frequency indicator in CW mode.
* **TX LED**, to signal transmission, band setting operation, end of calibration mode and terminal mode, watchdog activated, also keying in CW mode.
```
¡WARNING!
Due to the smaller drawing capability and lower output voltage of the GPIO pins 3mm red LED are
recommended for all five positions
```

### Si5351

Operation of the Si5351 clock generator is identical as in the ADX transceiver with minor differences, it's being used as:

* CLK0, transmitter oscilator and FSK generator.
* CLK1, receiver oscillator.
* CLK2, calibration reference signal.

```
¡WARNING!
The calibration process is manual as per the ADX transceiver (see the original method as described on Barb's web page and blog), the hardware
support a future implementation of an automatic calibration procedure which isn't been implemented in the firmware.
```

### Receiver

The receiver sub-system is identical than the ADX Transceiver.

### RF Power 

The RF power (driver and finals) is identical than the ADX Transceiver.

### Low Pass Filter

The Low Pass Filter (actually more than that) is needed to suppress unwanted spurious responses and also to achieve high efficiency class E operation.
The design is identical than the ADX Transceiver.


### ADX2PDX daughter board

Barb (WB2CBA) created a small daughterboard, dubbed as ADX2PDX, which can be used to transform a standard ADX transceiver into a PDX transceiver with 
minimal modifications.

The board replaces the Arduino Nano on a standard ADX board, the signals in that socket are rerouted according with the rp2040 pin
assignment. Additional support circuits are added as well:

* +5Vcc regulator, the ADX depends on the +5Vcc regulator present on the Arduino Nano.
* Comparators (MOSFET and LM393) needed to process the signal for the rp2040 firmware counting methods to process.
* Miscellanea hardware needed for the rp2040 to operate.

A picture of the prototype daughter board is shown

![ADX2PDX](docs/PDX_ADX2PDX.jpeg "ADX2PDX daughter board") 


The daughterboard schematic is as follows:

![Alt Text](docs/PDX2ADX_DaughterBoard_V1.1_Schematic.jpg "ADX2PDX Schematic")

```
¡WARNING!
**ADX2PDX daughter board is at the prototype stage**
```

```
¡WARNING!
In order to accomodate the daughterboard on top of the standard ADX board some construction modifications are recommended:
* Bend L2,C16,C17 and C19 as they are large components with might interfere with the daughterboard.
* Use a 2x pin strip (female) instead of a socket for the Arduino nano as it creates better grip of the daughter board.
* Remove R12,R13 and R14 to avoid +5Vcc to reach the GPIO pins of the rp2040 rated for operation at +3.3V and prevent potential damage. 
```


This approach allows an existing ADX board to be upgraded with the new processor but to develop either a wired prototype
 or a custom board using the rp2040 processor instead of the ATMEGA328p are also options.


```
Construction note
For building flexibility both the MOSFET and CI based comparators are provided in the daughter board but only one
must be connected, even if both can physically be present the selections of the JP1 and JP2 jumpers will define
which one is actually used.
Some modifications on the ADX2PDX daughter board and the ADX board needs to be applied depending on the 
comparator used, please see text below for further details.
```

## RDX-rp2040 modifications

The following modifications applies to both the schematic of the RDX transceiver or the ADX2PDX daughterboard.
```

 * Build a small class A.
 * Connect the input of the amplifier to RXA.
 * Connect the output of the amplifier to GPIO26 (ADC0) pin 31 of the rp2040 board.

```
A suitable circuit can be seen in the following schematic

![Alt Text](docs/ADX-rp2040-AF.png "Audio amplifier")


## ADX2PDX daughter board prototype fixes

Same as the ADX-rp2040 project

## ADX2PDX PCB

Same as the ADX-rp2040 project


# Testing

Only preliminar testing has been performed as it is just an evaluation version of the firmware, functions will be
tested as the implementation evolves.

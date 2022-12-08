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

Same as the ADX-rp2040 project


### rp2040 pinout assignment

Same as the ADX-rp2040 project


### Power supply


Same as the ADX-rp2040 project

### Receiver

The receiver sub-system is identical than the ADX Transceiver.

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

 * Build a small class A.
 * Connect the input of the amplifier to RXA.
 * Connect the output of the amplifier to GPIO26 (ADC0) pin 31 of the rp2040 board.

```
A suitable circuit can be seen in the following schematic

![Alt Text](../../docs/ADX-rp2040-AF.png "Audio amplifier")


## ADX2PDX daughter board prototype fixes

Same as the ADX-rp2040 project

## ADX2PDX PCB

Same as the ADX-rp2040 project


# Testing

Only preliminar testing has been performed as it is just an evaluation version of the firmware, functions will be
tested as the implementation evolves.

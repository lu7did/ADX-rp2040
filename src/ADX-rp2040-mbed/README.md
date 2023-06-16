# ADX rp2040 Digital Transceiver with USB support
#(ADX_rp2040-mbed)

## Overview to Version 1.0

This project derives from the ADX-rp2040 with some mods to enable it to be ran using an USB connection
instead of audio cables.

Please refer to the main documentation of the ADX-rp2040 project, everything there is valid for this 
version unless noted otherwise in this documentation.

## Acknowledgement

This version could not be possible without the generous insights, preliminary work and code shared by
Sitoshi-san (JE1RAV). Excerpts from his QP-7C-CAT firmware has been extensively used in this firmware.

## Release history

```
*New in release 1.0 build(01) and higher*

* Initial evaluation release.
```


# Support and issues

This is an experimental, work-in-progress, non-profit, project performed as closest to the ham spirit as possible. Only spare, hobby,
time is available to move the project forward or to provide support on usage or issues.

If anybody has questions or issues please:
 

* Be sure you read the documentation first.
* Check on the issues list of the GitHub site, the issue might have been described there or even a workaround might exists for it.
* State it as an  English request. English isn’t even my fourth language, still I do my best to adhere to it. 
* For casual question you can use the groups.io uSDX forum and for a longer ones please open and issue at the GitHub portal of the project. 
* Express very clearly which version and level the firmware has. In most cases using the latest would solve the issue.
* Ensure the issue happens with a freshly downloaded last version of the firmware, don’t expect me to debug any modification you did.
* Describe in your own words the problem and what you did to expose it and what workarounds you attempted.
* Present a photo of the issue if it can be seen in the TFT display.
* Add the content of the monitoring terminal session with DEBUG enabled to help me understand what is going on.
* Attach any other documentation you think might help debugging the issue.
 

Please report ONE (1) issue per entry, and proceed as clean as possible with the debug instructions given to you to further understand or to fix the problem. 
I’ll address your issue as soon as my available time allows, not necessarily in a FIFO way.


# Hardware

This firmware version works with the following boards:

*	ADX-rp2040 (CD2003GP receiver), porting from the ADX-UNO board but using a rp2040 processor, no PCB available.
*	ADX-rp2040 daughterboard, an additional board to be plugged in the Arduino Nano socket of a standard ADX-UNO board, PCB available.
*	RDX-rp2040 (CD2003GP and Si4732 receiver versions), stand alone FT8 pocket transceiver, still can be flashed with this firmware.


# Firmware
## Build environment

The development environment used is the Arduino IDE, even with some limitations it's far more easy to setup and operate
than the Eclipse IDE alternative and present a much smoother transition from development for the Arduino environment into
the rp2040 environment.

The usage of the Arduino IDE is based on the [arduino pico mbed core libraries developed by Martino Facchin](https://github.com/arduino/ArduinoCore-mbed).

In order to install it a tutorial can be found at the GitHub repository under the section *Installation*

This is the standard Raspberry Pi Pico Arduino IDE support.

## Pre-requisites and libraries

In order to build the firmware some libraries are needed, the dependencies are shown as follows

Basic support

Code excerpts gathered from manyfold sources to recognize here, large pieces of code were extracted from former projects

* [QP-7C_CAT] (https://github.com/je1rav/QP-7C_RP2040_CAT/tree/main/qp7c_rp2040_cat).
* [PixiePi](https://github.com/lu7did/PixiePi).
* [Pixino](https://github.com/lu7did/Pixino).
* [OrangeThunder](https://github.com/lu7did/OrangeThunder).

## Code structure

 
* **Tone frequency counting**.
Uses the zero crossing algorithm used by Hans Summers (G0UPL) in his fantastic QDX transceiver as described in the user manual.

## CAT Support

When enabled by removing comments from the **"#define CAT     1"** statement the USB Serial port operate a CAT (Computer Aided Tuning).

The parameters of the support would be:

*	Protocol: Kenwood TS-2000.
*	Speed: 115200
*	Parity: 8N2
*	Control lines: DTR/RTS lines On. Handshake: none.

When a frequency change is made the FT8 & WSPR leds will blink for 10 seconds to remaind the operator that
the output filters might require a review prior to the operation.

## Automatic calibration (autocalibration)

No automatic calibration (autocalibration) function implemented yet.


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
* CLK1, receiver oscillator, if SUPERHERODYNE is defined then the CLK1 frequency is substracted from the BFO frequency.
* CLK2, N/A.

```
¡WARNING!
The calibration process is manual as per the ADX transceiver (see the original method as described on Barb's web page and blog), the hardware
support a future implementation of an automatic calibration procedure which isn't been implemented in the firmware.
```


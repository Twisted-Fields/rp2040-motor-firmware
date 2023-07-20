# Twisted Fields RP2040 motor controller firmware

This is the firmware for our custom dual brushless motor controller.

Hardware design on [Github: rp2040-motor-controller](https://github.com/Twisted-Fields/rp2040-motor-controller)

Our system is experimental and still undergoing testing.
Initial testing shows good results, but we cannot gurantee 
safe or reliable operation at this time.

# Firmware features

- SimpleFOC - FOC control for 2 motors
- Serial Commander interface for both motors
- Support for MT6701 encoder in SSI mode
- Support for MT6701 encoder in ABZ mode
- Status LED driver (3x Neopixel)

This board features two CPUs with the second CPU focused on
CAN bus communications and additional I/O management.


# Compiling the firmware

:warning: Current version depends on unreleased changes to the SimpleFOC library. To compile this project before the release of SimpleFOC 2.2.3, please git clone the [SimpleFOC dev branch](https://github.com/simplefoc/Arduino-FOC/tree/dev) into the lib/ directory of this project.

This project is set up to use PlatformIO. You can try in other environments, your mileage may vary. In particular, the board definition for the Twisted Fields RP2040 motor controller for PlatformIO is included in this project, and its equivalent for other environments would have to be configured first.
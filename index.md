![CI Build](https://github.com/vintagepc/MINI404/workflows/CI%20Build/badge.svg) ![Code Lint](https://github.com/vintagepc/MINI404/workflows/Code%20Lint/badge.svg)
# Mini404 - Like MK404, but with an appendix you can un-break!

![](https://raw.githubusercontent.com/wiki/vintagepc/MINI404/images/Line.png)

## Introduction

Mini404 is the natural successor to [MK404](https://vintagepc.github.io/MK404/) for Prusa's STM32-based Mini printer! It's still relatively young in its development and the ARM architecture and peripherals are much more complex to simulate. It is a spare time project but I am still actively developing features and capabilities where time permits. It's built on QEMU this time as that was the best ARM emulation platform I could find - or, at least, one with an ARM MCU and a bare-bones STM32F implementation. Be sure to check out the MK404 page for some additional background and motivation on how these projects came about. 

We feature many overlapping features with MK404, including the scripting engine, advanced arm-gdb debugging capabilities, and 3D printer visuals!

![](images.githubusercontent.com/53943260/99891866-7b1ba000-2c3c-11eb-804b-427196de95df.png)![image](https://user-images.githubusercontent.com/53943260/99891868-8242ae00-2c3c-11eb-91fd-7bab7657e3ee.png)![image](https://user-images.githubusercontent.com/53943260/101267602-70770580-3728-11eb-97f5-f6258eec8e11.png)![image](https://user-images.githubusercontent.com/53943260/101993374-12967080-3c88-11eb-915a-82a25005cbed.png)![image](https://user-images.githubusercontent.com/53943260/104094225-76975f00-525d-11eb-8bba-1d2388fc085d.png)

In addition, the use of QEMU gives us even more features to leverage:

- Simulated Ethernet to the host

- Simulated or physical USB drive on the host computer

- Emulation save states so you can snapshot the machine state and resume it later

- Many more capabilities we don't explicitly use but are provided by QEMU, such as virtual consoles, host serial devices, pipes, and file-backed storage for EEPROMs and data. 

![](https://raw.githubusercontent.com/wiki/vintagepc/MINI404/images/3D_model.png)

## Prusa Mini Features 

This is an overview of the current status of the Mini implementation. 

Name|Description|Status 
----|-----------|------
Bootloader/.bbf| Used to load stock images | ✔️ Works
CDC (USB) | Serial USB control | ❌ ⚠ Not working/in progress (#29). Use a workaround to re-route serial to UART 1 and then use the serial VC. 
EEPROM/flash | persistent storage|  ✔️ Works (`-pflash`, `-mtdblock`, see [Flash Storage Persistence](https://github.com/vintagepc/MINI404/wiki/Flash-Storage-Persistence))
Ethernet | LAN connectivity | ✔️ Appears to work, see [Ethernet](https://github.com/vintagepc/MINI404/wiki/Ethernet)
Fan| Simulated fan component |✔️ Works - with tachometer feedback
Heater| Simulated heater cartridge | ✔️ Works, feeds back temperature to thermistors.
IRSensor| IR-based MKxS sensor | ✔️ Works.
MINDA| Z-endstop sensor |  ⚠ Not implemented - currently using Z min "stall-guard" as MINDA input.
Print[er] Visualization |Viewing motion/print| ✔️ Works - see [Advanced Visuals](https://github.com/vintagepc/MINI404/wiki/Advanced-Visuals)
RotaryEncoder| Input knob | ✔️ Works
st25dv64k| I2C EEPROM chip | ✔️ Works 
st7789v | SPI Display driver | ✔️ Works 
Thermistor| Assorted temperature inputs | ✔️ Works 
TMC2209| Stepper driver | ✔️ Works ⚠ Not all registers/features implemented, many are transparent to the simulator.
w25q64jv| SPI flash chip | ✔️ Works 
USB | Flash drive for gcode input | ✔️ Works 

## Non-hardware-specific feature highlights

  - ARM-gdb debugging - the ARM version of GDB is new enough that it can be used together with the `Native debugging` extension in VS Code for a very nice debugging experience. 
  
  - Mini404 can run the factory .bbf files if the Mini boot-loader is installed as `bootloader.bin` in the same location as the `qemu-system-buddy` binary. 

  - Modular approach - Much like MK404, (and partly due to QEMU's architecture) the design is modular so it will be easy to re-use components to support additional printer models.

# Supported platforms

At current we only actively support Linux. While it should be possible to compile QEMU and probably the Mini404 components for Windows (MinGW64) or OSX, I do not currently have the time nor focus to handle this. Hopefully a native MinGW build will be available in the near future, however as I do not have OSX hardware or a development environment it is not something I can support nor maintain. 

See [Getting Started](https://github.com/vintagepc/MINI404/wiki/Getting-Started) for a primer on what you need to compile and how to get started. 

# A Note on the STM32 architecture

- The STM32F4xx architecture in Mini404's fork of QEMU has received significant expansion over the development of this project with my own peripheral implementations. So far it's the most complete one I'm aware of, and I'd like to thank both the QEMU developers/contributors for their work in the STM32 components that come "stock" with QEMU, as well as the [pebble-qemu](https://github.com/pebble/qemu) fork for their prior work Mini404 was able to leverage or adapt to make this project possible! Long term as these implementations stabilize, I hope these improvements will make it upstream, but currently I do not have the time nor plans to do so myself. 


# More Reading

Head on over to the [Main Repo](https://github.com/vintagepc/MINI404/tree/MINI404) to get started, or...

... browse the [Wiki](https://github.com/vintagepc/MINI404/wiki) for some more in-depth reading on features and how to use them. 

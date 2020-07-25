
# Laser4+

## Introduction

Laser4+ is an hardware upgrade of Hitec Laser4 Radio Controller transmitter. This radio came with features like 4 channels, V-tail and elevon mixing and trainer system, back in the day these were standard features and the only inconvenience is that transmits a FM modulated signal on the 35MHz band and caused all sort of issues, like having to constantly change the crystal oscillator or picking interferences.  

Now days the fpv hobby is very popular and accessible and that I caught attention, knowing that I had a possible supported remote control laying around I decided to give a try.  
First I did some research and found [UAVfutures](https://www.youtube.com/watch?v=Avp8MurmeEY) 99$ drone build, after that I started to buy all the parts. As soon I got all the parts I started the build and in no time all was hooked up and was time to connect the receiver and that was when I found the problem.  

The problem was basically that the flight controller that I bought does not support the old PWM protocol that my receiver uses but it supported PPM signal, so I made some research with the intention of making some kind of converter and found that my receiver already had this signal internally but made the conversion to separated PWM channels.
Next step was to connect the PPM signal from the receiver to the flight controller and I thought that would work but no. With the PPM signal from the receiver connected to the fight controller and using the Betaflight Configurator I could see that the signal from the remote was being received but the drone didn't perform any movement on the motors.

After a while I figure out that the drone required an arming channel, as they are potentially dangerous it make sense that a dedicated arming channel should exist. Now the problem was that my remote only had four channels, and at this point I though on just buy another remote, but the idea of already having a remote and buy a expensive one that was future proof didn't make sense to me.
Once more I made a more in depth research and found the following:  

- My remote transmitted four channels in PPM.  
- The training jack in the back had PPM in, PPM out and TX disable signals available.  
- My receiver supported a six channel transmitter.  
- Found [PWM to PPM converter](https://www.rcgroups.com/forums/showthread.php?1000132-DIY-Servo-Signal-%28PWM%29-to-PPM-Converter) using a microcontroller.  

With this information I concluded that was possible to add extra channels to a PPM signal and wrote a program for a Tiny13 that added two extra channels controlled by two switches. With this solution I was able to arm the drone and perform some successful flights.

Meanwhile I found the [Multiprotocol](https://github.com/pascallanger/DIY-Multiprotocol-TX-Module) project and concluded that it was possible to adapt it to the Laser4 remote, after that I decided to design my own transmitter module based on the STM32F103 microcontroller and CC2500 transmitter module with my own set of features, and with that the Laser4+ project started.

### Final result

![laser4+](/doc/laser4+.jpg)

### Switches:

1. Momentary switch AUX1 channel
1. Toggle switch AUX2 channel
1. Momentary switch AUX3 and rotary encoder AUX4

### Features

- FrSkyD protocol with CC2500+PA Radio module
- 6 PPM channels with 35MHz Radio
- 3 switch channels
- 1 rotary encoder channel
- OLED display
- USB joystick
- Internal buzzer
- Single 3.7V Li-ion battery
- Battery charge from usb
- Download firmware update support (DFU)

## Development

The project goal was to design a custom PCB (Printed Circuit Board) that interfaced with original radio PCB and added the required features. First I started to see what was power requirements for the original PCB, and as it used a pack of eight Ni-MH cell, the minimum supply voltage is 7.2V and the maximum 9.6V, so the new PCB had to have a step-up converter because it is supplied by a single Li-ion cell which has a maximum of 4.2V, the MT3608 part is a very common part for this task.
As charging is also a requirement I used the Microchip MCP73831 because is inexpensive and had a bunch of them laying around, but for future I will avoid this part for large battery capacity since the sot23-5 package cannot support charge currents above 300mA. Another feature to the charge circuit is the load-sharing circuit, this circuit allows powering the remote from USB and charge the battery simultaneously.
For the 3.3V regulation the RT9193-33 was also a cheep choice, not requiring much external components and easy to source from ebay. One particular feature that I wanted was the current consumption along time, and for that a current sense circuit was added.

Controlling everything is a STM32F103 microcontroller, it is very a capable device, is supported by the Multiprotocol and already used this part on multiple projects. Finally for the radio module, there are multiple options available like nrf24l01, cc2500, a7105 and many others, but the choice ended to be the cc2500 because it was the only module compatible with the cheapest receiver with sbus that I found.
After selecting the CC2500 as the radio chip, a module with it had to be selected and once again the cheaper version was used but for this I found that existed two versions, with power amplifier (WLC-24PA) and without it (WLC-24D). I ended up buying the two versions for testing and made the PCB compatible with both versions. The remaining circuitry is just simple switches connections, the full schematic of revision A can be found bellow. After assemble I notice an error that made the 35MHz FM transmitter always inactive, as workaround I removed R22 that make it always active by default then removed the external crystal oscillator. To enable back the 35MHz FM transmitter just plug back the external crystal on the socket.

### PCB Views

Top view
![l4pcb_t](/doc/l4pcb_top.jpg) 

Bottom view
![l4pcb_b](/doc/l4pcb_bottom.jpg)

PCB Installed (external antenna not installed yet)
![l4pcb](/doc/l4pcb.jpg)

### Schematic

[Laser4+ revA](/doc/Laser4+_revA_Schematic.PDF)

### Block diagram

![Block diagram](/doc/l4_bd.png)

### OLED Display

The display has a simple layout that shows basic battery information and operating mode.

![OLED Display](/doc/oled.jpg)

1. Mode indicator (35M, 2.4G, USB)
1. Instant current consumption
1. Current consumption accumulation
1. Battery voltage

## Software build requirements

- STM32Cube used for firmware installation
- FW_F1_V1.8.0  (Firmware for STM32F1 microcontroller family)
- arm-none-eabi-gcc v8.3.1

### Software Build

- `make`                default build.
- `make program`        default build and programming using OpenOCD with jlink compatible probe
- `make bootloader`     build dfu bootloader
- `make dfu`            build for dfu upload
- `make upload`         upload binary using dfu bootloader

### Operating mode selection

The remote can operate in three modes Multiprotocol (35MHz or 2.4GHz radio), USB game controller and DFU. These modes can selected with switch combination on radio power or through the configuration console.
By default the 2.4GHz radio is selected at power on, pressing and holding AUX3 before power on selects the 35MHz radio. AUX1 is used for the 2.4GHz binding process which requires restarting the remote at the end. Connecting the USB will change to game controller mode which also adds a virtual serial port for configuration. Finally pressing AUX1 in combination with AUX3 will enter in DFU mode that can only exit by restarting the remote.

### Configuration

As previously mentioned in usb mode, a virtual serial port is available and can be used to get the remote status and perform the following configurations:

- Set current measurement resistor value for more accurate current value.
- Set reference voltage used by ADC for more accurate battery voltage value.
- Force other operating modes

The available commands can be listed using the command `help` on a serial terminal.

## Future work

The features that were defined for the project are all included to the current release, but as always new features came up quickly and may be implemented later.

Features fo be added:

- Configurable channel limits and center.

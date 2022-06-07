# RC Sounds
A simple, low cost sound effect module for R/C vehicles, land or water based. It utilises a DF Robot DFR0299 MP3 module to continuously play a selected MP3 sound file. A Microchip ATTiny85 provides MP3 control functions, error handling and can support two servo PWM inputs. A set of 4 switches are used to provide easy selection of the desired sound sample. Using one of the PWM inputs, a horn/siren sound can be overlaid and once it finishes playing, normal sound resume. The horn/siren setting is programmable to allow for different receiver outputs.

A PAM8403 Class D amplifier, with EMC fixes is also provided, this allows the system to drive 2x 8 ohm speakers at 2W. Volume control is provided on-board as well as an option to mute the amplifier and finally use an external amplifier, if desired using the pre-amp outputs.

PCB designed to fit into a Hammond 1594B enclosure.
![Alt text](https://github.com/istedman/RC_Sounds/blob/main/Photos/RC_Sounds_boxed_2_web.JPG?raw=true "Boxed unit")

Total cost of all parts, PCB, enclosure and speakers is approximately £30. See the BOM directory for an HTML export with full ordering part numbers.

## Current state as of 5th June 2022.
Design is working well. Has based compatibility tests with multiple AM/2.4 GHz receivers, 2-6 channels and has been field tested.

Use the updated V1.2_test software, this adds 4 speeds to the engine sounds and utilises the auxilary input to act as the throttle. The software is written in the Arduino IDE, it uses the ATtinyCore library and no bootloader. The board must be programmed using a progammer, I use a USBASP. To allow the PWM settings function to work, you have to disable the reset function on the fuses. A high voltage programmer is required to unlock. I use this design, https://github.com/wokwi/attiny-hvsp-programmer and clip onto the ATTiny85. All fuse setting defined in the software.

ATTiny85 fuse settings are in the header of the main source code, use USBASP/Avrdudess to set them.

## Boat sound effects SD card image now available

My example SD card image, for boats has now been uploaded to the repository.

## Powering the design

Power is from a 3.7-12V supply or the receiver BEC circuit. Unit has a polyfuse to protect against reverse polarity and a linear regulator.
Do not use with a 1S LiPO ESC or any other ESC that boosts the BEC output. The current design assumes the stndard +5V BEC output voltage. My 1S LiPO ESCs output
6 or 7.2V to boost the servo power, using such an ESC (Trackstar Gen II 120A or Hobbywing Xerun) would irreprably damage the design as it will operate with 5.5V maximum.

You will need an UltraBEC/SuperBEC ESC, i.e. one that can output 2A @ 5V for reliable operation. This unit can demand a peak 0.6A @ 5V, when tested with a 1A BEC, and 3-4 servos, I had a few brownouts. If it reandomly resets at the same time as your receiver, use the external power option and remove the BEC jumper!

## Schematic and circuit description

![Alt text](https://github.com/istedman/RC_Sounds/blob/main/RC_Sounds_schematic.png?raw=true "Schematic")

Power comes from the 5V BEC output of the ESC, if you fit a jumper on J7 of from a 3.7-12V source via J2, using appropriate wire. Connections via J2 have a 1.1A polyfuse and a LD29080-5 5V, 800mA LDO regulator, which provde revers polarity protection.

The ATTiny 85 microcontroller provides the control function. It reads the PWM inputs from two receiver channels, the horn/hooter input connects to pin 5 and the throttle input connects to pin 6. 

Communications to/from the DFR0299 MP3 module are via pins 2 & 3 and use a standard 9600 8N1 serial interface. A USB to TTL adaptor can be connected to the TX output of the ATTiny 85 on pin 2 for debug messages, using J3.

A solitary LED, D1 is used to setup functions and when the horn has been activated.

A set of DIP switches, SW1, set various analogue voltages that determine the sound sample to be played. Limits are defined in the software. A setup switch, S1 is used in the setup routine to set the horn on/off position and the throttle positions.

The DAC output of the DFR0299 is connected, via a dual gang audio potentiometer R17A/B to a PAM8403 audio amplifier. J6 provides a pre-amplifier output.
The DFR0299 DAC outputs, post volume control are AC coupled via C2 and C3. Connector J1 can be used to mute the amplifier. Capacitor C10, a 47uF capacitor, is to reduce the effects of the amplifier on the power supplies. I have added the recommended filter circuit to the output stage, using FB1-FB4 and C12-C15, this reduces radio emissions in the region of 25-40 MHz, important for RC usage as there are still some 27 and 40 MHz systems around.

Into 8 ohm speakers the design can deliver 1.8W. Peak power i've observed has been close to 2W.

A photo of the PCB
![Alt text](https://github.com/istedman/RC_Sounds/blob/main/Photos/RC_Sounds_pic2_web.JPG?raw=true "Just the PCB")

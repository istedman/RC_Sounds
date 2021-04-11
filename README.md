# RC_Sounds
A simple, low cost sound effect module for R/C vehicles, land or water based. It utilises a DF Robot DFR0299 MP3 module to continuously play a selected MP3 sound file. A Microchip ATTiny85 provides MP3 control functions, error handling and can support two servo PWM inputs. A set of 4 switches are used to provide easy selection of the desired sound sample. Using one of the PWM inputs, a horn sound can be overlaid and once it finishes playing, normal sound resume. The horn/siren setting is programmable.

A PAM8403 Class D amplifier, with EMC fixes is also provided, this allows the system to drive 2x 8 ohm speakers at 2W. Volume control is provided as well as an option tu mute the amplifier and use an external amplifier, if desired.

Current state as of 11th April 2021.
Design is working well. Had to wire link the first PCBs as the switch inputs were not connected to an analogue input. The Schematic and PCB shown here are the corrected files.
Software first release is attached to the repository.

Power is from a 3.7-12V supply or the receiver BEC circuit. Unit has a polyfuse to protect against reverse polarity and a linear regulator.
![Alt text](https://github.com/istedman/RC_Sounds/blob/main/RC_Sounds_schematic.png?raw=true "Schematic")
![Alt text](https://github.com/istedman/RC_Sounds/blob/main/RC_Sounds_3d_PCB.png?raw=true "PCB")

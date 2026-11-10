# Electrical Information

Our electrical schematic below shows how our Raspberry Pi 4 connects to our quadrature motors and motor controller boards. It also highlights what connections are connected directly to our 12V batteries and what connections are connected to our regulated 5V rails. All of our 5V connections are spliced together with a common ground. Our 12V connections do the same.

We soldered together all of our connections, except for the wiring for our motors. These connections needed jumper cables so we used a pin-to-pin jumper connected to a pin-to-socket jumper to give us the length we needed. We wrapped the connection between the pin and socket with Kapton tape for added security and isolation. We also added a Raspberry Pi hat to the Raspberry Pi 4 that allowed us to solder those connections to ensure they would not vibrate loose. 

We added 5V regulators to our circuit to provide surge protection for our microcontrollers. This was needed since neither the Jetson Nano nor the Raspberry Pi 4 have proper over-voltage protection. Both of these microcontrollers need a microSD card. A 16 GB microSD will suffice for the features we have added, but to allow for new features, we would strongly recommend at least a 32 GB microSD card.

The LiDAR connects to the Jetson Nano via the USB ports. The Raspberry Pi 4 connects to the Jetson Nano via the CSI connector port. Due to the nominal current requirements of these two peripherals and the current needs of the Jetson Nano, it is necessary to power the Jetson Nano with the barrel jack rather than the micro-USB connection.

#### Electrical Schematic
![Icarus Electrical Schematic](/docs/Electrical/Icarus%20Electrical%20Schematic.png "Icarus Electrical Schematic")

#### Motor Pinout Table
![Icarus Motor Connections Table](/docs/Electrical/Motor%20Connections%20Table.png "Icarus Motor Connections Table")

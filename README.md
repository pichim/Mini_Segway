# MiniSegway
The Segway is a robot that has the ability to balance itself using two motors and an IMU sensor. It is controlled with a radio transmitter in two modes, a car where it moves on the ground maintaining three points of support but also as a segway, that is, balancing on two wheels. 

<!-- TODO picture of the robot -->
## Table of Contents
1. [Repository structure](#repository-structure)
2. [Hardware](#hardware)
3. [Prerequisites](#prerequisites)
4. [User manual](#user-manual)
5. [Software](#software)
6. [Notes](#notes)


## Repository structure
- ``/docs/cad`` - cad files with division into different extensions, including print-ready STL files and STEP files. <br>
- ``/docs/hardware`` - folder contains all hardware's datasheets and list <br>
- ``/docs/matlab`` - folder contains all development files in matlab <br>
- ``/docs/papers`` - folder contains all scientific papers regarding segway development 

## Hardware
Components and sensors employed in the design:
- Nucleo-F446RE
- Pololu Dual MC33926 Motor Driver Shield for Arduino
- 47:1 Metal Gearmotor 25Dx67L mm MP 12V with 48 CPR Encoder
- MPU6500 - 6DOF IMU
- OpenLager Blackbox
- TBS CROSSFIRE Nano RX
- 0,28" Mini Digital-Voltmeter mit LED Anzeige, 3,2-30V, 2-Wire, red
- Pololu mini push-button switch with reverse polarity protection, LV
Article no.: POL2808
- 3 x Mini Pushbutton Switch: PCB-Mount, 2-Pin, SPST, 50mA (5-Pack)
- 2 x LED diode
- 2 x Conrad energy NiMH battery packs 6V, 2300mAh
- Jumper wires

All frame parts 3D models are localized in [CAD folder](/docs/cad/)
WILL BE UPDATED

<!-- TODO Add all the links -->
<details Closed>
<summary>Links to hardware</summary>

[Nucleo-F446RE][1] <br> 
[Pololu Dual MC33926 Motor Driver Shield for Arduino][2] <br>
[47:1 Metal Gearmotor 25Dx67L mm MP 12V with 48 CPR Encoder][3] <br>
[MPU6500 - 6DOF IMU][4] <br>
[OpenLager Blackbox][5] <br>
[TBS CROSSFIRE Nano RX][6] <br>
[0,28" Mini Digital-Voltmeter mit LED Anzeige, 3,2-30V, 2-Wire, red][7] <br>
[Pololu mini push-button switch][8] <br>
[Mini Pushbutton Switch: PCB-Mount][9] <br>
[NiMH battery packs 6V, 2300mAh][10] <br>

</details>
<br>

## Prerequisites
- Mbed Studio
- Libraries:
    - mbed-os 6.17.0
    - eigen

All the libraries you need are included in the repository. Software is mostly based on files located in /src folder and main logic is included in MiniSegway class that is running in own thread.

## User manual

### Running the project for the first time
In order to run the project you need to import program to Mbed Studio and follow next steps:
- as the board is using external power source, board has jumper JP5 switched to E5V postion which requires to first supply board with external power and then plug it in to the computer
- now program can be compiled and flashed to the board

### Using robot
In order to use the robot you must follow the steps below:
- Start the radio controller that is paired with the receiver on the robot
- Turn on the power supply on the robot (in this situation both LED, green and blue will start blinking)
- Arm the radio controler (SD button) (in such case the green LED will start to shine continuously)
- Then the USER button on the robot can be pressed, which will lead the robot to CAR mode (blue LED will start to shine continuously)
- To use the robot as segway, raise it to a standing position and wait until the robot visibly starts to balance.

## Software



## Notes:
- Update dependencies after releasing drivers! 

[1]: https://os.mbed.com/platforms/ST-Nucleo-F446RE/
[2]: https://www.play-zone.ch/en/pololu-dual-mc33926-motor-driver-shield-for-arduino.html
[3]: https://www.pololu.com/product/4865
[4]: https://www.smart-prototyping.com/MPU6500-6DOF-Sensor-Breakout-Board
[5]: https://www.racedayquads.com/products/airbot-openlager-blackbox
[6]: https://www.team-blacksheep.com/products/prod:crossfire_nano_rx
[7]: https://www.berrybase.ch/0-28-mini-digital-voltmeter-mit-led-anzeige-3-2-30v-2-wire-rot
[8]: https://www.pololu.com/product/2808
[9]: https://www.pololu.com/product/1400
[10]: https://www.conrad.ch/de/p/reely-modellbau-akkupack-nimh-6-v-2300-mah-zellen-zahl-5-mignon-aa-side-by-side-jr-buchse-2613252.html
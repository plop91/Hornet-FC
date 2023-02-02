# Hornet Flight Controller

Ian Sodersjerna

## Mission Statement

My goal for this project is to create a flight controller that can run on many different sets of hardware and is capable of many different control types including RC controlled, AI controlled and WIFI controlled.  

## Personal Note

This flight controller project was started the end of summer of 2022. It is something I am developing as I find time between school and work. 

## Hardware design

The Hornet is designed using easy to source off the shelf modules from sites such as Amazon. A Gerber file, wiring diagram and BOM will be available once the first prototype has been built and tested.

The flight controller has 10 PWM outputs, 2 HV channels controlled by MOSFETS, a 9-axis IMU (w/ built in Kalman filter), Barometer, Temperature sensor, GPS, SBUS (Radio control of aircraft) and NRF 905 Radio (telemetry).


## Ideas

### Swarm mode

* This mode would enable communication between hornet FC's
* This would require high bandwidth comms with the FC, and likely require a camera and more powerful micro-controller

## V2 board

* Step away from modules instead create a fully integrated board
* vtx support and possibly integrate MAX7456 for OSD
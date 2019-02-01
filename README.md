# ESPlorer I - Arduino robot controller

### Arduino advanced robot firmware

#### Works with the ESP32 Arduino platform. Refer to https://github.com/espressif/arduino-esp32

## Contents
 - [About](#about)
 - [Installing](#installing)
 - [Known issues](#known-issues)
 - [How it looks](#how-it-looks)
 - [Releases](#releases)
 - [Thanks](#thanks)

## About

This is a firmware for the ESPlorer I robot.

But can control any robots 4WD, 2WD or tank. (need some work to another motor drive than the DRV8833)

The program is for Arduino ESP32, have advanced features as:

    - Web control (not need an app, just an browser)
    - Motor control by virtual joystick
    - Slow speed by PMW over PWM, with higher torque
    - Optional OV7670 VGA camera with servo
    - Accelerometer to anti-rollover system (not yet develop)


## Installing

First have some library dependencies (please help the authors give a github start on it):

    - Arduino websocket:        https://github.com/Links2004/arduinoWebSockets
    - RemoteDebug:              https://github.com/JoaoLopesF/RemoteDebug
    - SerialDebug (optional):   https://github.com/JoaoLopesF/SerialDebug

After install it, just upload this firmware to ESP32

## Schematics

Schematics of robot controller:

![Schematics]
(images/Schematic_ESPlorer-v1.png)

### Knew issues

  - Wrong colors on camera
  - Not working on L298 yet
  - Documentation is in construction

## How it looks

See this firmware in action on ESPlorer I robot:

[![Youtube](https://img.youtube.com/vi/KUckqEnlK_E/0.jpg)](https://https://www.youtube.com/watch?v=KUckqEnlK_E)

## Releases

#### 0.1.0 - 2018-02-01

- First Beta
    
## Thanks

First thanks Igrr for code of OV7670. https://github.com/igrr/esp32-cam-demo
And Mudassar for put it working with socket server: https://github.com/mudassar-tamboli/ESP32-OV7670-WebSocket-Camera
(this code is based on Iggr code)

And yoannmoinet for javascript virtual joystick  https://github.com/yoannmoinet/nipplejs


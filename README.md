# 4m20-coursework2
matlab/python/arduino code for controlling the robot

## Hardware summary

### Actuators
Servos capable of slightly under 180 degrees of motion. Limited mechanically in the positive direction, and electronically in the negative direction.

Servos are connected through a torque spring to the joint, which causes the motion to be less rigid.

### Sensors
There are potentiometers that measure the angular displacement of each of the torque springs (relative to the motor shaft), which give an analog reading to the controller. This gives an indication of torque.

## Messaging protocol

The rationale for using a custom message stack over the builtin matlab one is that:

* We're not tied to using matlab
* We can ensure that all the joint angles arrive in the same message - setting joints becomes atomic, rather than risking one being dropped
* We don't need to re-implement arduino sensor drivers in matlab (like the MPU9250)

We don't need to start using this immediately in matlab - switching later should not be hard.

The messaging stack, from low to high level is as follows:

* transport level - USB serial link
* framing level - COBS ([Consistent-Overhead-Byte-Stuffing](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing)), with null bytes between messages. This is so that if a message is corrupted, we always can recover and find the next message
* message level - a single letter to denote the type of packet, followed by little-endian binary data to be read into a struct.

This requires the code in the `arduino` directory to be put on the arduino.

This repo either contains or should contain matlab and/or python code that processes these messages at the PC end

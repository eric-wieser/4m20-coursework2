Arduino code
============

Download the [latest arduino software](https://www.arduino.cc/en/Main/Software), and open the arduino.ino file, so named because this file has to be the same name as the directory containing it.

The arduino code implements a simple but robust messaging scheme, to allow the actuators to be controlled and the sensors read over a serial connection.

The description of each message type can be found in `messages.h`


If using the builtin matlab `arduino` API, then this code will not be used, as matlab replaces the arduino software with its own code.
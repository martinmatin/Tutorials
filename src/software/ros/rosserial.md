# ROS and Arduino

## What is rosserial ?

« Rosserial is a protocol for wrapping standard ROS serialized messages and multiplexing multiple topics and services over a character device such as a serial port or network socket.
Rosserial provides a ROS communication protocol that works over your Arduino's UART. It allows your Arduino to be a full fledged ROS node which can directly publish and subscribe to ROS messages, publish TF transforms, and get the ROS system time.
The rosserial protocol is aimed at point-to-point ROS communications over a serial transmission line. We use the same serialization/de-serialization as standard ROS messages, simply adding a packet header and tail which allows multiple topics to share a common serial link. »

These explanations come from the following websites :

http://wiki.ros.org/rosserial

http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

Now you know what rosserial is used for, but before testing you have to install some packages.

## Rosserial installation

To install and use rosserial you have to run the terminal, first update apt-get which allows you to install packages :

```
sudo apt-get update
```
You can choose to install the packages one by one :

First type :
```
sudo apt-get install arduino
```
Then after the first installation :
```
sudo apt-get install ros-kinetic-rosserial
```
And then :
```
sudo apt-get install ros-kinetic-rosserial-arduino
```

Or you can directly install all the packages with one command in the terminal :
```
sudo apt-get install -y \
                arduino \
                ros-kinetic-rosserial-arduino \
                ros-kinetic-rosserial
```

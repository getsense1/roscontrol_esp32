## rosserial_esp32 hardware joint


This [rosserial](http://wiki.ros.org/rosserial) ros_lib ESP32 port 
that enable communication between ROS(Kinetic on Ubuntu Trusty 14.04 LTS) node and ESP32 framework using ESP-IDF TCP stack.


```
how work
$ git clone https://github.com/getsense1/roscontrol_esp32
$ cd ./rosserial_esp32
$ cp -fr ./esp-idf/components/rosserial_esp32 esp/esp-idf/components/
copy main/_main.c, main/esp_chatter.cpp, include to IDF project folder
build and flash bin image to ESP32 board 
```


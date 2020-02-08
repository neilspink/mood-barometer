# Arduino mood barometer with ESP32

This project implements an analog display that gets its data from an [MQTT broker](http://mqtt.org/). 

![Mood Barometer Front](picture-front.jpg)

![Mood Barometer Back](picture-back.jpg)

![Mood Barometer Back](picture-back-cover.jpg)

Download link for construction of the housing in Fusion 360

http://a360.co/2nmHa7m

The ESP32 development board I used features a built-in hall effect sensor, it detects changes in the magnetic field in its surroundings, i.e. the magnet mounted on the clock-arm. 

The idea and housing came from a [blog post](https://www.bastelgarage.ch/index.php?route=extension/d_blog_module/post&post_id=10) on a Swiss onlineshop for DIY electronics [www.bastelgarage.ch](www.bastelgarage.ch). 

## Components

* NodeMCU-32S ESP32 WiFi Bluetooth development board
* Stepper motor 28BYJ-28 with a ULN2003 driver board
* 6x10mm neodymium bar magnet

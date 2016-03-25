# SmartContainer
My submission for Arduino Maker Challenge 2016: https://www.hackster.io/challenges/arduino-microsoft-maker/ : the contest requires to use arduino and Windows 10.

## page under construction

## The Project
A special container for store objects, with sensors inside, ready for iot.
This container is able to keep track of stocks stored inside it and react:
- sending notification to the cloud
- turn an alarm led on 
when the stocks are low.

An LCD show information on the container like: label or setup information.
Throught a web interface is possible to control the status of all the smart containers available.
An universal windows app allows to configure some settings: wifi network,wifi password, label, bottom threshold(content under threshold are considered low), position/location.
The container is powered by a battery and a rechargeable circuit, thanks to the MKR1000.

### Scenario
Containers are everywhere, so this is also useful:
- store/warehouse/depot: keep track of materials to buy automatically and phisically view which one are low (thanks to Led notification). This action helps anyone to understand immediately the status of the container... without opening it or control it on the server.
- house: keep track of items or food. What should I have to buy to today? check the smart containers status!

## Hardware / Material List
- a load cell in the bottom
- a 2 line display in the front panel
- a light or a led for visual notification
- a lipo battery
- arduino MKR1000

## Software

### Arduino
The project is tested with the MKR1000: maybe it works also with Yun and other wifi-arduinos.

### Universal Windows App
The windows 10 application helps the user to configure the container.
The user can plug the USB of the container to the windows 10 device (a tablet for example), launch the app and configure the container:
- label
- position
- threshold
- wifi settings: ssid and password

After setup is completed the container can be unplugged.
The software use the Firmata, Remote Wiring and Serial as dependency.

### Web Interface

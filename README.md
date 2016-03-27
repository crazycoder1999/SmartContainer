
# SmartContainer
My submission for Arduino Maker Challenge 2016: https://www.hackster.io/challenges/arduino-microsoft-maker/ : the contest requires to use arduino and Windows 10.

## The Project
A special container for store objects, with sensors inside, iot-ready.
This container is able to keep track of stocks stored inside it and react:
- sending notification to the cloud
- turn an alarm led on 
when the stocks are low.

An LCD show information on the container like: label or setup information.
Throught a web interface is possible to control the status of all the registered smart containers.
An universal windows app allows to configure some settings: wifi network,wifi password, label, bottom threshold(content under threshold are considered low), position/location.
The circuit can be powered by:
- 5V with a micro usb cable
- battery with a lipo battery, 3.7V at least 700 mhA (check the specs).
The arduino MKR1000 contains a recharge circuit.

### Scenario
Everywhere you can replace a simple container with a smart container:
- store/warehouse/depot: keep track of materials to buy automatically and "phisically" view which one are low (thanks to Led notification). This last action helps anyone to understand immediately the status of the container... without opening it or control it on the server.
- house: keep track of items or food to buy. What should I have to buy for launch today? check the smart containers page!

## Hardware / Material List
- a load cell with HX711 driver
- a 2 line display LCD / HD44780: for content information
- a light or a led for visual notification
- a lipo battery (optional, you can leave it plugged to 5V)
- Arduino MKR1000

## Software

### Arduino
Some configuration settings are hard-coded like server address and port.
The code is based on standard firmata example: I tried to reduce it to the essential, but I had problems with the UWA..that's why I didn't changed a lot.
The project is tested with the MKR1000: maybe it works also with Yun and other wifi-arduinos.
The circuit is in the Arduino folder of this repository.
The code:
- First, it configures the lcd display: it is used for sending information to the user
- Prepare the firmata library configuration. This is the last step for configuration
- Read the wifi MacAddress: this is the identifier of the device (it is enough for now)
- Wait for connection from the client UWA application, and a firmata "UPDATEME" message
- When "UPDATEME" message is arrived, the code parse the parameters received from the client: wifi ssid, wifi pwd, label to display on the container. The parameters have this format: "wifissid;wifipwd;label;"
- The code sends back to the UWA the macAddress of the container.
- The arduino establish a connection to the wifi network
- The code configure the load cell and set the tare (The container should be empty)
- The final loop: the code periodically checks the weight of the container
	- if the content is changed: it send an HTTP get requests to the server with the macAddress and new weight
	- the server can send back a "1" or "0". "1" means the new weight is under threshold, led notification should be ON. "0" otherwise.

NOTE: It is not actually possible to switch off the wifi. Wifi should be switched off all the time and turned on, only when there is a change to notify, than switched off again.

### Universal Windows App
The windows 10 application helps the user to configure the container.
The user can plug the USB of the container to the windows 10 device (a tablet for example), launch the app and configure the container:
- label
- position
- threshold
- wifi settings: ssid and password
The UWA send to the container the parameters: wifi ssid, wifi pwd, label.
From the arduino, it receive the macAddress than, it sends an http response to the server to:
- register the container with the macAddress
- update container informations: label, threshold, positions

The configuration-mode is available only on powerup for this version.
After setup is completed the container can be unplugged: if battery is present and charged :)
To compile the code you need these dependencies from Microsoft github: Firmata, Remote Wiring and Serial.

### Server Code
Server side code is made with flask and python.
It is my first webapp with flask: I found very easy at least for this project.
The information are saved with python shelve: it is _enough_ for this demo.
The frontend use bootstrap and some simple javascript code, all inside list.html (sorry :) )


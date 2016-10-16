# README

Small python app to send data from sensors and drone telemetry.

# Setup

to enable i2c & SPI

`sudo raspi-config`

`sudo apt-get update sudo apt-get upgrade`

`sudo apt-get install i2c-tools sudo adduser pi i2c`

Pin 3 (SDA) -> Arduino A4 (SDA)
Pin 5 (SCL) -> Arduino A5 (SCL)

i2c python support

`sudo apt-get install python-smbus`

UART python support

`sudo apt-get install python-serial`

Python libs and Mavlink Proxy

`sudo apt-get install screen python-wxgtk2.8 python-matplotlib python-opencv python-pip python-numpy python-dev libxml2-dev libxslt-dev sudo pip install pymavlink sudo pip install mavproxy`

`sudo -s`


`mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --aircraft DroneName`

Via USB cable

`mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --aircraft DroneName`

DroneKit framework

`sudo apt-get install python-pip python-dev python-numpy python-opencv python-serial python-pyparsing python-wxgtk2.8 libxml2-dev libxslt-dev`

`sudo pip install droneapi echo "module load droneapi.module.api" >> ~/.mavinit.scr`

`pip install dronekit pip install dronekit-sitl`

Install DHT22 sensor libraries

`sudo apt-get update sudo apt-get install build-essential python-dev`

`git clone https://github.com/adafruit/Adafruit_Python_DHT.git cd Adafruit_Python_DHT`

`sudo python setup.py install`

The MPL3115A2 requires a proper repeated start command in it's I2C communication, this line is needed every startup.

`sudo su - echo -n 1 > /sys/module/i2c_bcm2708/parameters/combined exit`


# Testing

List avalible addresses

`i2cdetect -y 1`

Find your USB device

`ls /dev/tty*`

# Troubleshooting

To fix some troubles install pip with

`sudo easy_install pip`

To fix get_page error, correct date & time is needed

`sudo date -s "Thu Aug 9 00:00:00 UTC 2016"`

Some SSH client make locale errors, try installing all in Raspi terminal.


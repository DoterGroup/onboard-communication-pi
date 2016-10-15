# Sudo grants
sudo su

# Requiered for enable i2c
echo -n 1 > /sys/module/i2c_bcm2708/parameters/combined

# Launch MavProxy
mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --aircraft Sweeper
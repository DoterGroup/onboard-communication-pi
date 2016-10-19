
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

import spidev
import sys
import time
import argparse 
import json

import smbus
import serial
import Adafruit_DHT

bus = smbus.SMBus(1)

# Connecting telemetry module
try:
    ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=5)
except e:
    print "Serial is not avalible."

# Connecting Mavproxy
try:
    global vehicle
    parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
    parser.add_argument('--connect', default='127.0.0.1:14550',help="vehicle connection target. Default '127.0.0.1:14550'")
    args = parser.parse_args()
    print "\nConnecting to vehicle on: %s" % args.connect
    vehicle = connect(args.connect, wait_ready=True)
    vehicle = connect('127.0.0.1:14550', wait_ready=True)

except Exception, e:
   print "Error to Connect"

# BGN: GENERAL FUNCTIONS

def writeNumber(value):
    bus.write_byte(address, int(value))
    return -1

def readNumber():
    number = bus.read_byte(address)
    return number

def set_arm(arm):
    if arm:
        print "Arming motors"
        vehicle.armed = True
    else:
        print "Disarming motors"
        vehicle.armed = False


def set_mode(mode):
    if mode == 1:
        vehicle.mode = VehicleMode("STABILIZE")

    if mode == 2:
        vehicle.mode = VehicleMode("ALT_HOLD")

    if mode == 3:
        vehicle.mode = VehicleMode("LOITER")

    if mode == 4:
        vehicle.mode = VehicleMode("RTL")

    if mode == 5:
        vehicle.mode = VehicleMode("AUTO")

    print "Change mode: " + str(mode)

def takeoff(t):
    vehicle.commands.takeoff(t)
    print "takeoff at " + str(t)

# Function to read SPI data from MCP3008 chip
# Channel must be an integer 0-7
def ReadChannel(channel):
  adc = spi.xfer2([1,(8+channel)<<4,0])
  data = ((adc[1]&3) << 8) + adc[2]
  return data
 
# Function to convert data to voltage level,
# rounded to specified number of decimal places.
def ConvertVolts(data,places):
  volts = (data * 3.3) / float(1023)
  volts = round(volts,places)
  return volts
 


# END: GENERAL FUNCTIONS


# MAIN LOOP
def main():
    try:
        while True:
            ## Vehicle data ##

            lat = vehicle.location.global_frame.lat
            lon = vehicle.location.global_frame.lon
            alt = vehicle.location.global_frame.alt

            arm = str(vehicle.armed)
            mode = str(vehicle.mode.name)
            batt = str(vehicle.battery.level)

            vel = str(vehicle.velocity)
            ground_speed = str(vehicle.groundspeed)
            air_speed = str(vehicle.airspeed)

            print "Mode: %s" % mode
            print "Latitud: " + str(lat)
            print "Longitud: " + str(lon)
            print "Attitude: " + str(alt)

            data = "[]"


            ## Sending data ##

            pressure = random.randrange(100)

            celsius = random.randrange(50)

            humidity = random.randrange(100)

            temperature = random.randrange(50)

            co_level = random.randrange(200)


            data = '[' + str(arm) + ',' + str(mode) + ',' + str(batt) + ',' + str(lat) + ',' + str(lon) + ',' + str(alt) + ',' + str(pressure+p_decimal) + ',' + str(celsius) + ',' + str(humidity) + ',' + str(temperature) + ',' + str(ground_speed) + ',' + str(co_level) + ']'

            ser.write(str(data) + '\n')
            time.sleep(0.1)

            callback = ''

            if ser.inWaiting() > 1:
                print "Reading data..."
                callback = ser.readline()
                command = callback.split(':')
                print "%s:%s" % (str(command[0]), str(command[1]))
                if str(command[0]) == 'mode':
                    set_mode(int(command[1]))

                ser.flush()

    except KeyboardInterrupt:
        GPIO.cleanup()
        ser.close()


if __name__ == '__main__':
    print "==Inicio=="
    main()

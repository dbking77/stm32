#!/usr/bin/env python

"""
Usage: logdata.py <dev> <file>

Args:
  dev : serial device ie: /dev/ttyUSB0

"""

import sys
import serial


def usage():
    print __doc__

def main():
    argv = sys.argv
    if len(argv) != 3:
        usage()
        sys.exit(1)

    devname = argv[1]
    fn = argv[2]

    fd = open(fn, 'wb')

    ser = serial.Serial(devname, 38400, timeout=1.0)
    ser.write('g')

    while True:
        line = ser.readline()
        if line is not None:
            print line
	    fd.write(line)
	ser.write('g')

    ser.close()
    

main()

    

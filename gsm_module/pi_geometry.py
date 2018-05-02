#!/usr/bin/python3

__author__ = 'chandrahassingh'
'''
This script is used to find latitude and longitude using AT command with raspberry pi and GSM module
'''

import time
import serial


class GSMLOCATION:

    # constructor definition
    def __init__(self):
        self.__gsm_port = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

    def radio_send(self, radioTime, radioLat, radioLong, radioAccuracy, radioAltitude):
        ser = serial.Serial("/dev/ttyUSB1",
                            baudrate=9600,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            writeTimeout=0,
                            timeout=1)
        time.sleep(1)
        writeString = "Time {0} Latitude {1} Longitude {2} Accuracy {3} Altitude {4}".format(str(radioTime),
                                                                                                 str(radioLat),
                                                                                                 str(radioLong),
                                                                                                 str(radioAccuracy),
                                                                                                 str(radioAltitude))
        ser.write(writeString + "\n\r")
        ser.close()

    def process_data(self, radioTime, radioLat, radioLong, radioAccuracy, radioAltitude):
        lat1 = float(radioLat) / 100
        lat2, lat3 = divmod(lat1, 1)
        latitude = lat2 + (lat3) / 60
        long1 = float(radioLong) / 100
        long2, long3 = divmod(long1, 1)
        longitude = long2 + (long3) / 60
        if radioAccuracy == 0:
            position = "Fixed"
        elif radioAltitude == 1:
            position = "Not Fixed"
        print "Time {0} Latitude {1} Longitude {2} Accuracy {3} Altitude {4}".format(str(radioTime),
                                                                                     str(radioLat),
                                                                                     str(radioLong),
                                                                                     str(radioAccuracy),
                                                                                     str(radioAltitude))
        self.radio_send(str(radioTime), str(latitude), str(longitude), str(radioAccuracy), str(radioAltitude))
        self.gsm_hit(str(radioTime), str(latitude), str(longitude), str(radioAltitude), str(radioAltitude))

    def gsm_init(self):
        self.__gsm_port.write('AT' + '\r\n')
        time.sleep(0.5)
        print(self.__gsm_port.read(100))
        self.__gsm_port.write('AT+CGATT=1' + '\r\n')
        time.sleep(0.5)
        print(self.__gsm_port.read(100))
        self.__gsm_port.write('AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"' + '\r\n')
        time.sleep(0.5)
        print(self.__gsm_port.read(100))
        self.__gsm_port.write('AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"' + '\r\n')
        time.sleep(0.5)
        print(self.__gsm_port.read(100))
        self.__gsm_port.write("AT+SAPBR=1,1" + '\r\n')
        time.sleep(0.5)
        print(self.__gsm_port.read(100))
        self.__gsm_port.write("AT+HTTPINIT" + '\r\n')
        time.sleep(0.5)
        print(self.__gsm_port.read(100))

    def gsm_hit(self, radioTime, radioLat, radioLong, radioAccuracy, radioAltitude):
        self.__gsm_port.write("AT+HTTPPARA=\"CID\",1" + '\r\n')
        time.sleep(0.5)
        print(self.__gsm_port.read(100))
        self.__gsm_port.write("AT+HTTPPARA=\"URL\",\"http://www.chandrahassingh.com/gps.php?")
        time.sleep(0.5)
        print(self.__gsm_port.read(100))
        self.__gsm_port.write("Time=" + str(radioTime) + "&Latitude=" + str(radioLat) + "&Longitude=" + str(radioLong)
                              + "&Altitude=" + str(radioAccuracy) + "&Position=" + str(radioAltitude) + "\"\r\n")
        time.sleep(2)
        print(self.__gsm_port.read(100))
        self.__gsm_port.write("AT+HTTPACTION=1" + '\r\n')
        time.sleep(0.5)
        print(self.__gsm_port.read(100))
        self.__gsm_port.write("AT+HTTPREAD=0,1000" + '\r\n')
        time.sleep(0.5)
        print(self.__gsm_port.read(100))

    def gsm_end(self):
        self.__gsm_port.write("AT+HTTPTERM" + '\r\n')
        time.sleep(0.5)
        print(self.__gsm_port.read(100))
        self.__gsm_port.write("AT+SAPBR=0,1" + '\r\n')
        time.sleep(0.5)

    def read_data(self):
        ser = serial.Serial(port='/dev/ttyUSB2',
                            baudrate=9600,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            timeout=1)
        while True:
            x = ser.readline()
            for line in x.split('\n'):
                if line.startswith('$GNGGA'):
                    counter = line.strip().split(',')
                    print "Time={0} Latitude={1} Longitude={2} Accuracy={3} Altitude={4}".format(str(counter[1]),
                                                                                                 str(counter[2]),
                                                                                                 str(counter[4]),
                                                                                                 str(counter[6]),
                                                                                                 str(counter[7]))
                    # give some valid name below
                    p = float(counter[1])
                    q = float(counter[2])
                    r = float(counter[4])
                    s = float(counter[6])
                    t = float(counter[7])
                    self.process_data(p, q, r, s, t)


def main():
    obj = GSMLOCATION()
    run = 0
    while True:
        if run == 0:
            obj.gsm_end()
            obj.gsm_init()
            run = 1
        obj.read_data()


if __name__ == "__main__":
    main()

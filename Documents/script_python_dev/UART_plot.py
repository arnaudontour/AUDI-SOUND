# -*- coding: utf-8 -*-
"""
Created on Wed Jun 14 15:08:14 2023

@author: 434926
"""


import serial
import matplotlib.pyplot as plt



freq = []
data = []



ser = serial.Serial(
    port='COM6',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("connected to: " + ser.portstr)
count=1

while True:
    while (ser.read()) != b'a':
        print("waiting")
    d = ""
    print("coucou")
    f = ""
    count : 1
    while True :
        for line in ser.read():
            line = chr(line)
            if line == "" :
                count = count +1
            if count == "2":
                f = f + chr(line)
            if count == "4":
                d = d + chr(line)
            if count == "5":
                freq.append(f)
                data.append(d)
                break
            

print("lol")
ser.close()


plt.semilogx(freq,data)
#plt.axis([1000, 10000, 20, 50])
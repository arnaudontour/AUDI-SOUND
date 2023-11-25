# -*- coding: utf-8 -*-
"""
Created on Mon Jun 12 20:19:19 2023

@author: antoi
"""

import serial #Importation de la bibliothèque « pySerial »

ser = serial.Serial(port= "COM6", baudrate=115200,timeout=1)
ser.open() 

while true:


    line = ser.readline() 
    
    

ser.close() #Cloture du port pour le cas ou il serait déjà ouvert ailleurs
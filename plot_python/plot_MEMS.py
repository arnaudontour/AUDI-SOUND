# -*- coding: utf-8 -*-
"""
Created on Mon Jun 12 12:54:49 2023

@author: 434926
"""
import matplotlib.pyplot as plt

# Ouvrir le fichier en lecture seule
file = open('plot.txt', "r")
# utiliser readlines pour lire toutes les lignes du fichier
# La variable "lignes" est une liste contenant toutes les lignes du fichier
lines = file.readlines()
# fermez le fichier après avoir lu les lignes
file.close()

freq = []
data = []


# Itérer sur les lignes
for line in lines:
    a = line.split()
    print(a)
    data.append(float(a[2]))
    freq.append(float(a[0]))
 
plt.plot(freq,data)
#plt.axis([1000, 10000, 20, 50])
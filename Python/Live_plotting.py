# -*- coding: utf-8 -*-
"""
Created on Thu Aug 11 17:59:06 2016

@author: Antoine
"""

from python_parser_class import protocolParser
import time
from drawnow import *

import matplotlib.pyplot as plt
import pandas as pd
import msvcrt as ms
import numpy as np

plt.ion()


sensor_list = ['28ccbd0700008055','283e700300008089','2803fa0300008058'
,'28c3e70300008057','285bfe03000080c8','28bf92030000800d','28ffb107000080c0','28ff2ddd23160314','28ff71dd231603a0','28ffea8e3316037d','28ff6008201604f6','282df3030000800e']

columns_ids = ['Centre top','Neoprene','Bord centre','Bord fond','Centre centre','Bord top','Centre fond','Heatsink','Paroie fond','Paroie centre','Paroie top','Air Ambiant']


Temperature = [0,0,0,0,0,0,0,0,0,0,0,0]

def searchCallback(sensorID):
    print("Found sensor ID: " + ' '.join(format(x, '02x') for x in sensorID))

def readCallback(sensorID, temp):
    sen_id = 0
    for i in sensor_list:
        sensorID1 = ''.join(format(x, '02x') for x in sensorID)
        if i == sensorID1:
            Temperature[sen_id] = temp[0]
            
        sen_id +=1
            
#    print("temp: %0.2f" % temp)
#    print("Sensor ID: " + ' '.join(format(x, '02x') for x in sensorID))
 

def readScaleCallback(weight):
    print("Weight: %0.2f" % weight)

parser = protocolParser('com4', 115200)
parser.setCmdCallback(1, searchCallback)
parser.setCmdCallback(2, readCallback)
parser.setCmdCallback(10, readScaleCallback)
parser.printDebug(0)
parser.printAscii(0)
parser.start()

time.sleep(2)
parser.setCurrentCmd(0,3)
parser.setCurrentCmd(1,3)
parser.setCurrentCmd(2,3)
parser.setCurrentCmd(3,3)


def makeFig(): 
    plt.interactive(True)
#    plt.ylim(5,50)                                 
    plt.title('Temperature data Testbench')     
    plt.grid(True)                                  
    plt.ylabel('Temperature .C')   
                   
    plt.plot(T1_data, 'r-', label='Centre top : '+str(Temperature[0])+'.C')
    plt.pause(0.001)
    plt.plot(T2_data, 'b-', label='Neoprene : '+str(Temperature[1])+'.C')
    plt.plot(T3_data, 'g-', label='Bord centre : '+str(Temperature[2])+'.C')
    plt.plot(T4_data, 'c-', label='Bord fond : '+str(Temperature[3])+'.C')
    plt.plot(T5_data, 'm-', label='Centre centre : '+str(Temperature[4])+'.C')
    plt.plot(T6_data, 'y-', label='Bord top : '+str(Temperature[5])+'.C')
    plt.plot(T7_data, 'k-', label='Centre fond : '+str(Temperature[6])+'.C')
    plt.plot(T8_data, 'r-', label='Heatsink : '+str(Temperature[7])+'.C')
    plt.plot(T9_data, 'b-', label='Paroie fond : '+str(Temperature[8])+'.C')
    plt.plot(T10_data, 'g-', label='Paroie centre : '+str(Temperature[9])+'.C')
    plt.plot(T11_data, 'c-', label='Paroie top : '+str(Temperature[10])+'.C')
    plt.plot(T12_data, 'm-', label='Air Ambiant : '+str(Temperature[11])+'.C')
    
    plt.legend(loc='lower left')
    





T1_data = []
T2_data = []
T3_data = []
T4_data = []
T5_data = []
T6_data = []
T7_data = []
T8_data = []
T9_data = []
T10_data = []
T11_data = []
T12_data = []



timestep = 1
max_graph = 120*60


old = 0
now = time.strftime("%d-%m-%Y_%I-%M-%S")
temp_frame = pd.DataFrame([Temperature], columns=columns_ids)####,columns=['T1','T2','T3','T4','T5','T6','T7','T8','T9','T10','T11','T12']
temp_frame.to_csv(now+'.csv', sep=',')###
temps_old = 0
temps = 2

#parser.startCmd()

line = 000

while (True):
    
    parser.readCmd()
    T1_data.append(Temperature[0])
    T2_data.append(Temperature[1])
    T3_data.append(Temperature[2])
    T4_data.append(Temperature[3])
    T5_data.append(Temperature[4])
    T6_data.append(Temperature[5])
    T7_data.append(Temperature[6])
    T8_data.append(Temperature[7])
    T9_data.append(Temperature[8])
    T10_data.append(Temperature[9])
    T11_data.append(Temperature[10])
    T12_data.append(Temperature[11])
#    
    temp_frame = pd.concat([temp_frame,pd.DataFrame([Temperature], columns=columns_ids)],axis=0)
    
    temp_frame = temp_frame.set_index(np.arange(0,temps,timestep))
    
    drawnow(makeFig, show_once=True)

    temps += timestep
#    
    if(temps>max_graph):
        
        T1_data.pop(0)                       
        T2_data.pop(0)
        T3_data.pop(0)                       
        T4_data.pop(0)
        T5_data.pop(0)                       
        T6_data.pop(0)
        T7_data.pop(0)                       
        T8_data.pop(0)
        T9_data.pop(0)                       
        T10_data.pop(0)
        T11_data.pop(0)                       
        T12_data.pop(0)

    if (temps-temps_old>(5*60)):
       ###
        with open(now+'.csv', 'a') as f:
             temp_frame.to_csv(f, header=False)
             
        temp_frame = pd.DataFrame([Temperature], columns=columns_ids)   
        temps_old = temps
    
    time.sleep(timestep)
    
    if ms.kbhit():        
        line = input()
        
    if(line == "q"):
        break

parser.setCurrentCmd(0,0)
parser.setCurrentCmd(1,0)
parser.setCurrentCmd(2,0)
parser.setCurrentCmd(3,0)
parser.stop()
parser.join()
print("Exiting program")

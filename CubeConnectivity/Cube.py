#!/usr/bin/env python
import os
import sys
from collections import deque
import numpy as np
if os.name!='nt':
    import rospy
    from std_msgs.msg import *


class Cube(object):
    '''This is how '''
    def __init__(self,id,downface=None):
        self.id = id
        self.light_channel = "/cube_%s/serial"%id
        self.pub = rospy.Publisher(self.light_channel,Int8MultiArray)
        self.downFace = downface
        history_len = 10
        self.lightSensorHistory= np.zeros((history_len,6),dtype = 'bool')
        self.faceUpHistory = np.zeros((history_len),dtype = 'int')


    def _faceLEDBool(self,id,cmd):
        cmd_array=[False,False,False,False]
        if cmd: cmd_array=[True,True,True,True]
        self._faceLEDArray(id,cmd_array)    
         

    def _faceLEDArray(self,id,cmd):
        '''sets the LEDs for face based on the cmd array of length 4'''
        message = Int8MultiArray()
        self.pub(message)
        pass

    def faceLEDs(self,id,cmd):
        if type(cmd)==type(True): self._faceLEDBool(id,cmd)
        elif len(cmd)==4: self._faceLEDArray(id,cmd)
        

    def setMostRecentLightSensorState(self,state):
        self.lightSensorHistory = np.vstack((state,self.lightSensorHistory[:-1,:])).astype(bool)

    def setMostRecentFaceUp(self,id):
        self.faceUpHistory = np.hstack((id,self.faceUpHistory[:-1]))
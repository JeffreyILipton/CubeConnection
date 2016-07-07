#!/usr/bin/env python
import os
import sys
from collections import deque
import numpy as np
if os.name!='nt':
    import rospy
    from std_msgs.msg import *


class Cube(object):
    '''This is how to interface with a cube'''
    def __init__(self,id,downface=None):
        self.id = id
        self.light_channel = "/cube_%s/serial"%id
        self.pub = rospy.Publisher(self.light_channel,Int8MultiArray)
        self.downFace = downface
        history_len = 10
        self.lightSensorHistory= np.zeros((history_len,6),dtype = 'bool')
        self.faceUpHistory = np.zeros((history_len),dtype = 'int')
        self.faceUpHistory.fill(-1)
        self.faceConnections={0:(),1:(),2:(),3:(),4:(),5:()}
        self.unknown_faces=range(0,6)
        self.base_state = np.zeros((history_len),dtype = 'bool')


    def _faceLEDBool(self,id,cmd):
        cmd_array=[False,False,False,False]
        if cmd: cmd_array=[True,True,True,True]
        self._faceLEDArray(id,cmd_array)    
         

    def _faceLEDArray(self,id,cmd):
        '''sets the LEDs for face based on the cmd array of length 4'''
        vals = [int(x) for x in cmd]
        message = "irled_%i_%i%i%i%i"%(id,vals[0],vals[1],vals[2],vals[3])
        self.pub(message)
        

    def setAllFaceLEDs(self,cmd):
        for id in range(0,6):self.faceLEDs(cmd)

    def faceLEDs(self,id,cmd):
        if type(cmd)==type(True): self._faceLEDBool(id,cmd)
        elif len(cmd)==4: self._faceLEDArray(id,cmd)
        
    def connectFace(self,id,connection):
        self.unknown_faces = [x for x in self.unknown_faces if x!=id]
        self.faceConnections[id] = connection

    def setMostRecentLightSensorState(self,state):
        self.lightSensorHistory = np.vstack((state,self.lightSensorHistory[:-1,:])).astype(bool)

    def setMostRecentFaceUp(self,id):
        self.faceUpHistory = np.hstack((id,self.faceUpHistory[:-1]))

    def getMostRecentLightSensorState(self):
        return self.lightSensorHistory[0,:]

    def getMostRecentFaceUp(self):
        return self.faceUpHistory[0]

def faceUpToDown(face):
    faces = { 0:2,
              1:3,
              2:0,
              3:1,
              4:5,
              5:4
            }
    return faces[face]
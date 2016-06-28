#!/usr/bin/env python
import os
import sys
import numpy as np
if os.name!='nt':
    import rospy
    from rospy.numpy_msg import numpy_msg
    from std_msgs.msg import *

from Cube import *


class ConnectionManager(object):
    def __init__(self,initial_cube_id_list):
        rospy.init_node('ConnectionManager')
        rospy.Suscriber('mblocks_light',numpy_msg(Int8MultiArray),self.parseState)
        
        self.cubes={}
        self.updated = False
        self.lasttime = 0
        for cube_id in initial_cube_id_list:
            cubes[cube_id] = Cube(cube_id)


    def parseState(self,message_array):
        system_time = message_array[0]
        message = np.delete(message_array.flatten(),0)
        length = message.shape[0]
        nrows = int(length/8)
        if length%nrows !=0: 
            print "Error cant reshape"
            return

        message = message.reshape(nrows,8)
        for row_id in nrows:
            row = message[row_id]
            cube_id = row[0]
            if cube_id not in self.cubes.keys():
                self.cubes[cube_id] = Cube(cube_id)
            faceup = row[1]
            sensors = row[2:]
            cube = self.cubes[cube_id]
            cube.setMostRecentFaceUp(faceup)
            cube.setMostRecentLightSensorState(sensors)
        self.updated = True
        self.lasttime = system_time

    
            


    

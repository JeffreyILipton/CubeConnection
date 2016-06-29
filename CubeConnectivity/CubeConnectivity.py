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
        self.channel = 'mblocks_light'
        self.channel_type = numpy_msg(Int8MultiArray)
        self.state_sub =  rospy.Suscriber(self.channel,self.channel_type,self.parseState)
        
        self.cubes={}
        self.lasttime = 0
        self.ambient_threshold = 1
        for cube_id in initial_cube_id_list:
            cubes[cube_id] = Cube(cube_id)



    def parseState(self,message_array):
        system_time = message_array[0]
        if system_time == self.lasttime: return
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
            sensors = sensors<self.ambient_threshold
            cube = self.cubes[cube_id]
            cube.setMostRecentFaceUp(faceup)
            cube.setMostRecentLightSensorState(sensors)

        self.lasttime = system_time

    def updateStates(self):
        system_state = rospy.waitForMessage(self.channel,self.channel_type)
        self.parseState(system_state)
    
    def runConnections(self):
        self.state_sub.unregister()
        #turn off all LEDs
        for cube_id in self.cubes.keys():
            self.cubes[cube_id].faceLEDs(False)
        # get the state of the system with all lights off
        self.updateStates()

        # eliminate faces from testing while developing the base state
        for cube_id in self.cubes.keys():
            cube = self.cubes[cube_id]
            cur_state = cube.getMostRecentLightSensorState()
            cube.base_state = cur_state
            # remove the down face from testing, the face bellow it will light up
            faces = [x for x in range(0,6) if x != faceUpToDown(cube.faceUpHistory[0])]
            # remove the states that are already light up by the ambient light
            faces = [x for x in faces if not cur_state[x]]
            cube_faces_to_test[cube_id] = faces
            cube.unknown_faces = faces

        # Build connections without angles
        for cube_id in self.cubes.keys():
            cube = self.cubes[cube_id]
            faces = cube_faces_to_test[cube_id]
            for face in faces:
                cube.faceLEDs(True)
                ## pause for the signal to go through?
                self.updateStates()
                connection = DiffStates(self.cubes)
                if len(connection)>0:
                    cube.connectFace(face,connection)
                    other_cube = self.cubes[connection[0]]
                    other_cube.connectFace(connection[1],(cube_id,face))
        #We now have all of the connections
        
        # now we can orient the cubes
        #the unknowns are the connnected faces
        for cube_id in self.cubes.keys():
            cube = self.cubes[cube_id]
            cube.unknown_faces = cube.faceConnections.keys()


                ###



def DiffStates(cube_dict):
    cubes_that_saw_the_light=[]
    for cube_id in cube_dict.keys():
        cube = cube_dict[cube_id]
        current = cube.getMostRecentLightSensorState()
        base_state = cube.base_state
        changes = np.logical_and(np.logical_not(base_state),np.logical_xor(base_state,current))
        saw_the_light = np.logical_or.reduce(changes)

        if saw_the_light: 
            faces = list(np.where(changes==True)[0]) # its one D
            cubes_that_saw_the_light.append((cube_id,faces[0]))
            if len(faces > 1): print "multiple faces saw the light"
            # should be able to return now
    if len(cubes_that_saw_the_light)>1 : print "multiple cubes saw the light"
            
    return cubes_that_saw_the_light[0]


    

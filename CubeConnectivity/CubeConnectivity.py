#!/usr/bin/env python
import os
import sys
import numpy as np
if os.name!='nt':
    import rospy
    from rospy.numpy_msg import numpy_msg
    from std_msgs.msg import *

from Cube import *

state_channel = 'mblocks_light'
state_channel_type = numpy_msg(Int32MultiArray)
connection_channel = '/cubes/RunConnection'
connection_type = Bool
cubes={}
lasttime = 0
ambient_threshold = 1
state_sub = None

def listener():
    rospy.init_node('ConnectionManager')
    state_sub =  rospy.Subscriber(state_channel,state_channel_type,parseState)
    rospy.Subscriber(connection_channel,connection_type ,runConnections)
    rospy.spin()



def parseState(message_array):
    system_time = message_array[0]
    if system_time == lasttime: return
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
        if cube_id not in cubes.keys():
            cubes[cube_id] = Cube(cube_id)
        faceup = row[1]-1 #1-6 or 0-5
        sensors = row[2:]
        # is this the truth table i want or sensors>
        sensors = sensors<ambient_threshold
        cube = cubes[cube_id]
        cube.setMostRecentFaceUp(faceup)
        cube.setMostRecentLightSensorState(sensors)

    lasttime = system_time

def updateStates():
    system_state = rospy.waitForMessage(state_channel,state_channel_type)
    parseState(system_state)
    
def runConnections():
    state_sub.unregister()
    #turn off all LEDs
    for cube_id in cubes.keys():
        cubes[cube_id].setAllFaceLEDs(False)
    # get the state of the system with all lights off
    updateStates()

    # eliminate faces from testing while developing the base state
    for cube_id in cubes.keys():
        cube = cubes[cube_id]
        cur_state = cube.getMostRecentLightSensorState()
        cube.base_state = cur_state
        # remove the down face from testing, the face bellow it will light up
        faces = [x for x in range(0,6) if x != faceUpToDown(cube.faceUpHistory[0])]
        # remove the states that are already light up by the ambient light
        faces = [x for x in faces if not cur_state[x]]
        cube_faces_to_test[cube_id] = faces
        cube.unknown_faces = faces

    # Build connections without angles
    for cube_id in cubes.keys():
        cube = cubes[cube_id]
        faces = cube_faces_to_test[cube_id]
        for face in faces:
            cube.faceLEDs(face,True)
            ## pause for the signal to go through?
            updateStates()
            connection = DiffStates(cubes)[0]
            if len(connection)>0:
                cube.connectFace(face,connection)
                other_cube = cubes[connection[0]]
                other_cube.connectFace(connection[1],(cube_id,face))
                cube_faces_to_test[connection[0]] = [x for x in cube_faces_to_test[connection[0]] if x!= connection[1]]
    #We now have all of the connections
        
    # now we can orient the cubes
    #the unknowns are the connnected faces
    for cube_id in cubes.keys():
        cube = cubes[cube_id]
        cube.unknown_faces = cube.faceConnections.keys()

    #prune to get only 1/2 of each connected pair
    for cube_id in cubes.keys():
        cube = cubes[cube_id]
        for face in cube.unknown_faces:
            (other_id,other_face) = cube.faceConnections[face]
            other = cubes[other_id]
            other.unknown_faces = [x for x in other.unknown_faces if x!=other_face]
        
    #turn on each cubes light
    cmds = [ [1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    for led in range(0,4):
        cmd = cmds[led]
        angle = LEDToAngle[led]
        #turn on each remaining faces light
        for cube_id in cubes.keys():
            cube = cubes[cube_id]
            for face in cube.unknown_faces:
                cube.faceLEDs(face,cmd)
        #get the new state
        updateStates()
        #find who saw this LED
        cubes_that_saw_the_lights = DiffStates(cubes)
        for cube_face_pair in cubes_that_saw_the_lights:
            # the pair saw the connection, update the angle
            cube_id,face = cube_face_pair
            connected_cube = cubes[cube_id]
            connection = connected_cube.faceConnections[face]
            new_connection = (connection[0],connection[1],angle)
            connected_cube.faceConnections[face] = new_connection
                
            lit_cube = cubes[connection[0]]
            lit_connection = lit_cube.faceConnections[connection[1]]
            new_lit_connection = (lit_connection[0],lit_connection[1],angle)
            lit_cube.faceConnections[connection[1]] = new_lit_connection

    # now we have the correct connection list.
    for cube_id in cubes.keys():
        cube = cubes[cube_id]
        for face in range(0,6):
            print "cube:%s\tface:%i"%(cube_id,face)," :", cube.faceConnections[face]

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
            
    return cubes_that_saw_the_light

def LEDToAngle(i):
    vals = [0,90,180,270]
    return vals[i]

  

if __name__ == "__main__":
    listener()
#!/usr/bin/env python
"""--------------------------------------------------------------------
This Script creates an instance of a standalone client to receive, 
process and execute user commands for the Robotiq 2 finger adaptive grippers.

Parameters:
    comport: USB Communication port to which the gripper is connected to (not needed in `sim` mode).  
    baud: Baudrate of communication with gripper (not needed in `sim` mode).
    stroke: Maximum distance in meters, between the gripper fingers (Only 0,085 and 0.140 are currently supported)
    
@author: Matt Boyd
@email: mcboyd@mit.edu
--------------------------------------------------------------------"""

import time
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerStandaloneGripperDriver 


if __name__ == "__main__":

    # Set parameters
    comport = '/tmp/ttyUR'
    baud = 115200
    stroke = 0.05  # In meters
    
    # Create instance of standalone Robotiq Gripper Driver (automatically activates gripper, causing it to close and then open fully)
    gripper_driver = Robotiq2FingerStandaloneGripperDriver( comport=comport, baud=baud, stroke=stroke)

    while True:
        try:
            # Below assumes Python 2; for Python 3, change 'raw_input' to simply 'input'
            task = raw_input("Select function: c=close, o=open, q=quit\n")
            if(task == "c"): 
                gripper_driver.close()
            elif(task == "o"):
                gripper_driver.open()
            elif(task=="q"):
                break
            else:
                gripper_driver.goto(pos=0.03)
        except KeyboardInterrupt:
            #Statements to execute upon that exception
            print("You typed CTRL + C, which is the keyboard interrupt exception")
            break
    

#! /usr/bin/env python
"""--------------------------------------------------------------------
COPYRIGHT 2015 Stanley Innovation Inc.

Software License Agreement:

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 file   robotiq_2f_gripper_driver.py

 brief  Driver for Robotiq 85 communication

 Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""

"""
This Module defines the `Robotiq2FingerGripperDriver` and `Robotiq2FingerSimulatedGripperDriver` classes, which 
allow for the control and operation (also simulation of operation) of the 2 finger adaptive grippers from robotiq.

The end user should not need to use this class direcly since an instance of it is created with every action server
controlling a given gripper, and commanded by the user commands puubished by an action client instance.  
"""

from robotiq_2f_gripper import Robotiq2FingerGripper
from robotiq_2f_gripper_msgs.msg import RobotiqGripperCommand, RobotiqGripperStatus, CommandRobotiqGripperGoal, CommandRobotiqGripperFeedback, CommandRobotiqGripperResult
from sensor_msgs.msg import JointState
import numpy as np
import rospy
from enum import Enum
import time

WATCHDOG_TIME = 2.0   # Max Time without communication with gripper allowed
GOAL_DETECTION_THRESHOLD = 0.002 # Max deviation, in meters, from target goal to consider as goal "reached" 

class Robotiq2FingerGripperDriver:
    """
    This class represents an abstraction of a gripper driver, it handles the gripper connection, initialization,
    command send request, status request, and publishing of joint position to the `/joint_state` topic.   

    Args:
        comport: Name of the USB port to which the gripper is connected to.
        baud: Baudrate to use during Modbus protocol communication.
        stroke: Stroke of the gripper you are using, should be `0.085` or `0.140`.
        finger_joint: Name of the URDF joint to publish on the `/joint_state` topic. 

    Attributes:
        is_ready: Boolean indicating gripper is ready to take commands.
        _gripper: Instance of `robotiq_2f_gripper_control/Robotiq2FingerGripper` class representing the abstraction 
            of the real gripper.
        _gripper_joint_state_pub: Ros publisher for the `/joint_state` topic
        _driver_state: Internal machine state variable indicating the current gripper status
                            0: Driver not running
                            1: Driver is running
                            2: Gripper has been activated
    """
    def __init__(self, comport = '/dev/ttyUSB0', baud = '115200', stroke = 0.085, joint_name='finger_joint'):
        self._comport = comport
        self._baud = baud
        self._joint_name = joint_name          
        

        # Instanciate and open communication with gripper.
        self._gripper = Robotiq2FingerGripper(device_id=0, stroke=stroke, comport=self._comport, baud=self._baud)
        
        self._max_joint_limit = 0.08
        if( self._gripper.stroke == 0.140 ):
            self._max_joint_limit = 0.07
        # MCB added to account for Hand-e gripper
        elif( self._gripper.stroke == 0.05 ):
            self._max_joint_limit = 0.05
        # END MCB addition
        
        if not self._gripper.init_success:
            rospy.logerr("Unable to open commport to %s" % self._comport)
            return
        else:
            rospy.loginfo("Connection to gripper with stroke %.3f[m] on port %s successful" % ( self._gripper.stroke, self._comport))
            rospy.loginfo("Time: %.3f" % (rospy.get_time()))

        self._gripper_joint_state_pub = rospy.Publisher("/joint_states" , JointState, queue_size=10)        

        self._seq = 0
        self._prev_joint_pos = 0.0
        self._prev_joint_state_time = rospy.get_time() 
        self._driver_state = 0
        self.is_ready = False
        
        # MCB added looping structure below to accomodate cases where "getstatus" is a little slow
        status_check = 0
        while not self._gripper.getStatus():
            rospy.sleep(0.05)
            status_check += 1
            if status_check > 10:
                rospy.logerr("Failed to contact gripper on port %s ... ABORTING" % self._comport)
                return                
        # END MCB changes

        self._run_driver()
        self._last_update_time = rospy.get_time()
        
    def _clamp_position(self,cmd):
        out_of_bouds = False
        if (cmd < 0.0):
            out_of_bouds = True
            cmd_corrected = 0.0
        elif (cmd > self._gripper.stroke):
            out_of_bouds = True
            cmd_corrected = self._gripper.stroke
        if(out_of_bouds):
            rospy.logdebug("Position (%.3f[m]) out of limits for %d[mm] gripper: \n- New position: %.3f[m]\n- Min position: %.3f[m]\n- Max position: %.3f[m]" % (cmd, self._gripper.stroke*1000, cmd_corrected, 0.0, self._gripper.stroke))
            cmd = cmd_corrected
        return cmd

    def _clamp_speed(self,cmd):
        out_of_bouds = False
        if (cmd <= 0.013):
            out_of_bouds = True
            cmd_corrected = 0.013
        elif (cmd > 0.101):
            out_of_bouds = True
            cmd_corrected = 0.1
        if(out_of_bouds):
            rospy.logdebug("Speed (%.3f[m/s]) out of limits for %d[mm] gripper: \n- New speed: %.3f[m/s]\n- Min speed: %.3f[m/s]\n- Max speed: %.3f[m/s]" % (cmd, self._gripper.stroke*1000, cmd_corrected, 0.013, 0.1))
            cmd = cmd_corrected
        return cmd
    
    def _clamp_force(self,cmd):
        out_of_bouds = False
        if (cmd < 0.0):
            out_of_bouds = True
            cmd_corrected = 0.0
        elif (cmd > 100.0):
            out_of_bouds = True
            cmd_corrected = 100.0
        if(out_of_bouds):
            # rospy.logdebug("Force (%.3f[%]) out of limits for %d[mm] gripper: \n- New force: %.3f[%]\n- Min force: %.3f[%]\n- Max force: %.3f[%]" % (cmd, self._gripper.stroke*1000, cmd_corrected, 0, 100))
            cmd = cmd_corrected
        return cmd
    
    def update_gripper_command(self,cmd):
        """
        Updates the driver internal goal/setpoint, which will be use to command the robotiq gripper/ 

        Args:
            cmd: Instance of `robotiq_2f_gripper_msgs/RobotiqGripperCommand` message, holding the most recent 
            user provided command parameters. See the message declaration for fields description 
        """
        if (True == cmd.emergency_release):
            self._gripper.activate_emergency_release(open_gripper=cmd.emergency_release_dir)
            return
        else:
            self._gripper.deactivate_emergency_release()

        if (True == cmd.stop):
            self._gripper.stop()
            
        else:
            pos = self._clamp_position(cmd.position)
            vel = self._clamp_speed(cmd.speed)
            force = self._clamp_force(cmd.force)
            # rospy.loginfo(" %.3f %.3f %.3f " % (pos,vel,force))
            self._gripper.goto(pos=pos,vel=vel,force=force)
            
    def get_current_gripper_status(self, wait=False):
        """
        Public function to obtain the current gripper status.

        Args:
            wait: If wait is True, it waits until the gripper is not moving and then pulls the current status.
                  When False, the information can be several cycles old, but this can be sufficient in many circumstances.

        Returns:  Instance of `robotiq_2f_gripper_msgs/RobotiqGripperStatus` message. See the message declaration for fields description
        """
        # MCB added to get CURRENT gripper status:
        if wait:
            while not self._gripper.getStatus() or self._gripper.is_moving():
                rospy.sleep(0.01)
        # END MCB addition

        status = RobotiqGripperStatus()
        status.header.stamp = rospy.get_rostime()
        status.header.seq = self._seq
        status.is_ready = self._gripper.is_ready()
        status.is_reset = self._gripper.is_reset()
        status.is_moving = self._gripper.is_moving()
        status.obj_detected = self._gripper.object_detected()
        status.fault_status = self._gripper.get_fault_status()
        status.position = self._gripper.get_pos()
        status.requested_position = self._gripper.get_req_pos()
        status.current = self._gripper.get_current()
        return status
        
    def _update_gripper_joint_state(self):
        """
        Private helper function to create a JointState message with the current gripper joint position
        """
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = rospy.get_rostime()
        js.header.seq = self._seq
        js.name = [self._joint_name]
        max_joint_limit = self._max_joint_limit  # MCB added (and commented out below) to use class parameter rather than check each time!
        # max_joint_limit = 0.8
        # if( self._gripper.stroke == 0.140 ):
        #     max_joint_limit = 0.7
        
        pos = np.clip(max_joint_limit - ((max_joint_limit/self._gripper.stroke) * self._gripper.get_pos()), 0., max_joint_limit)
        
        js.position = [pos]
        dt = rospy.get_time() - self._prev_joint_state_time
        self._prev_joint_state_time = rospy.get_time()
        js.velocity = [(pos-self._prev_joint_pos)/dt]
        self._prev_joint_pos = pos
        return js
        
    def _run_driver(self):
        """
        Private function to test the comunication with the gripper and initialize it.
        WARNING: 
            Initialization of the gripper will generate a close motion of the real
            gripper. 

        Raises: 
            IOError: If Modbus RTU communication with the gripper is not achieved.
        """
        last_time = rospy.get_time()
        r = rospy.Rate(rospy.get_param('~rate', 100))
        while not rospy.is_shutdown() and self._driver_state != 2:
            # Check if communication is failing or taking too long
            dt = rospy.get_time() - last_time
            if (0 == self._driver_state):
                if (dt < 0.5):
                    self._gripper.deactivate_gripper()
                else:
                    self._driver_state = 1
            # If driver is not running, activate gripper 
            elif (1 == self._driver_state):
                is_gripper_activated = True
                self._gripper.activate_gripper()
                is_gripper_activated &= self._gripper.is_ready()
                if (is_gripper_activated):
                    rospy.loginfo("Gripper on port %s Activated" % self._comport)
                    self._driver_state = 2
                        
            success = True
            success &= self._gripper.sendCommand()
            success &= self._gripper.getStatus()
            if not success and not rospy.is_shutdown():
                rospy.logerr("Failed to initialize contact with gripper %d"% self._gripper.device_id)
            else:
                stat = RobotiqGripperStatus()
                #js = JointState()
                stat = self.get_current_gripper_status(True)
                js = self._update_gripper_joint_state()
                if stat.is_ready:
                    self.is_ready = True 
                # self._gripper_pub.publish(stat)
                # self._gripper_joint_state_pub.publish(js)
                            
            r.sleep()

    def update_driver(self):
        """
        Public function that:
            1. Sends the current driver command.
            2. Request the current gripper status.  
            3. Publish the gripper current joint position to the `/joint_state` topic.

        Raises:
            Log Error: If Modbus RTU communication with the gripper is not achieved.
        """
        # Try to update gripper command and status
        success = self._gripper.sendCommand()
        success &= self._gripper.getStatus()

        # Check if communication is broken
        update_time = rospy.get_time()
        if success:
            js = JointState()
            js = self._update_gripper_joint_state()
            self._gripper_joint_state_pub.publish(js)
            self._last_update_time = update_time
        
        # If communication failed, check if connection was truly lost
        elif (update_time - self._last_update_time) > WATCHDOG_TIME:
            print("Update time: {0}, Last time: {1}, Diff: {2}, Watchdog time: {3}".format(update_time, self._last_update_time, (update_time - self._last_update_time), WATCHDOG_TIME))
            rospy.logfatal("Failed to contact gripper on port: %s"% self._comport)
            # self._gripper.shutdown()
            # rospy.signal_shutdown("Communication to gripper lost")         
            

    def from_distance_to_radians(self, linear_pose ):
      """
      Private helper function to convert a command in meters to radians (joint value)
      """
      return np.clip(self._max_joint_limit - ((self._max_joint_limit/self._gripper.stroke) * linear_pose), 0.0, self._max_joint_limit)
  
    def from_radians_to_distance(self, joint_pose):
      """
      Private helper function to convert a joint position in radians to meters (distance between fingers)
      """
      # MCB added below return for Hand-E grippers only: to double the size of the incoming joint_pose (i.e., "2*joint_pose") because...
      #   RViz thinks it is commanding each of the two grippers to move an amount, but this driver is concerned with how far apart the grippers are.
      # So, for example, if we want the grippers to be 10mm apart, Rviz will tell both grippers to move 20mm towards each other...
      #   since 20mm movement by left gripper and 20mm movement by right gripper: 50mm stroke - 40mm total movement = 10mm apart.
      #   But this driver just wants to receive 10mm so it can move the grippers to be 10mm apart (hence the other mathematical gymnastics below).
      if self._gripper.stroke == 0.05:
        return np.clip(self._gripper.stroke - ((self._gripper.stroke/self._max_joint_limit) * (2*joint_pose)), 0.0, self._max_joint_limit)

      # Original return statement for non-Hand-E grippers below  
      return np.clip(self._gripper.stroke - ((self._gripper.stroke/self._max_joint_limit) * (joint_pose)), 0.0, self._max_joint_limit)
      
    def get_current_joint_position(self):
      return self.from_distance_to_radians(self._gripper.get_pos())
    ######################################################################################################
    # STATIC functions for fast control of the gripper.

    @staticmethod
    def goto( client, pos, speed=0.1, force=0, block = True ):
        """
        Static function to update the gripper command

        Args:
            client: `SimpleActionClient` instance connected to the action server holding a robotiq gripper
            instance.
            pos: [m] Position (distance in between fingers) in meters desired for the gripper.
            speed: [m/s] Motion speed in meters over seconds. Min value: 0.013[m/s] - Max value: 0.1[m/s]
            force: [%] Force value to apply in gripper motion, see robotiq manuals to calculate an appropiate
                    gripping force value in Newtons.  
            block: Boolean indicating whether to lock the current thread until command has been reached or not.
        """
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = pos
        goal.speed = speed
        goal.force = force

        # Sends the goal to the gripper.
        if block:   
            client.send_goal_and_wait(goal)
            client.wait_for_result()
        else:
            client.send_goal(goal)
    
    @staticmethod
    def close( client, speed=0.1, force=0, block = True,):
        """
        Static function to close the gripper 

        Args:
            client: `SimpleActionClient` instance connected to the action server holding a robotiq gripper
            instance.
            speed: [m/s] Motion speed in meters over seconds. Min value: 0.013[m/s] - Max value: 0.1[m/s]
            force: [%] Force value to apply in gripper motion, see robotiq manuals to calculate an appropiate
                    gripping force value in Newtons.  
            block: Boolean indicating whether to lock the current thread until command has been reached or not.
        """
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = 0.0
        goal.speed = speed
        goal.force = force

        # Sends the goal to the gripper.
        if block:
            client.send_goal_and_wait(goal)
        else:
            client.send_goal(goal)

    @staticmethod
    def open( client, speed=0.1, force=50, block = True):
        """
        Static function to open the gripper 

        Args:
            client: `SimpleActionClient` instance connected to the action server holding a robotiq gripper
            instance.
            speed: [m/s] Motion speed in meters over seconds. Min value: 0.013[m/s] - Max value: 0.1[m/s]
            force: [%] Force value to apply in gripper motion, see robotiq manuals to calculate an appropiate
                    gripping force value in Newtons.  
            block: Boolean indicating whether to lock the current thread until command has been reached or not.
        """
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = 255 # Use max value to make it stroke independent
        goal.speed = speed
        goal.force = force

        # Sends the goal to the gripper.
        if block:
            client.send_goal_and_wait(goal)
        else:
            client.send_goal(goal)

    @staticmethod
    def stop( client, block = True):
        """
        Static function to stop gripper motion

        Args:
            client: `SimpleActionClient` instance connected to the action server holding a robotiq gripper
            instance.
            block: Boolean indicating whether to lock the current thread until command has been reached or not.
        """
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False

        # Sends the goal to the gripper.
        if block:
            client.send_goal_and_wait(goal)
        else:
            client.send_goal(goal)

    @staticmethod
    def emergency_release( client ):
        """
        Static function to trigger an emergency release motion.

        Args:
            client: `SimpleActionClient` instance connected to the action server holding a robotiq gripper
            instance.
        """
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = True
        client.send_goal_and_wait(goal)

class Robotiq2FingerSimulatedGripperDriver:
    """
    This class represents an abstraction of a gripper driver for a SIMULATED gripper,
    by simulating the operation/response of the gripper to a given command and the publishing 
    of the simulated gripper joint position to the `/joint_state` topic.  

    Args:
        stroke: Stroke of the gripper you are using, should be `0.085` or `0.140`.
        finger_joint: Name of the URDF joint to publish on the `/joint_state` topic. 

    Attributes:
        is_ready: Boolean indicating gripper is ready to take commands.
        _gripper_joint_state_pub: Ros publisher for the `/joint_state` topic.
        _current_joint_pos: [radians] Position of the simulated joint in radians.
        _current_goal: Instance of `RobotiqGripperCommand` message holding the latest user command.
    """
    def __init__(self, stroke=0.085, joint_name='finger_joint'):
        self._stroke = stroke
        self._joint_name = joint_name
        self._current_joint_pos = 0.0                                 
        self._prev_time = rospy.get_time()
        self._current_goal = CommandRobotiqGripperGoal()
        self._current_goal.position = self._stroke
        self._gripper_joint_state_pub = rospy.Publisher("/joint_states" , JointState, queue_size=10)  
        self.is_ready = True
        self._is_moving = False
        self._max_joint_limit = 0.08
        if( self._stroke == 0.140 ):
            self._max_joint_limit = 0.07


    def update_driver(self):
        """
        Public function simulating the gripper state change given the current gripper command and the time
        difference between the last call to this function.
        """
        delta_time = rospy.get_time() - self._prev_time
        linear_position_goal = self._current_goal.position                                                   #[mm]
        joint_goal = self.from_distance_to_radians(linear_position_goal)                                     #[rad]
        position_increase = (delta_time * self._current_goal.speed) * (self._max_joint_limit/self._stroke) 
        if( abs(joint_goal - self._current_joint_pos) > position_increase ):
            self._current_joint_pos += position_increase if (joint_goal - self._current_joint_pos) > 0 else -position_increase
            self._is_moving = True
        else:
            self._current_joint_pos = joint_goal
            self._is_moving = False
        js = self._update_gripper_joint_state()
        self._gripper_joint_state_pub.publish(js)
        self._prev_time = rospy.get_time()


    def update_gripper_command(self, goal_command):
        """
        Updates the driver internal goal/setpoint, which will be use to command the robotiq gripper/ 

        Args:
            goal_command: Instance of `robotiq_2f_gripper_msgs/RobotiqGripperCommand` message, holding the most recent 
            user provided command parameters. See the message declaration for fields description 
        """
        self._current_goal = goal_command
        self._current_goal.position = self._clamp_position(goal_command.position)
        self._current_goal.speed = self._clamp_speed(goal_command.speed)

    def get_current_gripper_status(self):
        """
        Public function to obtain the current gripper status.

        Returns:  Instance of `robotiq_2f_gripper_msgs/RobotiqGripperStatus` message. See the message declaration for fields description
        """
        status = RobotiqGripperStatus()
        status.header.stamp = rospy.get_rostime()
        status.header.seq = 0
        status.is_ready = True
        status.is_reset = False
        status.is_moving = self._is_moving
        status.obj_detected = False
        status.fault_status = False
        status.position = self.from_radians_to_distance(self._current_joint_pos) #[mm]
        status.requested_position = self._current_goal.position                                                                                 #[mm]
        status.current = 0.0
        return status

    def get_current_joint_position(self):
      return self._current_joint_pos

    def from_distance_to_radians(self, linear_pose ):
      """
      Private helper function to convert a command in meters to radians (joint value)
      """
      return np.clip(self._max_joint_limit - ((self._max_joint_limit/self._stroke) * linear_pose), 0.0, self._max_joint_limit)
  
    def from_radians_to_distance(self, joint_pose):
      """
      Private helper function to convert a joint position in radians to meters (distance between fingers)
      """
      return np.clip(self._stroke - ((self._stroke/self._max_joint_limit) * joint_pose), 0.0, self._max_joint_limit)

    def _update_gripper_joint_state(self):
        """
        Private helper function to create a JointState message with the current gripper joint position
        """
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = rospy.get_rostime()
        js.header.seq = 0
        js.name = [self._joint_name]
        pos = np.clip( self._current_joint_pos, 0.0, self._max_joint_limit)
        js.position = [pos]
        js.velocity = [self._current_goal.speed]
        self._prev_joint_pos = pos
        return js

    def _clamp_position(self,cmd):
        if (cmd <= 0.0):
            cmd = 0.0
        elif (cmd >= self._stroke):
            cmd = self._stroke
        return cmd
    
    def _clamp_speed(self,cmd):
        if (cmd < 0.0):
            cmd = 0.01
        elif (cmd > 0.101):
            cmd = 0.1
        return cmd

# MCB Added Class
class Robotiq2FingerStandaloneGripperDriver:
    """
    This class represents an abstraction of a gripper driver, it handles the gripper connection, initialization,
    command send request, and status request. All ROS references have been removed so it can be invoked stand-alone.   

    Args:
        comport: Name of the USB port to which the gripper is connected to.
        baud: Baudrate to use during Modbus protocol communication.
        stroke: Stroke of the gripper you are using, should be `0.05`, `0.085`, or `0.140`.

    Attributes:
        is_ready: Boolean indicating gripper is ready to take commands.
        _gripper: Instance of `robotiq_2f_gripper_control/Robotiq2FingerGripper` class representing the abstraction 
            of the real gripper.
        _driver_state: Internal machine state variable indicating the current gripper status
                            0: Driver not running
                            1: Driver is running
                            2: Gripper has been activated
    """
    def __init__(self, comport = '/dev/ttyUSB0', baud = '115200', stroke = 0.05):
        self._comport = comport
        self._baud = baud      
        
        # Instanciate and open communication with gripper.
        self._gripper = Robotiq2FingerGripper(device_id=0, stroke=stroke, comport=self._comport, baud=self._baud)
        
        self._max_joint_limit = 0.08
        if( self._gripper.stroke == 0.140 ):
            self._max_joint_limit = 0.07
        elif( self._gripper.stroke == 0.05 ):
            self._max_joint_limit = 0.05
        
        if not self._gripper.init_success:
            print("Unable to open commport to %s" % self._comport)
            return
        else:
            print("Connection to gripper with stroke %.3f[m] on port %s successful" % ( self._gripper.stroke, self._comport))

        self._seq = 0
        self._driver_state = 0
        self.is_ready = False
        
        status_check = 0
        while not self._gripper.getStatus():
            time.sleep(0.05)
            status_check += 1
            if status_check > 10:
                print("Failed to contact gripper on port %s ... ABORTING" % self._comport)
                return                

        self._run_driver()
        self._last_update_time = time.time()  # epoch seconds
        
    def _clamp_position(self,cmd):
        out_of_bouds = False
        if (cmd < 0.0):
            out_of_bouds = True
            cmd_corrected = 0.0
        elif (cmd > self._gripper.stroke):
            out_of_bouds = True
            cmd_corrected = self._gripper.stroke
        if(out_of_bouds):
            print("Position (%.3f[m]) out of limits for %d[mm] gripper: \n- New position: %.3f[m]\n- Min position: %.3f[m]\n- Max position: %.3f[m]" % (cmd, self._gripper.stroke*1000, cmd_corrected, 0.0, self._gripper.stroke))
            cmd = cmd_corrected
        return cmd

    def _clamp_speed(self,cmd):
        out_of_bouds = False
        if (cmd <= 0.013):
            out_of_bouds = True
            cmd_corrected = 0.013
        elif (cmd > 0.101):
            out_of_bouds = True
            cmd_corrected = 0.1
        if(out_of_bouds):
            print("Speed (%.3f[m/s]) out of limits for %d[mm] gripper: \n- New speed: %.3f[m/s]\n- Min speed: %.3f[m/s]\n- Max speed: %.3f[m/s]" % (cmd, self._gripper.stroke*1000, cmd_corrected, 0.013, 0.1))
            cmd = cmd_corrected
        return cmd
    
    def _clamp_force(self,cmd):
        out_of_bouds = False
        if (cmd < 0.0):
            out_of_bouds = True
            cmd_corrected = 0.0
        elif (cmd > 100.0):
            out_of_bouds = True
            cmd_corrected = 100.0
        if(out_of_bouds):
            # rospy.logdebug("Force (%.3f[%]) out of limits for %d[mm] gripper: \n- New force: %.3f[%]\n- Min force: %.3f[%]\n- Max force: %.3f[%]" % (cmd, self._gripper.stroke*1000, cmd_corrected, 0, 100))
            cmd = cmd_corrected
        return cmd
    
    def update_gripper_command(self,cmd):
        """
        Updates the driver internal goal/setpoint, which will be use to command the robotiq gripper/ 

        Args:
            cmd: Instance of `robotiq_2f_gripper_msgs/RobotiqGripperCommand` message, holding the most recent 
            user provided command parameters. See the message declaration for fields description 
        """
        if (True == cmd.emergency_release):
            self._gripper.activate_emergency_release(open_gripper=cmd.emergency_release_dir)
            return
        else:
            self._gripper.deactivate_emergency_release()

        if (True == cmd.stop):
            self._gripper.stop()
            
        else:
            pos = self._clamp_position(cmd.position)
            vel = self._clamp_speed(cmd.speed)
            force = self._clamp_force(cmd.force)
            # print("update_gripper_command: %.3f %.3f %.3f " % (pos,vel,force))
            self._gripper.goto(pos=pos,vel=vel,force=force)
            
    def get_current_gripper_status(self, wait=False):
        """
        Public function to obtain the current gripper status.

        Args:
            wait: If wait is True, it waits until the gripper is not moving and then pulls the current status.
                  When False, the information can be several cycles old, but this can be sufficient in many circumstances.

        Returns:  Instance of `robotiq_2f_gripper_msgs/RobotiqGripperStatus` message. See the message declaration for fields description
        """
        # Get CURRENT gripper status:
        if wait:
            while not self._gripper.getStatus() or self._gripper.is_moving():
                time.sleep(0.01)
        
        status = RobotiqGripperStatus()
        status.header.stamp = time.time()  # epoch seconds
        status.header.seq = self._seq
        status.is_ready = self._gripper.is_ready()
        status.is_reset = self._gripper.is_reset()
        status.is_moving = self._gripper.is_moving()
        status.obj_detected = self._gripper.object_detected()
        status.fault_status = self._gripper.get_fault_status()
        status.position = self._gripper.get_pos()
        status.requested_position = self._gripper.get_req_pos()
        status.current = self._gripper.get_current()
        return status
        
    def _run_driver(self):
        """
        Private function to test the comunication with the gripper and initialize it.
        WARNING: 
            Initialization of the gripper will generate a close motion of the real
            gripper. 

        Raises: 
            IOError: If Modbus RTU communication with the gripper is not achieved.
        """
        last_time = time.time()  # epoch seconds
        while self._driver_state != 2:
            # Check if communication is failing or taking too long
            dt = time.time() - last_time
            if (0 == self._driver_state):
                if (dt < 0.5):
                    self._gripper.deactivate_gripper()
                else:
                    self._driver_state = 1
            # If driver is not running, activate gripper 
            elif (1 == self._driver_state):
                is_gripper_activated = True
                self._gripper.activate_gripper()
                is_gripper_activated &= self._gripper.is_ready()
                if (is_gripper_activated):
                    print("Gripper on port %s Activated" % self._comport)
                    self._driver_state = 2
                        
            success = True
            success &= self._gripper.sendCommand()
            success &= self._gripper.getStatus()
            if not success:
                print("Failed to initialize contact with gripper %d"% self._gripper.device_id)
            else:
                stat = RobotiqGripperStatus()
                stat = self.get_current_gripper_status(True)
                if stat.is_ready:
                    self.is_ready = True 
                            
            time.sleep(0.02)

    def update_driver(self):
        """
        Public function that:
            1. Sends the current driver command.
            2. Request the current gripper status.  

        Raises:
            Log Error: If Modbus RTU communication with the gripper is not achieved.
        """
        # Try to update gripper command and status
        success = self._gripper.sendCommand()
        success &= self._gripper.getStatus()

        # Check if communication is broken
        update_time = time.time()  # epoch seconds
        if success:
            self._last_update_time = update_time
        
        # If communication failed, check if connection was truly lost
        elif (update_time - self._last_update_time) > WATCHDOG_TIME:
            print("Update time: {0}, Last time: {1}, Diff: {2}, Watchdog timeout: {3}".format(update_time, self._last_update_time, (update_time - self._last_update_time), WATCHDOG_TIME))
            print("Failed to contact gripper on port: %s"% self._comport)      
        else:
            print("update_driver NOT successful")
            

    def from_distance_to_radians(self, linear_pose ):
      """
      Private helper function to convert a command in meters to radians (joint value)
      """
      return np.clip(self._max_joint_limit - ((self._max_joint_limit/self._gripper.stroke) * linear_pose), 0.0, self._max_joint_limit)
  
    def from_radians_to_distance(self, joint_pose):
      """
      Private helper function to convert a joint position in radians to meters (distance between fingers)
      """
      # MCB added below return for Hand-E grippers only: to double the size of the incoming joint_pose (i.e., "2*joint_pose") because...
      #   RViz thinks it is commanding each of the two grippers to move an amount, but this driver is concerned with how far apart the grippers are.
      # So, for example, if we want the grippers to be 10mm apart, Rviz will tell both grippers to move 20mm towards each other...
      #   since 20mm movement by left gripper and 20mm movement by right gripper: 50mm stroke - 40mm total movement = 10mm apart.
      #   But this driver just wants to receive 10mm so it can move the grippers to be 10mm apart (hence the other mathematical gymnastics below).
      if self._gripper.stroke == 0.05:
        return np.clip(self._gripper.stroke - ((self._gripper.stroke/self._max_joint_limit) * (2*joint_pose)), 0.0, self._max_joint_limit)

      # Original return statement for non-Hand-E grippers below  
      return np.clip(self._gripper.stroke - ((self._gripper.stroke/self._max_joint_limit) * (joint_pose)), 0.0, self._max_joint_limit)
      
    def get_current_joint_position(self):
      return self.from_distance_to_radians(self._gripper.get_pos())


    def send_goal(self, goal_command):
      # Send incoming command to gripper driver
      self.update_gripper_command(goal_command)
      # Wait until command is received by the gripper 
      time.sleep(0.1)
      self._processing_goal = True        
      self._is_stalled = False
      
      feedback = CommandRobotiqGripperFeedback()
      result = CommandRobotiqGripperResult()

      while self._processing_goal and not self._is_stalled:             # While moving and not stalled provide feedback and check for result
          self.update_driver()
          feedback = self.get_current_gripper_status()
          # print("Error = %.5f Requested position = %.3f Current position = %.3f" % (abs(feedback.requested_position - feedback.position), feedback.requested_position, feedback.position))
          # Check for completition of action 
          if( feedback.fault_status != 0 and not self._is_stalled):               # Check for errors
              # print(self._action_name + ": fault status (gFLT) is: %d", feedback.fault_status)
              self._is_stalled = True
              # self._action_server.set_aborted( feedback , (self._action_name + ": fault status (gFLT) is: %d" % feedback.fault_status))
              break
          if( abs(feedback.requested_position - feedback.position) < GOAL_DETECTION_THRESHOLD or feedback.obj_detected):    # Check if position has been reached 
              self._processing_goal = False 
              self._is_stalled = False             
              # print("hopefully stopping because object detected: ")
              # print(feedback)
          time.sleep(0.02)

      result = self.get_current_gripper_status(True)
      
      # Send result 
      if not self._is_stalled:
          print("done sending goal, goal reached or object detected Pos: %.3f PosRequested: %.3f ObjectDetected: %r" % (goal_command.position, feedback.requested_position, feedback.obj_detected))  
      else:
          print("done sending goal, goal aborted Pos: %.3f PosRequested: %.3f ObjectDetected: %r" % (goal_command.position, feedback.requested_position, feedback.obj_detected) )

      self._processing_goal = False 
      self._is_stalled = False 

    ######################################################################################################
    # Functions for fast control of the gripper.

    def goto(self, pos, speed=0.1, force=120):
        """
        Function to update the gripper command; blocks further execution until complete

        Args:
            pos: [m] Position (distance in between fingers) in meters desired for the gripper.
            speed: [m/s] Motion speed in meters over seconds. Min value: 0.013[m/s] - Max value: 0.1[m/s]
            force: [%] Force value to apply in gripper motion, see robotiq manuals to calculate an appropiate
                    gripping force value in Newtons.  
        """
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = pos
        goal.speed = speed
        goal.force = force

        # Sends the goal to the gripper.
        self.send_goal(goal)
    
    def close(self):
        """
        Function to close the gripper; blocks further execution until complete

        Useful inputs:
            speed: [m/s] Motion speed in meters over seconds. Min value: 0.013[m/s] - Max value: 0.1[m/s]
            force: [%] Force value to apply in gripper motion, see robotiq manuals to calculate an appropiate
                    gripping force value in Newtons.  
        """
        speed=0.1
        force=0
        
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = 0.0
        goal.speed = speed
        goal.force = force

        # Sends the goal to the gripper.
        self.send_goal(goal)

    def open(self):
        """
        Function to open the gripper; blocks further execution until complete 

        Useful inputs:
            speed: [m/s] Motion speed in meters over seconds. Min value: 0.013[m/s] - Max value: 0.1[m/s]
            force: [%] Force value to apply in gripper motion, see robotiq manuals to calculate an appropiate
                    gripping force value in Newtons.  
        """
        speed=0.1
        force=50

        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = self._gripper.stroke  # Value is in meters
        goal.speed = speed
        goal.force = force

        # Sends the goal to the gripper.
        self.send_goal(goal)

    def stop(self):
        """
        Function to stop gripper motion; blocks further execution until complete 
        """
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False

        # Sends the goal to the gripper.
        self.send_goal(goal)

    def emergency_release(self):
        """
        Function to trigger an emergency release motion; blocks further execution until complete 
        """
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = True
        self.send_goal(goal)
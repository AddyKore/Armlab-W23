"""!
The state machine that implements the logic.
"""
from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
import time
import numpy as np
import rospy 

class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
    """
    #Variables for Teach and Playback functionality
    Teach = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])    # teach position array
    flag = 0
    grip_pos = 0
    clicked = 0

    #Variables for Camera calibration
    aprilTagData = np.array([])
    aprilTagPoints = np.array([[-250, -25, 0], [250, -25, 0], [-250, 250, 0], [250, 250, 0]])

    def __init__(self, rxarm, camera):
        """!
        @brief      Constructs a new instance.

        @param      rxarm   The rxarm
        @param      planner  The planner
        @param      camera   The camera
        """
        self.rxarm = rxarm
        self.camera = camera
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.waypoints = [
            [-np.pi/2,       -0.5,      -0.3,            0.0,       0.0],
            [0.75*-np.pi/2,   0.5,      0.3,      0.0,       np.pi/2],
            [0.5*-np.pi/2,   -0.5,     -0.3,     np.pi / 2,     0.0],
            [0.25*-np.pi/2,   0.5,     0.3,     0.0,       np.pi/2],
            [0.0,             0.0,      0.0,         0.0,     0.0],
            [0.25*np.pi/2,   -0.5,      -0.3,      0.0,       np.pi/2],
            [0.5*np.pi/2,     0.5,     0.3,     np.pi / 2,     0.0],
            [0.75*np.pi/2,   -0.5,     -0.3,     0.0,       np.pi/2],
            [np.pi/2,         0.5,     0.3,      0.0,     0.0],
            [0.0,             0.0,     0.0,      0.0,     0.0]]

    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

            This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    TODO: Add states and funcitons as needed.
        """
        if self.next_state == "initialize_rxarm":
            self.initialize_rxarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()

        if self.next_state == "detect":
            self.detect()

        if self.next_state == "manual":
            self.manual()

        if self.next_state == "Record":
            self.Record()

        if self.next_state == "Play":
            self.Play()

        if self.next_state == "grip_open":
            self.grip_open()

        if self.next_state == "grip_close":
            self.grip_close()
        
        if self.next_state == "place":
            self.PickPlace()

        # addy added them  
        if self.next_state == "PNPSP":
            self.PNPSP()
        if self.next_state == "PNPDP":
            self.PNPDP()
        if self.next_state == "PNPL":
            self.PNPL()
        if self.next_state == "Sort":
            self.Sort()
        if self.next_state == "SStack":
            self.SStack()
        if self.next_state == "SAStack":
            self.SAStack()
        #addy part done
    """Functions run for each state"""

    def manual(self):
        """!
        @brief      Manually control the rxarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check rxarm and restart program"
        self.current_state = "estop"
        self.rxarm.disable_torque()

    def execute(self):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        self.status_message = "State: Execute - Executing motion plan"
        self.current_state = "execute"
        for i in range(0,9):
           self.rxarm.set_positions(self.waypoints[i])
           rospy.sleep(2)      

        self.next_state = "idle"

    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"
        self.next_state = "idle"

        """TODO Perform camera calibration routine here"""
        print("Calibration")
        
        self.aprilTagData = self.camera.aprilTagDetect()
        print(self.aprilTagData)
        '''
        translation = self.aprilTagPoints - self.aprilTagData
        print(translation)
        '''
        self.status_message = "Calibration - Completed Calibration"

    """ TODO """
    def detect(self):
        """!
        @brief      Detect the blocks
        """
        self.current_state = "detect"
        self.next_state = "idle"
        print("Detection")        
        self.camera.blockDetector()
        self.status_message = "Block detection complete"
        rospy.sleep(1)

    def initialize_rxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.current_state = "initialize_rxarm"
        self.status_message = "RXArm Initialized!"
        if not self.rxarm.initialize():
            print('Failed to initialize the rxarm')
            self.status_message = "State: Failed to initialize the rxarm!"
            rospy.sleep(5)
        self.next_state = "idle"

    def Record(self):
        self.status_message = "State: Record - recorded current motor positions"
        self.current_state = "Record"
        posit = self.rxarm.get_positions()
        posit = np.append(posit, [self.grip_pos])
        print(posit)
        print(len(self.Teach))
        if self.flag == 0:
                self.Teach[0] = posit
                self.flag = 1
                print(self.Teach[0])
        else:
            self.Teach = np.append(self.Teach, [posit], axis=0)
            print(len(self.Teach))
            print(self.Teach)

        self.next_state = "idle"

    def Play(self):
        self.status_message = "State: Play - Executing recorded motion plan"
        self.current_state = "Play"

        print(len(self.Teach))
        for i in self.Teach:
            posit = i[0:4:1]
            gripper = i[5]
            print(self.Teach)
            self.rxarm.set_positions(posit)
            print(self.rxarm.get_positions())

            if gripper == 0:
                self.rxarm.open_gripper()
            else:
                self.rxarm.close_gripper()
            
            print(i)
            rospy.sleep(2)      

        self.next_state = "idle"

    def grip_open(self):
        self.status_message = "State: Opening - Opening the Gripper"
        self.current_state = "grip_open"
        self.grip_pos=0
        self.next_state = "idle"

    def grip_close(self):
        self.status_message = "State: Closing - Closing the Gripper"
        self.current_state = "grip_close"
        self.grip_pos=1
        self.next_state = "idle"

    def PickPlace(self):
        self.status_message = "State: Pick and Place - Click to grab and place a block"
        self.current_state = "place"
        
        if (self.clicked == 2):
            self.clicked = 0
            self.next_state = "idle"
        else:
            self.next_state = "place"
    # addy added them 
    def PNPSP(self):
        self.status_message = "State: Pick and Place in place - Automatic code"
        self.current_state = "PNPSP"
    def PNPDP(self):
        self.status_message = "State: Pick and Place in different place - Automatic code"
        self.current_state = "PNPDP"
    def PNPL(self):
        self.status_message = "State: Pick and Place in different place - Automatic code"
        self.current_state = "PNPL"
    def Sort(self):
        self.status_message = "State: Pick and Place in different place - Automatic code"
        self.current_state = "Sort"
    def SStack(self):
        self.status_message = "State: Picks and stacks size wise - Automatic code"
        self.current_state = "SStack"
    def SAStack(self):
        self.status_message = "State: Pick and Place in different place - Automatic code"
        self.current_state = "SAStack"
        

class StateMachineThread(QThread):
    """!
    @brief      Runs the state machine
    """
    updateStatusMessage = pyqtSignal(str)
    
    def __init__(self, state_machine, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      state_machine  The state machine
        @param      parent         The parent
        """
        QThread.__init__(self, parent=parent)
        self.sm=state_machine

    def run(self):
        """!
        @brief      Update the state machine at a set rate
        """
        while True:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            rospy.sleep(0.05)

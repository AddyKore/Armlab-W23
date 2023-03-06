#!/usr/bin/python
"""!
Main GUI for Arm lab
"""
import os
script_path = os.path.dirname(os.path.realpath(__file__))

import argparse
import sys
import cv2
import numpy as np
import rospy
import time
from functools import partial

from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel,
                         QMainWindow, QCursor, QFileDialog)

from ui import Ui_MainWindow
from rxarm import RXArm, RXArmThread
from camera import Camera, VideoThread
from state_machine import StateMachine, StateMachineThread
""" Radians to/from  Degrees conversions """
D2R = np.pi / 180.0
R2D = 180.0 / np.pi
Teach = np.array([])    # teach position array


class Gui(QMainWindow):
    """
    Main GUI Class

    Contains the main function and interfaces between the GUI and functions.
    """
    def __init__(self, parent=None, dh_config_file=None):
        QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        """ Groups of ui commonents """
        self.joint_readouts = [
            self.ui.rdoutBaseJC,
            self.ui.rdoutShoulderJC,
            self.ui.rdoutElbowJC,
            self.ui.rdoutWristAJC,
            self.ui.rdoutWristRJC,
        ]
        self.joint_slider_rdouts = [
            self.ui.rdoutBase,
            self.ui.rdoutShoulder,
            self.ui.rdoutElbow,
            self.ui.rdoutWristA,
            self.ui.rdoutWristR,
        ]
        self.joint_sliders = [
            self.ui.sldrBase,
            self.ui.sldrShoulder,
            self.ui.sldrElbow,
            self.ui.sldrWristA,
            self.ui.sldrWristR,
        ]
        """Objects Using Other Classes"""
        self.camera = Camera()
        print("Creating rx arm...")
        if (dh_config_file is not None):
            self.rxarm = RXArm(dh_config_file=dh_config_file)
        else:
            self.rxarm = RXArm()
        print("Done creating rx arm instance.")
        self.sm = StateMachine(self.rxarm, self.camera)
        """
        Attach Functions to Buttons & Sliders
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        # Video
        self.ui.videoDisplay.setMouseTracking(True)
        self.ui.videoDisplay.mouseMoveEvent = self.trackMouse
        self.ui.videoDisplay.mousePressEvent = self.calibrateMousePress

        # Buttons
        # Handy lambda function falsethat can be used with Partial to only set the new state if the rxarm is initialized
        #nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state if self.rxarm.initialized else None)
        nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state)
        self.ui.btn_estop.clicked.connect(self.estop)
        self.ui.btn_init_arm.clicked.connect(self.initRxarm)
        self.ui.btn_torq_off.clicked.connect(
            lambda: self.rxarm.disable_torque())
        self.ui.btn_torq_on.clicked.connect(lambda: self.rxarm.enable_torque())
        self.ui.btn_sleep_arm.clicked.connect(lambda: self.rxarm.sleep())

        #User Buttons
        self.ui.btnUser1.setText("Calibrate")
        self.ui.btnUser1.clicked.connect(partial(nxt_if_arm_init, 'calibrate'))
        self.ui.btnUser2.setText('Open Gripper')
        self.ui.btnUser2.clicked.connect(lambda: self.rxarm.open_gripper())
        self.ui.btnUser3.setText('Close Gripper')
        self.ui.btnUser3.clicked.connect(lambda: self.rxarm.close_gripper())
        self.ui.btnUser4.setText('Execute')
        self.ui.btnUser4.clicked.connect(partial(nxt_if_arm_init, 'execute'))
        self.ui.btnUser5.setText('Detect Blocks')
        self.ui.btnUser5.clicked.connect(partial(nxt_if_arm_init, 'detect'))
        self.ui.btnUser6.setText('Click to Place')
        self.ui.btnUser6.clicked.connect(partial(nxt_if_arm_init, 'place'))

        # addy added them
        self.ui.btnUser7.setText('E4')
        self.ui.btnUser7.clicked.connect(partial(nxt_if_arm_init, 'PNPSP'))  #Pick and place in the same place
        self.ui.btnUser8.setText('PicknPlace DXM')
        self.ui.btnUser8.clicked.connect(partial(nxt_if_arm_init, 'PNPDP'))  #pick and place in a different place but same for each block
        self.ui.btnUser9.setText('E3- Line')
        self.ui.btnUser9.clicked.connect(partial(nxt_if_arm_init, 'PNPL'))   #Pick and place blocks in a line
        self.ui.btnUser10.setText('Sort Color')
        self.ui.btnUser10.clicked.connect(partial(nxt_if_arm_init, 'Sort'))  #sorts different colors
        self.ui.btnUser11.setText('E1 - Sort Size')
        self.ui.btnUser11.clicked.connect(partial(nxt_if_arm_init, 'SStack'))  #sort and stack all same color points in same point
        self.ui.btnUser12.setText('Sort a Stack')
        self.ui.btnUser12.clicked.connect(partial(nxt_if_arm_init, 'SAStack'))   #sort a stack and stack all same color points in same point
        #addy part done

        self.ui.btn_task1.setText('Record')
        self.ui.btn_task1.clicked.connect(partial(nxt_if_arm_init, 'Record'))
        self.ui.btn_task2.setText('Play')
        self.ui.btn_task2.clicked.connect(partial(nxt_if_arm_init, 'Play'))
        self.ui.btn_task3.setText('Make Open')
        self.ui.btn_task3.clicked.connect(partial(nxt_if_arm_init, 'grip_open'))
        self.ui.btn_task4.setText('Make Close')
        self.ui.btn_task4.clicked.connect(partial(nxt_if_arm_init, 'grip_close'))
        

        # Sliders
        for sldr in self.joint_sliders:
            sldr.valueChanged.connect(self.sliderChange)
        self.ui.sldrMoveTime.valueChanged.connect(self.sliderChange)
        self.ui.sldrAccelTime.valueChanged.connect(self.sliderChange)
        # Direct Control
        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
        # Status
        self.ui.rdoutStatus.setText("Waiting for input")
        """initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)
        """Setup Threads"""

        # State machine
        self.StateMachineThread = StateMachineThread(self.sm)
        self.StateMachineThread.updateStatusMessage.connect(
            self.updateStatusMessage)
        self.StateMachineThread.start()
        self.VideoThread = VideoThread(self.camera)
        self.VideoThread.updateFrame.connect(self.setImage)
        self.VideoThread.start()
        self.ArmThread = RXArmThread(self.rxarm)
        self.ArmThread.updateJointReadout.connect(self.updateJointReadout)
        self.ArmThread.updateEndEffectorReadout.connect(
            self.updateEndEffectorReadout)
        self.ArmThread.start()

    """ Slots attach callback functions to signals emitted from threads"""

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)

    @pyqtSlot(list)
    def updateJointReadout(self, joints):
        for rdout, joint in zip(self.joint_readouts, joints):
            rdout.setText(str('%+.2f' % (joint * R2D)))

    ### TODO: output the rest of the orientation according to the convention chosen
    @pyqtSlot(list)
    def updateEndEffectorReadout(self, pos):
        self.ui.rdoutX.setText(str("%+.2f mm" % (1000 * pos[0])))
        self.ui.rdoutY.setText(str("%+.2f mm" % (1000 * pos[1])))
        self.ui.rdoutZ.setText(str("%+.2f mm" % (1000 * pos[2])))
        #self.ui.rdoutPhi.setText(str("%+.2f rad" % (pos[3])))
        #self.ui.rdoutTheta.setText(str("%+.2f" % (pos[4])))
        #self.ui.rdoutPsi.setText(str("%+.2f" % (pos[5])))

    @pyqtSlot(QImage, QImage, QImage, QImage)
    def setImage(self, rgb_image, depth_image, tag_image, point_image):
        """!
        @brief      Display the images from the camera.

        @param      rgb_image    The rgb image
        @param      depth_image  The depth image
        """
        
        if (self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(rgb_image))
        if (self.ui.radioDepth.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_image))
        if (self.ui.radioUsr1.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(tag_image))
        if (self.ui.radioUsr2.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(point_image))

    """ Other callback functions attached to GUI elements"""

    def estop(self):
        self.rxarm.disable_torque()
        self.sm.set_next_state('estop')

    def sliderChange(self):
        """!
        @brief Slider changed

        Function to change the slider labels when sliders are moved and to command the arm to the given position
        """
        for rdout, sldr in zip(self.joint_slider_rdouts, self.joint_sliders):
            rdout.setText(str(sldr.value()))

        self.ui.rdoutMoveTime.setText(
            str(self.ui.sldrMoveTime.value() / 10.0) + "s")
        self.ui.rdoutAccelTime.setText(
            str(self.ui.sldrAccelTime.value() / 20.0) + "s")
        self.rxarm.set_moving_time(self.ui.sldrMoveTime.value() / 10.0)
        self.rxarm.set_accel_time(self.ui.sldrAccelTime.value() / 20.0)

        # Do nothing if the rxarm is not initialized
        if self.rxarm.initialized:
            joint_positions = np.array(
                [sldr.value() * D2R for sldr in self.joint_sliders])
            # Only send the joints that the rxarm has
            self.rxarm.set_positions(joint_positions[0:self.rxarm.num_joints])

    def directControlChk(self, state):
        """!
        @brief      Changes to direct control mode

                    Will only work if the rxarm is initialized.

        @param      state  State of the checkbox
        """
        if state == Qt.Checked and self.rxarm.initialized:
            # Go to manual and enable sliders
            self.sm.set_next_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        else:
            # Lock sliders and go to idle
            self.sm.set_next_state("idle")
            self.ui.SliderFrame.setEnabled(False)
            self.ui.chk_directcontrol.setChecked(False)
    
    def trackMouse(self, mouse_event):
        """!
        @brief      Show the mouse position in GUI

                    TODO: after implementing workspace calibration display the world coordinates the mouse points to in the RGB
                    video image.

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """

        pt = mouse_event.pos()
        if self.camera.DepthFrameRaw.any() != 0:
            z = self.camera.DepthFrameRaw[pt.y()][pt.x()]
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" %
                                             (pt.x(), pt.y(), z))                            
            
            InverseIntrinsic=np.linalg.inv(self.camera.intrinsic_matrix)
            InverseExtrinsic=np.linalg.inv(self.camera.extrinsic_matrix)
            
            temp = np.array([[pt.x()],[pt.y()],[1]])
            camera_cord = np.matmul(InverseIntrinsic,(temp*z))

            self.world_cord = np.matmul(InverseExtrinsic,[camera_cord[0],camera_cord[1],camera_cord[2],1])
            self.ui.rdoutMouseWorld.setText(str(int(self.world_cord[0]))+','+str(int(self.world_cord[1]))+','+(str(int(self.world_cord[2]))))
            
    def pickUpBlock(self, coords):
        contours = self.camera.contours
        index = 0
        #print(contours)
        for shape in contours:
            location = cv2.pointPolygonTest(shape, (coords[0], coords[1]), False)
            print(shape[0])
            print(self.camera.conts[index])
            print(location)
            if ((location >= 0)  and (self.camera.conts[index] == 'square')):
                length = np.sqrt(cv2.contourArea(shape))/2.0
                print(length)
                return length
            index += 1
        return 0.0
    
    def calibrateMousePress(self, mouse_event):
        """!
        @brief Record mouse click positions for calibration

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """
        """ Get mouse posiiton """
        pt = mouse_event.pos()
        self.camera.last_click[0] = pt.x()
        self.camera.last_click[1] = pt.y()
        self.camera.new_click = True
        # print(self.camera.last_click)
        if(self.sm.current_state == "place"):
            #print("Click!")
            #print(self.world_cord)
            #Go to home to avoid hitting anything (Rudimentary object avoidance to prevent damage to arm)
            camCoord = np.array([pt.x(), pt.y()])
            worldCoord = self.world_cord[:3]
            if(self.sm.clicked == 0):
                self.rxarm.open_gripper()
                offset = self.pickUpBlock(camCoord)
                worldCoord[2] -= 20
            else:
                offset = self.pickUpBlock(camCoord)
                worldCoord[2] += 20

            print(worldCoord)
            Pose = self.rxarm.get_IK_joint_pose((worldCoord/1000.0))
            #print(Pose)
            intWorldCord=np.array([worldCoord[0],worldCoord[1],(worldCoord[2]+70)])
            intPose = self.rxarm.get_IK_joint_pose((intWorldCord/1000.0))
                    

            intjoint_angles = np.zeros(5)
            intnum_angles = 4
            intjoint_angles = np.append(intPose[0, 0:intnum_angles], intjoint_angles[intnum_angles:intjoint_angles.shape[0]])
            print(intjoint_angles)
            
            joint_angles = np.zeros(5)
            num_angles = 4
            joint_angles = np.append(Pose[0, 0:num_angles], joint_angles[num_angles:joint_angles.shape[0]])
            #print(joint_angles)

            self.moving_time = 1.5
            self.accel_time = 0.5

            #Motion sequence: Go to home position, go to defined position, return to home
            self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                             accel_time=self.accel_time,
                             blocking=False)
            
            self.rxarm.set_positions(intjoint_angles)
            rospy.sleep(1)
            
            self.rxarm.set_positions(joint_angles)
            rospy.sleep(1)
            if (self.sm.clicked== 1):
                self.rxarm.open_gripper()
            else:
                self.rxarm.close_gripper()
            rospy.sleep(1)
            
            self.rxarm.set_positions(intjoint_angles)
            rospy.sleep(1)
            

            self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                             accel_time=self.accel_time,
                             blocking=False)
            
            self.sm.clicked += 1
        # addy added them     
        if (self.sm.current_state == "PNPSP"):

            self.camera.blockDetector()
                    
                

        if (self.sm.current_state == "PNPDP"):
           
            self.camera.blockDetector()
            for i in range(0,len(self.camera.contours)):
                if self.camera.contc[i]!='VIBGYOR':
                    worldCord= np.array([self.camera.wx[i],self.camera.wy[i],self.camera.wz[i]])
                    #worldCord= np.array([300,325,150])
                    Pose = self.rxarm.get_IK_joint_pose((worldCord/1000.0))
                    intWorldCord=np.array([worldCord[0],worldCord[1],(worldCord[2]+70)])
                    intPose = self.rxarm.get_IK_joint_pose((intWorldCord/1000.0))
                    Pose = self.rxarm.get_IK_joint_pose((worldCord/1000.0))
                    destinationCord=np.array([-0.300,-0.075,0.050])
                    DPose=self.rxarm.get_IK_joint_pose((destinationCord))

                    #calculating block joint angles

                    joint_angles = np.zeros(5)
                    num_angles = 4
                    joint_angles = np.append(Pose[0, 0:num_angles], joint_angles[num_angles:joint_angles.shape[0]])
                    print(joint_angles)

                    #calculating block joint angles

                    intjoint_angles = np.zeros(5)
                    intnum_angles = 4
                    intjoint_angles = np.append(intPose[0, 0:intnum_angles], intjoint_angles[intnum_angles:intjoint_angles.shape[0]])
                    print(intjoint_angles)

                    #calculating place destination joint angles
                    Djoint_angles = np.zeros(5)  #destination angles
                    Dnum_angles = 4
                    Djoint_angles = np.append(DPose[0, 0:Dnum_angles], Djoint_angles[Dnum_angles:Djoint_angles.shape[0]])
                    print(Djoint_angles)

                    self.moving_time = 2.0
                    self.accel_time = 1.0

                    
                    #Motion sequence: Go to home position, go to defined position, return to home
                    self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                    accel_time=self.accel_time,
                                    blocking=False)
                    rospy.sleep(2)
                    
                    self.rxarm.set_positions(intjoint_angles)
                    rospy.sleep(1.5)
                    self.rxarm.set_positions(joint_angles)
                    rospy.sleep(1.5)
                    self.rxarm.set_positions(intjoint_angles)
                    rospy.sleep(1.5)

                    self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                    accel_time=self.accel_time,
                                    blocking=False)
                    rospy.sleep(2)
                    
                    self.rxarm.set_positions(Djoint_angles)
                    rospy.sleep(1.5)

                    self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                    accel_time=self.accel_time,
                                    blocking=False)
                    rospy.sleep(2)


        if (self.sm.current_state == "PNPL"):  #Pick up and place them in a line ROYGBV order   -- Level 2  block avoidance not sure but maybe shape thing can held    
            self.camera.blockDetector()
            #loop for bigger blocks
            seq = ['Red',"Orange",'Yello','Green','Blue','Violet']
            for j in range(0,len(self.camera.contours)):
                for i in range(0,len(self.camera.contours)):
                    if self.camera.conta > 1000 and self.camera.conta< 5000:  #change limits
                        if self.camera.contc[i]==(seq[len(seq)-1]):  # this will only let the block placement begin if it is the last color on the lise
                            seq.pop(seq[len(seq)-1]) #once in the last of the list is poped so that next time it goes for the nxt color
                            worldCord= np.array([self.camera.wx[i],self.camera.wy[i],self.camera.wz[i]])
                            Pose = self.rxarm.get_IK_joint_pose((worldCord/1000.0))
                            destinationCord=np.array([-0.400,(-0.075+j*0.055),0.050])   #multiplying 0.6 with increade the factor added in Y making the placement center to move up the Y axis
                            DPose=self.rxarm.get_IK_joint_pose((destinationCord))

                            #calculating block joint angles

                            joint_angles = np.zeros(5)
                            num_angles = 4
                            joint_angles = np.append(Pose[0, 0:num_angles], joint_angles[num_angles:joint_angles.shape[0]])
                            print(joint_angles)

                            #calculating place destination joint angles
                            Djoint_angles = np.zeros(5)  #destination angles
                            Dnum_angles = 4
                            Djoint_angles = np.append(DPose[0, 0:Dnum_angles], Djoint_angles[Dnum_angles:Djoint_angles.shape[0]])
                            print(Djoint_angles)

                            self.moving_time = 2.0
                            self.accel_time = 1.0

                            
                            #Motion sequence: Go to home position, go to defined position, return to home
                            self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                            accel_time=self.accel_time,
                                            blocking=False)
                            rospy.sleep(2)
                            
                            self.rxarm.set_positions(joint_angles)
                            rospy.sleep(1.5)

                            self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                            accel_time=self.accel_time,
                                            blocking=False)
                            rospy.sleep(2)
                            
                            self.rxarm.set_positions(Djoint_angles)
                            rospy.sleep(1.5)

                            self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                            accel_time=self.accel_time,
                                            blocking=False)
                            rospy.sleep(2)

            #loop for smaller blocks                
            seq = ['Red',"Orange",'Yello','Green','Blue','Violet']
            for j in range(0,len(self.camera.contours)):
                for i in range(0,len(self.camera.contours)):
                    if self.camera.conta >100 and self.camera.conta< 1000:  #change limits
                        if self.camera.contc[i]==(seq[len(seq)-1]):  # this will only let the block placement begin if it is the last color on the lise
                            seq.pop(seq[len(seq)-1]) #once in the last of the list is poped so that next time it goes for the nxt color
                            worldCord= np.array([self.camera.wx[i],self.camera.wy[i],self.camera.wz[i]])
                            Pose = self.rxarm.get_IK_joint_pose((worldCord/1000.0))
                            destinationCord=np.array([0.400,(-0.075+j*0.055),0.050])   #multiplying 0.6 with increade the factor added in Y making the placement center to move up the Y axis
                            DPose=self.rxarm.get_IK_joint_pose((destinationCord))

                            #calculating block joint angles

                            joint_angles = np.zeros(5)
                            num_angles = 4
                            joint_angles = np.append(Pose[0, 0:num_angles], joint_angles[num_angles:joint_angles.shape[0]])
                            print(joint_angles)

                            #calculating place destination joint angles
                            Djoint_angles = np.zeros(5)  #destination angles
                            Dnum_angles = 4
                            Djoint_angles = np.append(DPose[0, 0:Dnum_angles], Djoint_angles[Dnum_angles:Djoint_angles.shape[0]])
                            print(Djoint_angles)

                            self.moving_time = 2.0
                            self.accel_time = 1.0

                            
                            #Motion sequence: Go to home position, go to defined position, return to home
                            self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                            accel_time=self.accel_time,
                                            blocking=False)
                            rospy.sleep(2)
                            
                            self.rxarm.set_positions(joint_angles)
                            rospy.sleep(1.5)

                            self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                            accel_time=self.accel_time,
                                            blocking=False)
                            rospy.sleep(2)
                            
                            self.rxarm.set_positions(Djoint_angles)
                            rospy.sleep(1.5)

                            self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                            accel_time=self.accel_time,
                                            blocking=False)
                            rospy.sleep(2)
        
        if (self.sm.current_state == "Sort"):  #Pick up and place them in a line           
            self.camera.blockDetector()
            color=np.array([['Green','Blue', 'RED','PURPLE','ORANGE','YELLOW'],
                            [-0.35,-0.25,-0.15,0.15,0.25,0.35]
                            [-1,-1,-1,-1,-1,-1]])                      #each color has a place coordinate associated with it
                    #starts with -1 coz as a block is found it will make it 0 appropriate for the later calculations
            for i in range(0,len(self.camera.contours)):
                if self.camera.contc[i]!='VIBGYOR':

                    for i in range(0,5):
                        if self.camera.contc[i]==color[0,i]:
                            color[2,i]=1+color[2,i]  #this will increment each color block stack length
                            break                    # loop breaks once color is matched and that i is used to determine the placement position
                    
                    worldCord= np.array([self.camera.wx[i],self.camera.wy[i],self.camera.wz[i]])
                    Pose = self.rxarm.get_IK_joint_pose((worldCord/1000.0))
                    destinationCord=np.array([color[1,i],-0.075,(0.01+color[2,i]*0.05)])  #depending on the number of blocks already stacked this will increment the destination height of the end effector
                    DPose=self.rxarm.get_IK_joint_pose((destinationCord))

                    #calculating block joint angles

                    joint_angles = np.zeros(5)
                    num_angles = 4
                    joint_angles = np.append(Pose[0, 0:num_angles], joint_angles[num_angles:joint_angles.shape[0]])
                    print(joint_angles)

                    #calculating place destination joint angles
                    Djoint_angles = np.zeros(5)  #destination angles
                    Dnum_angles = 4
                    Djoint_angles = np.append(DPose[0, 0:Dnum_angles], Djoint_angles[Dnum_angles:Djoint_angles.shape[0]])
                    print(Djoint_angles)

                    self.moving_time = 2.0
                    self.accel_time = 1.0

                    
                    #Motion sequence: Go to home position, go to defined position, return to home
                    self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                    accel_time=self.accel_time,
                                    blocking=False)
                    rospy.sleep(2)
                    
                    self.rxarm.set_positions(joint_angles)
                    rospy.sleep(1.5)

                    self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                    accel_time=self.accel_time,
                                    blocking=False)
                    rospy.sleep(2)
                    
                    self.rxarm.set_positions(Djoint_angles)
                    rospy.sleep(1.5)

                    self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                    accel_time=self.accel_time,
                                    blocking=False)
                    rospy.sleep(2)

        if (self.sm.current_state == "SStack"):  #Pick and sort size wize can do leven 2 now, if want to go to level 3 will need to put efforts        
            self.camera.blockDetector()
            print('in sstack')
            BigS=0    #block stacked
            SmallS=0
            for i in range(0,len(self.camera.contours)):
                print('sorting loop')
                if self.camera.contc[i]!='VIBGYOR':
                    print('in color loop')
                    worldCord= np.array([self.camera.wx[i],self.camera.wy[i],(self.camera.wz[i]-20)])
                    Pose = self.rxarm.get_IK_joint_pose((worldCord/1000.0))
                    intWorldCord=np.array([worldCord[0],worldCord[1],(worldCord[2]+70)])
                    intPose = self.rxarm.get_IK_joint_pose((intWorldCord/1000.0))
                    destinationCord=np.array([0,0,0])
                    print('world pose')
                    print(worldCord)
                    if self.camera.conta[i]>1000 and self.camera.conta[i]<5000:
                        destinationCord=np.array([-0.25,-0.075,(0.01+BigS*0.05)])
                        BigS=BigS+1
                    else:
                        destinationCord=np.array([0.25,-0.075,(0.01+SmallS*0.05)])
                        SmallS=SmallS+1
                        
                    DPose=self.rxarm.get_IK_joint_pose((destinationCord))

                    intDCord=np.array([destinationCord[0],destinationCord[1],destinationCord[2]+100])
                    intDPose = self.rxarm.get_IK_joint_pose((intDCord))
                    
                    #calculating block joint angles

                    joint_angles = np.zeros(5)
                    num_angles = 4
                    joint_angles = np.append(Pose[0, 0:num_angles], joint_angles[num_angles:joint_angles.shape[0]])
                    print(joint_angles)

                    intDjoint_angles = np.zeros(5)
                    intDnum_angles = 4
                    intDjoint_angles = np.append(intDPose[0, 0:intDnum_angles], intDjoint_angles[intDnum_angles:intDjoint_angles.shape[0]])
                    print(intDjoint_angles)


                    intjoint_angles = np.zeros(5)
                    intnum_angles = 4
                    intjoint_angles = np.append(intPose[0, 0:intnum_angles], intjoint_angles[intnum_angles:intjoint_angles.shape[0]])
                    print(intjoint_angles)


                    #calculating place destination joint angles
                    Djoint_angles = np.zeros(5)  #destination angles
                    Dnum_angles = 4
                    Djoint_angles = np.append(DPose[0, 0:Dnum_angles], Djoint_angles[Dnum_angles:Djoint_angles.shape[0]])
                    print(Djoint_angles)
                    self.moving_time = 2.0
                    self.accel_time = 1.0
                    #Motion sequence: Go to home position, go to defined position, return to home
                    print('move to initial')

                    self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                accel_time=self.accel_time,
                                blocking=False)
                    rospy.sleep(2)
                        
                    print('block')

                    self.rxarm.set_positions(intjoint_angles)
                    rospy.sleep(1.5)
                    self.rxarm.set_positions(joint_angles)
                    rospy.sleep(2)
                    self.rxarm.close_gripper()
                    rospy.sleep(1)
                   
                
                    self.rxarm.set_positions(intjoint_angles)
                    rospy.sleep(1.5)

                    self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                    accel_time=self.accel_time,
                                    blocking=False)
                    rospy.sleep(2)
                    print('estinationD')

                    self.rxarm.set_positions(intDjoint_angles)
                    rospy.sleep(1.5)
                    self.rxarm.set_positions(Djoint_angles)
                    rospy.sleep(2)
                    self.rxarm.open_gripper()
                    rospy.sleep(1)
                    self.rxarm.set_positions(intDjoint_angles)
                    rospy.sleep(1.5)
                    print('move to initial')
                    self.rxarm.go_to_home_pose(moving_time=self.moving_time,
                                    accel_time=self.accel_time,
                                    blocking=False)
                    rospy.sleep(2)   
                        
        #addy part ended
        
    def initRxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.ui.SliderFrame.setEnabled(False)
        self.ui.chk_directcontrol.setChecked(False)
        self.rxarm.enable_torque()
        self.sm.set_next_state('initialize_rxarm')


### TODO: Add ability to parse POX config file as well
def main(args=None):
    """!
    @brief      Starts the GUI
    """
    app = QApplication(sys.argv)
    app_window = Gui(dh_config_file=args['dhconfig'])
    app_window.show()
    sys.exit(app.exec_())


# Run main if this file is being run directly
### TODO: Add ability to parse POX config file as well
if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-c",
                    "--dhconfig",
                    required=False,
                    help="path to DH parameters csv file")
    main(args=vars(ap.parse_args()))

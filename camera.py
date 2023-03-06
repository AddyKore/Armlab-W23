"""!
Class to represent the camera.
"""

import cv2
import time
import numpy as np
from PyQt4.QtGui import QImage
from PyQt4.QtCore import QThread, pyqtSignal, QTimer
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from apriltag_ros.msg import *
from cv_bridge import CvBridge, CvBridgeError


class Camera():
    """!
    @brief      This class describes a camera.
    """
    def __init__(self):
        """!
        @brief      Construcfalsets a new instance.
        """
        self.contours=[]
        self.contx=[]
        self.conty=[]
        self.contz=[]
        self.conta=[]
        self.wcord=[]

        self.wx=[]
        self.wy=[]
        self.wz=[]
        self.conts=[]
        self.contc=[]
        
        

        self.image=np.zeros((720, 1280, 3)).astype(np.uint8)
        self.VideoFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.TagImageFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.DepthFrameRaw = np.zeros((720, 1280)).astype(np.uint16)
        self.HomographyFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.pointsFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.DepthFrameRGB = np.array([])

        # mouse clicks & calibration variables
        self.cameraCalibrated = False
        self.intrinsic_matrix = np.array([])#[917.94114, 0 , 644.63487],[0.0,918.95864,341.59458],[0,0,1]])
        self.extrinsic_matrix = np.array([[ 1,0,0, 20],[0,-0.95,-0.309, 200],[0,0.309,-0.95, 985],[0,0,0,1]])
        self.Homography_matrix = np.array([[1.311e+00, -1.8158e-01, -2.56e+02], [3.63e-02, 1.068e+00, -6.435e+01], [5.32e-06, -2.389e-04, 1.0e+00]])#np.zeros((3,3))

        #Apriltag array
        self.last_click = np.array([0, 0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5, 2), int)
        self.depth_click_points = np.zeros((5, 2), int)
        self.tag_detections = np.array([])
        self.tag_locations = [[-250, -25], [250, -25], [250, 275]]
        """ block info """
        self.block_contours = np.array([])
        self.block_detections = np.array([20,50,50])

    def processVideoFrame(self):
        """!
        @brief      Process a video frame
        """
        cv2.drawContours(self.VideoFrame, self.block_contours, -1,
                         (255, 0, 255), 3)

    def ColorizeDepthFrame(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        self.DepthFrameHSV[..., 0] = self.DepthFrameRaw >> 1
        self.DepthFrameHSV[..., 1] = 0xFF
        self.DepthFrameHSV[..., 2] = 0x9F
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV,
                                          cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """

        self.VideoFrame = cv2.cvtColor(
            cv2.imread("data/rgb_image.png", cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB)

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread("data/raw_depth.png",
                                        0).astype(np.uint16)

    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            
            cv2.drawContours(self.VideoFrame, self.contours, -1, (0,255,255), 3)
                    
            for j in range(0,len(self.contx)):
                #cord=[self.wx(j)]
                cv2.putText(self.VideoFrame, self.contc[j], (self.contx[j],self.conty[j]), cv2.FONT_HERSHEY_TRIPLEX, 0.5 , (255,255,255), 1)
                cv2.putText(self.VideoFrame, self.conts[j], (self.contx[j]-20,self.conty[j]-25), cv2.FONT_HERSHEY_TRIPLEX, 0.5 , (255,255,255), 1)
                #cv2.putText(self.VideoFrame, str(self.contx[j])+','+str(self.conty[j])+','+str(self.contz[j]), (self.contx[j]-40,self.conty[j]+35), cv2.FONT_HERSHEY_TRIPLEX, 0.5 , (255,255,255), 1) 
                
                #cv2.putText(self.VideoFrame, str(self.wx[j])+','+str(self.wy[j])+','+str(self.wz[j]), (self.contx[j]-40,self.conty[j]+35), cv2.FONT_HERSHEY_TRIPLEX, 0.5 , (255,255,255), 1) 
                #print(len(self.wcord(j)))  
          
            frame = cv2.resize(self.VideoFrame, (1280, 720))            
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtDepthFrame(self):
        """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
        try:
            img = QImage(self.DepthFrameRGB, self.DepthFrameRGB.shape[1],
                         self.DepthFrameRGB.shape[0], QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtTagImageFrame(self):
        """!
        @brief      Converts tag image frame to format suitable for Qt

        @return     QImage
        """

        try:

            frame = cv2.resize(self.TagImageFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtPointFrame(self):
        try:

            frame = cv2.resize(self.HomographyFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

        @param      coord1  Points in coordinate frame 1
        @param      coord2  Points in coordinate frame 2

        @return     Affine transform between coordinates.
        """
        pts1 = coord1[0:3].astype(np.float32)
        pts2 = coord2[0:3].astype(np.float32)
        print(cv2.getAffineTransform(pts1, pts2))
        return cv2.getAffineTransform(pts1, pts2)

    def loadCameraCalibration(self, file):
        """!
        @brief      Load camera intrinsic matrix from file.

                    TODO: use this to load in any calibration files you need to

        @param      file  The file
        """
        pass

    def aprilTagDetect(self):
        data = self.tag_detections
        numtags = 5
        positions = np.zeros((numtags, 4))
        aprilTags = np.array([[7, -250, -25, 0], 
                              [2, 250, -25, 0], 
                              [3, 250, 275, 0], 
                              [4, -250, 275, 0], 
                              [5, -400, 75, 156]])
                              #[6, 150, 375, 155],
                              #[8, 350, 175, 245]])
        distCoeff = np.array([0.1536298245191574, -0.4935448169708252, -0.0008808146812953055, 0.0008218809380196035, 0.4401721])

        #Read and store apriltags in the corresponding index in positions array
        for det in data.detections:
            pos = det.pose.pose.pose.position
            id = det.id[0]
            #print(id)
            pos_arr = np.array([id, pos.x, pos.y, pos.z])
            for i in range(0,len(aprilTags)):
                if (aprilTags[i][0] == id):
                    if (positions[i, 0] != id):
                        positions[i] = pos_arr
                    #break
            print(positions)

        #Remove ID from apriltag positions and normalize Z
        coord = positions[0:positions.shape[0], 1:positions.shape[1]]
        for i in range(0, coord.shape[0]):
            coord[i] = np.divide(coord[i], coord[i, 2])
        
        #Translate to Pixel coordinates and remove noramlized Z
        pixel_coord = np.transpose(np.matmul(self.intrinsic_matrix, np.transpose(coord)))
        pixel_coord = pixel_coord[0:pixel_coord.shape[0], 0:pixel_coord.shape[1]-1]
        #print(pixel_coord)

        aprilTagCoords = aprilTags[0:aprilTags.shape[0], 1:aprilTags.shape[1]]
        #print(aprilTagCoords)

        #Calculate Extrinsic Rotation and Translation and populate the extrinsic matrix
        success, R_exp, t = cv2.solvePnP(aprilTagCoords.astype(np.float), 
                                                      pixel_coord.astype(np.float), 
                                                      self.intrinsic_matrix, 
                                                      distCoeff, flags = cv2.SOLVEPNP_ITERATIVE)
        R, _ = cv2.Rodrigues(R_exp)
        #print(_)
        #print(success)
        #print(R_exp)
        #print(t)
        self.extrinsic_matrix = np.row_stack((np.column_stack((R,t)), (0, 0, 0, 1)))
        #print(self.extrinsic_matrix)
        boardCorner = np.array([-450, 425, 0, 450, 425, 0, -450, -125, 0, 450, 125, 0]).reshape((4,3))
        aprilTagCoords = np.append(aprilTagCoords, boardCorner, axis=0)
        #Homography transform
        tagCameraCoords = np.matmul(self.extrinsic_matrix, np.transpose(np.append(aprilTagCoords, np.ones((aprilTagCoords.shape[0], 1)), axis = 1)))
        tagCameraCoords = np.transpose(tagCameraCoords)
        for i in range(0, tagCameraCoords.shape[0]):
            tagCameraCoords[i] = np.divide(tagCameraCoords[i], tagCameraCoords[i, 2])
        print(tagCameraCoords)
        tagPixelCoords = np.matmul(np.append(self.intrinsic_matrix, np.zeros((self.intrinsic_matrix.shape[0], 1)), axis=1), np.transpose(tagCameraCoords))
                                    
        tagPixelCoords = np.transpose(tagPixelCoords)
        tagPixelCoords = tagPixelCoords[0:tagPixelCoords.shape[0], 0:tagPixelCoords.shape[1]-1]
        self.pointsFrame = self.VideoFrame

        grid_coord = np.array([232, 107, 1103, 110, 272, 600, 1055, 606]).reshape((4,2))
        pixel_coord = np.append(pixel_coord, grid_coord, axis=0)
        for pt in pixel_coord.astype(int):
            cv2.circle(self.pointsFrame, tuple(pt), 5, (0, 255, 0), -1)
        cv2.imwrite("Homography_pre.png", self.pointsFrame)

        print(pixel_coord.astype(int))
        print(tagPixelCoords.astype(int))

        self.Homography_matrix = cv2.findHomography(pixel_coord[0:5, :].astype(int), tagPixelCoords[0:5, :].astype(int))[0]
        print(self.Homography_matrix)

        self.HomographyFrame = cv2.warpPerspective(self.pointsFrame, self.Homography_matrix, (1280, 720))
        cv2.imwrite("Homography_post.png", self.HomographyFrame)



    def blockDetector(self):
        print('detecting blocks')
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections

        """
        #self.contours=[0]
         
        img = self.VideoFrame    #setting rgb frame n video frame from in class variable to local variable
        cv2.imwrite("image.png",img)

        depth_data = self.DepthFrameRaw
        mask= np.zeros_like(depth_data,dtype=np.uint8)
        cv2.rectangle(mask, (275,120),(1100,720), 255, cv2.FILLED)  #drawing boxes
        cv2.rectangle(mask, (575,414),(723,720), 0, cv2.FILLED)

        cv2.rectangle(img, (275,120),(1100,720), (255, 0, 0), 2)
        cv2.rectangle(img, (575,414),(723,720), (255, 0, 0), 2)
        width=int(img.shape[1])
        height=int(img.shape[0])
        #print(f'width : {width}    ;    Height : {height}')

        img_hsv= cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        alpha_l= np.array([0,65,51])
        alpha_h= np.array([179,255,245])

        green_lower = np.array([35,138,51])       #done
        green_upper = np.array([78,234,106])

        blue_lower = np.array([0,189,0])     #done
        blue_upper = np.array([22,255,255])

        red_lower = np.array([119,67,106])   #done
        red_upper = np.array([142,255,202])

        purple_lower = np.array([0,98,71])      #done
        purple_upper = np.array([15,177,118])

        orange_lower = np.array([100,179,156])  #done
        orange_upper = np.array([120,255,255])

        yello_lower = np.array([91,119,173])    #done
        yello_upper = np.array([102,255,229])

        frame_mask= cv2.bitwise_and(cv2.inRange(img_hsv, alpha_l, alpha_h),mask)  # comming out with an image with only board visible
        #cv2.imshow('mask',frame_mask)
        frame_res= cv2.bitwise_and(img,img,mask=frame_mask)         #applying the mask to the BGR image
        #cv2.imshow('masked img',frame_res)
        _,contours_og, _= cv2.findContours(frame_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)    #finding contours for the mask

        # now that we have contors we will check the color of the area inside the contours

        
        cont=[]
        self.contc=[]
        self.contx=[]
        self.conty=[]
        self.wcord=[]
        for i in contours_og:
            #size detedtion
            c_area=cv2.contourArea(i)
            if c_area>400 and c_area<5500:
                self.conta.append(c_area)
                print(c_area)
                #position detection
                M = cv2.moments(i) #creating a center of the contour detected
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                x=center[1]
                y=center[0]
                
                green='green'
                d=depth_data[x][y]
                z=d
                
                self.contx.append(y)   #it is flipped it is right this way
                self.conty.append(x)
                self.contz.append(d)
                cont.append(i)

                cv2.circle(img,(y,x),4, (0,0,255),thickness=3)   

                #color detection
                h = img_hsv[x][y][0]
                s = img_hsv[x][y][1]
                v = img_hsv[x][y][2]  

                
                print("h  s  v")
                print(h)
                print(s)
                print(v)
                      
                cv2.putText(img, 'C- '+str(y)+','+str(x)+','+str(d), (y-50,x-20), cv2.FONT_HERSHEY_TRIPLEX, 0.5 , (255,255,255), 1)        
                #cv2.putText(img, str(h)+':'+str(s)+':'+str(v)+':', (y,x-20), cv2.FONT_HERSHEY_TRIPLEX, 0.5 , (255,255,255), 1)        
                if h>35 and h<80 and s>138 and s<234 and v>51 and v<120:
                    self.contc.append('Green')
                    print('Green')

                elif h>0 and h<22 and s>189 and s<=255 and v>0 and v<255:
                    self.contc.append('Blue')
                    print('blue')


                elif h>119 and h<142 and s>67 and s<255 and v>85 and v<202 :
                    self.contc.append('RED')
                    print('red')    

                

                elif h>91 and h<102 and s>119 and s<=255 and v>173 and v<230 :
                    self.contc.append('YELLOW')
                    print('yellow')
                
                elif h>100 and h<120 and s>50 and s<256 and v>156 and v<255 :
                    self.contc.append('ORANGE') 
                    print('orange')

                elif h>0 and h<205 and s>98 and s<=195 and v>71 and v<118 :
                    self.contc.append('PURPLE')
                    print('purple')
                    

                else:
                    self.contc.append('VIBGYOR') 
                
                
                    
                         
                
                
                #cam to world coordinates
                InverseIntrinsic=np.linalg.inv(self.intrinsic_matrix)
                InverseExtrinsic=np.linalg.inv(self.extrinsic_matrix)
                print(self.extrinsic_matrix)
                print('..')
                temp = np.array([[y],[x],[1]])
                camera_cord = np.matmul(InverseIntrinsic,(temp*z))
                
                self.world_cord = np.matmul(InverseExtrinsic,[int(camera_cord[0]),int(camera_cord[1]),int(camera_cord[2]),1])
                
                self.wcord.append(self.world_cord)

                self.wx.append(int(self.world_cord[0]))
                self.wy.append(int(self.world_cord[1]))
                self.wz.append(int(self.world_cord[2]))

                print("world Cord")
                wcc=[int(self.world_cord[0]),int(self.world_cord[1]),int(self.world_cord[2])]
                print(wcc)

                edges=cv2.approxPolyDP(i, 0.061*cv2.arcLength(i,True),True)
                #print(f"edges in i   {len(i)}")
                #print(f"edges in edges  {len(edges)}")

                if len(edges) == 3:
                    self.conts.append('Triangle')
                    cv2.putText(img, 'Triangle', (y,x),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                elif len(edges) == 4:
                    self.conts.append('square')
                    cv2.putText(img, 'square', (y,x+30),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                elif len(edges) == 5:
                    self.conts.append( 'Pentagon')
                    cv2.putText(img, 'Pentagon', (y,x),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                elif len(edges) == 6:
                    self.conts.append('Hexagon')
                    cv2.putText(img, 'Hexagon', (y,x),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                else:
                    self.conts.append('circle')
                    cv2.putText(img, 'circle', (y,x),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        self.contours=cont
        #cv2.imshow("Threshold window", thresh)
        '''
        cv2.imshow("Image window", img)
        cv2.waitKey(0)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

        
        
        print(f'the number of contours= {len(contours)}')
        for i in contours:
            b_area= cv2.contourArea(i) #this will give us the area of the contour
            if b_area>500 :
                print(f'the area of contour is {b_area}')
                cv2.drawContours(img, i, -1,(0,255,0), 5)

        cv2.imshow('img with contour',img)
        cv2.waitKey(0)
        '''

        # now that we have the image frame XYZ we need to convert them to world frame and how do we do that?
        #we use the intresnsic and extrensic we create from the april tags
        

    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth

                    TODO: Implement a blob detector to find blocks in the depth image
        """
        
        lower = 629
        upper=977
        rgb_image = self.VideoFrame    #setting rgb frame n video frame from in class variable to local variable
        depth_data = self.DepthFrameRaw
        mask = np.zeros_like(depth_data, dtype=np.uint8)
        #check videoframe n depth frame shape and if it same to the shape of the images we used to testing, coz that would alter the mask applied nxt  
        cv2.rectangle(mask, (275,120),(1100,720), 255, cv2.FILLED)  #drawing boxes
        cv2.rectangle(mask, (575,414),(723,720), 0, cv2.FILLED)
        cv2.rectangle(rgb_image, (275,120),(1100,720), (255, 0, 0), 2)
        cv2.rectangle(rgb_image, (575,414),(723,720), (255, 0, 0), 2)
        thresh = cv2.bitwise_and(cv2.inRange(depth_data, lower, upper), mask)
        # depending on your version of Opencv2, the following line could be:
        # contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_og, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        dcontours=[]

        for i in contours_og:
            #size detedtion
            c_area=cv2.contourArea(i)
            if c_area>500:
                dcontours.append(i)
                #position detection
                M = cv2.moments(i) #creating a center of the contour detected
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                x=center[1]
                y=center[0]
                d=depth_data[x][y]                
        cv2.drawContours(rgb_image, dcontours, -1, (0,255,255), 3)


class ImageListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
        self.camera.VideoFrame = cv_image


class TagImageListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
        self.camera.TagImageFrame = cv_image


class TagDetectionListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.tag_sub = rospy.Subscriber(topic, AprilTagDetectionArray,
                                        self.callback)
        self.camera = camera

    def callback(self, data):
        self.camera.tag_detections = data
        
        #for detection in data.detections:
        #print(detection.id[0])
        #print(detection.pose.pose.pose.position)


class CameraInfoListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.tag_sub = rospy.Subscriber(topic, CameraInfo, self.callback)
        self.camera = camera

    def callback(self, data):
        self.camera.intrinsic_matrix = np.reshape(data.K, (3, 3))
        #print(self.camera.intrinsic_matrix)


class DepthListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
            #cv_depth = cv2.rotate(cv_depth, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)
        self.camera.DepthFrameRaw = cv_depth
        #self.camera.DepthFrameRaw = self.camera.DepthFrameRaw/2
        self.camera.ColorizeDepthFrame()


class VideoThread(QThread):
    updateFrame = pyqtSignal(QImage, QImage, QImage, QImage)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent)
        self.camera = camera
        image_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
        camera_info_topic = "/camera/color/camera_info"
        tag_image_topic = "/tag_detections_image"
        tag_detection_topic = "/tag_detections"
        image_listener = ImageListener(image_topic, self.camera)
        depth_listener = DepthListener(depth_topic, self.camera)
        tag_image_listener = TagImageListener(tag_image_topic, self.camera)
        camera_info_listener = CameraInfoListener(camera_info_topic,
                                                  self.camera)
        tag_detection_listener = TagDetectionListener(tag_detection_topic,
                                                      self.camera)

    def run(self):
        if __name__ == '__main__':
            cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Tag window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Grid window", cv2.WINDOW_NORMAL)
            time.sleep(0.5)
        while True:
            rgb_frame = self.camera.convertQtVideoFrame()
            depth_frame = self.camera.convertQtDepthFrame()
            tag_frame = self.camera.convertQtTagImageFrame()
            point_frame = self.camera.convertQtPointFrame()
            if ((rgb_frame != None) & (depth_frame != None)):
                self.updateFrame.emit(rgb_frame, depth_frame, tag_frame, point_frame)
            time.sleep(0.03)
            if __name__ == '__main__':
                cv2.imshow(
                    "Image window",
                    cv2.cvtColor(self.camera.VideoFrame, cv2.COLOR_RGB2BGR))
                cv2.imshow("Depth window", self.camera.DepthFrameRGB)
                cv2.imshow(
                    "Tag window",
                    cv2.cvtColor(self.camera.TagImageFrame, cv2.COLOR_RGB2BGR))
                cv2.imshow(
                    "Grid window",
                    cv2.cvtColor(self.camera.HomographyFrame, cv2.COLOR_RGB2BGR))
                cv2.waitKey(3)
                time.sleep(0.03)


if __name__ == '__main__':
    camera = Camera()
    videoThread = VideoThread(camera)
    videoThread.start()
    rospy.init_node('realsense_viewer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

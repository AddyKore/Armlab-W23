"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm



def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle


def FK_dh(dh_params,joint_angles):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """
    #print("Transforming")
    HP = dh_params.copy()
    T=joint_angles

    #print(T)


    HP[0,0]=HP[0,0]+T[0]
    HP[1,0]=HP[1,0]-T[1]
    HP[2,0]=HP[2,0]+T[2]
    HP[3,0]=HP[3,0]+T[3]
    HP[4,0]=T[4]
   #print(HP)
    HT=np.array([[1,0,0,0],
                 [0,1,0,0],
                 [0,0,1,0],
                 [0,0,0,1]])
    
    for i in range(0,5):
        temp=get_transform_from_dh(np.array(HP[(i),:]))
        
        
        HT=np.matmul(HT, temp)
        '''
        print(i)
        print('')
        print(HT)
        print('')
        '''
    return HT 


def get_transform_from_dh(R):
    """!
    @brief      Gets the transformation matrix T from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transformation matrix.
    
    H=np.array([[np.cos(R[0]),  -1*np.sin(R[0])*np.cos(R[3]),     np.sin(R[0])*np.sin(R[3]), R[2]*np.cos(R[0]) ],
                [np.sin(R[0]),     np.cos(R[0])*np.cos(R[3]),  -1*np.cos(R[0])*np.sin(R[3]), R[2]*np.sin(R[0]) ],
                [           0,                  np.sin(R[3]),                  np.cos(R[3]),              R[1] ],
                [           0,                             0,                             0,                 1 ]])
    
    """
    Rz=np.array([[np.cos(R[0]), -np.sin(R[0]),   0,        0], #rotation along Z
             [np.sin(R[0]),  np.cos(R[0]),   0,        0],
             [0,          0,           1,        0],
             [0,          0,           0,        1]])

    Rx=np.array([[1,          0,           0,        0],#rotation along x
             [0,          np.cos(R[3]), -np.sin(R[3]), 0],
             [0,          np.sin(R[3]), np.cos(R[3]),  0],
             [0,          0,         0,          1]])

    Tz=np.array([[1,0,0,0],   #translation along Z
             [0,1,0,0],
             [0,0,1,R[1]],
             [0,0,0,1]])

    Tx=np.array([[1,0,0,R[2]],   #translation along X
             [0,1,0,0],
             [0,0,1,0],
             [0,0,0,1]])




    H = np.matmul(Rz,np.matmul(Tz,np.matmul(Tx,Rx)))

    return H


def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the 3 Euler angles from a 4x4 transformation matrix T
                If you like, add an argument to specify the Euler angles used (xyx, zyz, etc.)

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    pass


def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the 6DOF pose vector from a 4x4 transformation matrix T

    @param      T     transformation matrix

    @return     The pose vector from T.
    """
    pose=([int(T[0,3]),int(T[1,3]),int(T[2,3]),int(T[3,3])])
    pose=np.array([0,1,0,1])
    return pose


def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a  representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4x4 homogeneous matrix representing the pose of the desired link

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4x4 homogeneous matrix representing the pose of the desired link
    """
    pass


def to_s_matrix(w, v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    pass


def IK_geometric(dh_params, pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose vector as np.array to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose vector as np.array 

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    IKPoses = np.zeros((1,4))
    #Removing the case where Z occasionally is read as a negative value on the board
    pose[2] = pose[2] if(pose[2] >= 0.0) else 0.0

    if (CheckCoord(pose)):
        #Links listing is Base Height, Planar Arm Link 1, Planar Arm Link2, Wrist Length
        links = np.array([np.abs(dh_params[0, 1]), 
                        np.abs(dh_params[1, 2]), 
                        np.abs(dh_params[2, 2]),
                        np.abs(dh_params[4, 1])])
        #print(links)
        
        #wAngle = np.pi/2 - np.abs((np.pi/2)*(rad-limitRad)/(maxRad-limitRad))
        wAngle = AngleCalc(pose)

        #solving for base angle
        base = -np.arctan2(pose[0], pose[1])
        #print(base)

        coords = ChangeCoord(dh_params, pose, wAngle, base)
        #print(coords)

        #Solving for planar angles
        planar = solve2Link(coords, links[1:3])
        #print(planar)

        #Checks to make sure there is at least 1 valid configuration
        if (not CheckConfigs):
            print("This coordinate is unreachable")
            poses = np.zeros((4, 4))
            return
        else:
            for i in range(0,4):
                theta3 = (wAngle) + planar[i] - planar[(i%2) + 4]
                #print(theta3)
                newPose = np.array([[base, planar[i], planar[(i%2) + 4], theta3]])
                IKPoses = np.append(IKPoses, newPose, axis=0)

        IKPoses = IKPoses[1:, :]
    return IKPoses

def solve2Link(coords, links):
    #solving for angles based on links and coordinates
    #Distance from base gives us a new X distance.  Z-coordinate is new Y
    newX = np.sqrt(np.power(coords[0], 2) + np.power(coords[1], 2))
    newY = coords[2]

    theta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    theta2 = np.array([0.0,0.0, 0.0, 0.0])

    #Solve for the two cases of theta2 first
    ah = ((np.power(newX, 2) + np.power(newY, 2))- np.power(links[0], 2) - np.power(links[1],2))/(2*links[0]*links[1])
    #print(ah)
    theta2Solve = np.arccos(ah)
    theta2[0] = theta2Solve
    theta2[1] = -theta2Solve #Theta2 alternative solution
    theta2[2] = (np.pi/2) - theta2[0] - np.arctan2(50, 200)
    theta2[3] = (np.pi/2) - theta2[1] - np.arctan2(50, 200)
    #print(theta2)

    #Solve Theta1 based on Theta2
    for i in range(0,2):
        thetaDep = np.arctan2(links[1]*np.sin(theta2[i]), links[0] + links[1]*np.cos(theta2[i]))
        theta[i] = (np.pi/2) - np.arctan2(newY, newX) - thetaDep - np.arctan2(50, 200)

    theta[2] = np.pi + theta[0] #Theta[0] alternative solution
    theta[3] = np.pi + theta[1] #Theta[1] alternative solution
    theta[4:6] = theta2[2:4]         #Add theta2 into the array

    return theta

def ChangeCoord(dhParam, coords, wAngle, bAngle):
    dhparamLoc = dhParam.copy()
    newCoord = np.array([0, 0, 1, 0])
    coords = np.append(coords, 0.0)
    H = np.eye(4)
    dhparamLoc[3,0] += wAngle

    for i in range(3, 5):
        frameVals = dhparamLoc[i]
        Hnew = get_transform_from_dh(frameVals)
        H = np.matmul(H, Hnew)
        #print(H)

    H = np.matmul(get_transform_from_dh(np.array([-np.pi/2, 0.0, 0.0, -np.pi/2])), H) #Refram so Z-axis are aligned
    #print(np.matmul(dhParam[4, 1]*H, newCoord))
    H = np.matmul(get_transform_from_dh(np.array([bAngle, 0.0, 0.0, 0.0])), H) #account for base rotation and height
    #print(np.matmul(dhParam[4, 1]*H, newCoord))

    newCoord = coords + dhparamLoc[4, 1]*np.matmul(H, newCoord)
    newCoord[2] -= dhparamLoc[0,1]
    return newCoord

def CheckConfigs(pose):
    return

def CheckCoord(world_coord):
    robotLen = .579
    #print(world_coord)
    spherical_coord = np.sqrt(world_coord[0]**2 + world_coord[1]**2 + world_coord[2]**2)
    #print(spherical_coord)
    if (spherical_coord > (robotLen)):
        print("Coordinates are beyond the reach of the robot")
        return False
    elif ((np.abs(world_coord[0]) > .5) or ((world_coord[1] > .5) or (world_coord[1] < -0.175)) or ((world_coord[2] < 0) or (world_coord[2] > 1.0))):
        print("Coordinates are beyond the bounds of the workspace")
        return False
    else:
        return True
    
def AngleCalc(world_coords):
    cyl_coords = np.array([np.sqrt(world_coords[0]**2 + world_coords[1]**2), world_coords[2]])
    bounds_cyl = np.array([.400, 0.500])
    if (cyl_coords[1] < .230):
        if (cyl_coords[0] < bounds_cyl[0]):
            print("90 deg")
            return -np.pi/2
        elif cyl_coords[0] < bounds_cyl[1]:
            print("45 deg")
            return -np.pi/4
        else:
            print("0 deg")
            return 0
    else:
        print("0 deg")
        return 0
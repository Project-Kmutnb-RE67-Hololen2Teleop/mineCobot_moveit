#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler
from scipy.spatial.transform import Rotation as R
from enum import Enum
import pandas as pd
from std_msgs.msg import String
import time
import requests
import numpy as np
from sensor_msgs.msg import JointState
from urllib3.exceptions import InsecureRequestWarning
import warnings
import threading
class DEFAULT_VAR(Enum):
    ROW = 0
    PITCH = 0
    YAW = -90
    #-----
    DIGIT_CTRL = 2
    OFSET_UP = 5
    #-----
    OFSET_MANIP_X = 5.5   #ระยะจาก center ของ mobile ไป centerฐานของ manipulator
    OFSET_MANIP_Y = 0
    OFSET_MANIP_Z = 11.5    #ระยะสูงจาก center ของ mobile ไป centerฐานของ manipulator


class Transformation:
    def __init__(self):
        pass

    def Rx(self, Deg):
        rx = np.deg2rad(Deg)
        return np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])

    def Ry(self, Deg):
        ry = np.deg2rad(Deg)
        return np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])

    def Rz(self, Deg):
        rz = np.deg2rad(Deg)
        return np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])

    def setBaseManip(self, Pose, orien):
        self.R_baseManip = self.Rz(orien[2]) @ self.Ry(orien[1]) @ self.Rx(orien[0])
        self.BaseManip = np.eye(4)
        self.BaseManip[:3, :3] = self.R_baseManip
        self.BaseManip[:3, 3] = Pose

    def setObjectPose(self, Pose, orien):
        self.R_baseObject = self.Rz(orien[2]) @ self.Ry(orien[1]) @ self.Rx(orien[0])
        self.Object = np.eye(4)
        self.Object[:3, :3] = self.R_baseObject
        self.Object[:3, 3] = Pose

    def computeInverseBaseManip(self):
        R_inv = self.R_baseManip.T
        t = self.BaseManip[:3, 3]
        t_inv = -R_inv @ t

        self.BaseManip_inv = np.eye(4)
        self.BaseManip_inv[:3, :3] = R_inv
        self.BaseManip_inv[:3, 3] = t_inv

    def outputPosition(self):
        self.computeInverseBaseManip()  # เรียกก่อนเพื่อ set ค่า
        result = self.BaseManip_inv @ self.Object
        #print("Object pose in manipulator frame:\n", result)
        return result

jointState_Data = {
    "j1":0.0,
    "j2":0.0,
    "j3":0.0,
    "j4":0.0,
    "j5":0.0,
    "j6":0.0,
    "speed":0.0,
    "gripper":False
}
stages = ""
iteration = None
def JointCallBack(data):
    global jointState_Data , iteration
    jointState_Data["j1"] , jointState_Data["j2"] , jointState_Data["j3"] , jointState_Data["j4"] , jointState_Data["j5"] , jointState_Data["j6"] = np.rad2deg(data.position[:6])
    requests.post(url="https://188.166.222.52:12345/set_Manipulator/setMovementJoint",verify=False ,json=jointState_Data)
    

def move_to_pose(group, x, y, z, roll, pitch, yaw):
    global jointState_Data , iteration
    
    # Set the target pose
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    # Convert RPY to Quaternion (rotation in radians)
    q = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))  # Convert RPY to quaternion
    pose_target.orientation.x = q[0]
    pose_target.orientation.y = q[1]
    pose_target.orientation.z = q[2]
    pose_target.orientation.w = q[3]

    # Set the target pose
    group.set_pose_target(pose_target)

    # Plan and execute the motion (this will block until the robot finishes moving)
    success = group.go(wait=True)
    
    time.sleep(0.25)
    # print(success)
    if success:
        rospy.loginfo("Movement successful!")
    
    else:
        rospy.loginfo("Movement failed.")

def MoveSetPose(group,name:str):
    group.set_named_target(name)
    # Plan the motion
    plan = group.go(wait=True)

def Threading_CMDtoCTRLMoboile():
    global stages
    pub = rospy.Publisher("/CMD/ControlMoveMobile",String,queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(stages)
        rospy.sleep(0.1)
def main():
    global jointState_Data , iteration , stages
    # Initialize the moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_pose', anonymous=True)
    pub = rospy.Publisher("/gripper_trigger",String,queue_size=10)
    sub = rospy.Subscriber("joint_states", JointState, JointCallBack)
    # Get the robot's move group
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm_group")  # Replace "arm_group" with your group name
    SUB_TRIGGER = True
    prep_data = None
    control_mobile = threading.Thread(target = Threading_CMDtoCTRLMoboile)
    control_mobile.daemon = True
    control_mobile.start()
    
    try:
        # Example: Moving the robot to new poses sequentially
        while not rospy.is_shutdown():
            status_response = requests.get(url="https://188.166.222.52:12345/status/current",verify=False).json()
            if status_response['trigger'] == True:
                with open("./DataBag.csv","wb") as file :
                    file.write(requests.get(url="https://188.166.222.52:12345/download/csv/DataBag.csv",verify=False).content)
                requests.post(url="https://188.166.222.52:12345/status/update" , verify=False ,json={"trigger":False})
                SUB_TRIGGER = True
            if status_response['trigger'] == False:
                if SUB_TRIGGER == True :
                    data = pd.read_csv("./DataBag.csv")
                    msg = String()
                    print(len(data) - 100)
                    for i in range(len(data)):
                        
                        '''
                        ofset_x , ofset_y , ofset_z =  215 , 130 , 11.0   # recommend in distance X =  25 cm ref from manipulator base origin 
                        X, Y , Z ,= data.iloc[i]["Position_X"] - ofset_x , ofset_y - data.iloc[i]["Position_Y"] , data.iloc[i]["Position_Z"] - ofset_z
                        r , p , y = data.iloc[i]["Rotation_Roll"] + DEFAULT_VAR.ROW.value , data.iloc[i]["Rotation_Pitch"] + DEFAULT_VAR.PITCH.value , data.iloc[i]["Rotation_Yaw"] + DEFAULT_VAR.YAW.value
                        move_to_pose(group, 25/100, Y/100 , Z/100 ,(-1 )*p , r , y )'
                        '''
                        #set position of manipulator
                        response_MobilePose = requests.get("https://188.166.222.52:12345/MyAGV/Position/current",verify=False).json()
                        #transforms.setBaseManip([158.8 , 129 , 11 ],[0,0,0])
                        transforms.setBaseManip([response_MobilePose['x'] + DEFAULT_VAR.OFSET_MANIP_X.value,-(response_MobilePose['y']),DEFAULT_VAR.OFSET_MANIP_Z.value] , [response_MobilePose['row'],-(response_MobilePose['pitch']),response_MobilePose['yaw']])
                        transforms.setObjectPose([data.iloc[i]["Position_X"],(data.iloc[i]["Position_Y"]) ,data.iloc[i]["Position_Z"]],
                                                 [(data.iloc[i]["Rotation_Pitch"] + DEFAULT_VAR.PITCH.value) *-1 , data.iloc[i]["Rotation_Roll"] + DEFAULT_VAR.ROW.value, data.iloc[i]["Rotation_Yaw"] + DEFAULT_VAR.YAW.value])
                        matrix = transforms.outputPosition()
                        rotation = R.from_matrix(matrix[:3, :3])
                        Rr,Rp,Ry = rotation.as_euler('xyz', degrees=True)
                        x, y, z = matrix[:3, 3]
                        #print(x,y * -1 ,z)
                        print("iteration :",i)
                        print(response_MobilePose)
                        if i == 0 :
                            print("Iteration :",i+1,"/",len(data)+1 , "start")
                            MoveSetPose(group,"prep_pose")
                            time.sleep(2)
                            move_to_pose(group, round(x/100,DEFAULT_VAR.DIGIT_CTRL.value) , round((y/100)*-1,DEFAULT_VAR.DIGIT_CTRL.value), round(((z+DEFAULT_VAR.OFSET_UP.value)/100),DEFAULT_VAR.DIGIT_CTRL.value) ,Rr , Rp , Ry ) 
                            time.sleep(10)
                            move_to_pose(group, round(x/100,DEFAULT_VAR.DIGIT_CTRL.value) , round((y/100)*-1,DEFAULT_VAR.DIGIT_CTRL.value), round((z/100),DEFAULT_VAR.DIGIT_CTRL.value) ,Rr , Rp , Ry ) 
                            msg.data = "on"
                            rospy.loginfo("gripper on")
                            time.sleep(10)
                            iteration = "first"
                            pub.publish(msg)
                            jointState_Data['gripper'] = True
                            print(msg)
                            stages = "None"

                        if i == 50 :
                            stages = "movetoStep2"
                            print("MOVE2STAAGE2")
                            time.sleep(1)
                        if i > 0 and i < len(data) - 30:
                            msg.data = "pass"
                            pub.publish(msg)
                            #print(round(25/100,3) , round((y/100)*-1,3), round((z/100),3))
                            #print(prep_data)
                            if prep_data != [round(x/100,DEFAULT_VAR.DIGIT_CTRL.value) , round((y/100)*-1,DEFAULT_VAR.DIGIT_CTRL.value), round((z/100),DEFAULT_VAR.DIGIT_CTRL.value)]:
                                print("Iteration :",i+1,"/",len(data)+1)
                                move_to_pose(group, round(x/100,DEFAULT_VAR.DIGIT_CTRL.value) , round((y/100)*-1,DEFAULT_VAR.DIGIT_CTRL.value), round((z/100),DEFAULT_VAR.DIGIT_CTRL.value) ,Rr , Rp , Ry )   
                            
                            prep_data = [round(x/100,DEFAULT_VAR.DIGIT_CTRL.value) , round((y/100)*-1,DEFAULT_VAR.DIGIT_CTRL.value), round((z/100),DEFAULT_VAR.DIGIT_CTRL.value)]
                            stages = "None"

                        if i >= len(data) - 29 and i < len(data) -1 :
                            print("Iteration :",i+1 , "/",len(data)+1)
                            move_to_pose(group, x/100 , (y/100)*-1, z/100 ,Rr , Rp , Ry )
                            msg.data = "pass"
                            pub.publish(msg)
                            jointState_Data['gripper'] = True
                            stages = "None"
                        if i == len(data) - 1 :
                            print("Iteration :",i+1 , "/",len(data)+1,"Last")
                            move_to_pose(group, x/100 , (y/100)*-1, z/100 ,Rr , Rp , Ry )
                            iteration = "last"
                            msg.data = "off"
                            rospy.loginfo("gripper off")
                            pub.publish(msg)
                            jointState_Data['gripper'] = False
                            time.sleep(5)
                        
                            matrix_last = matrix @ np.array([[1,0,0,0],
                                                             [0,1,0,0],
                                                             [0,0,1,DEFAULT_VAR.OFSET_UP.value],
                                                             [0,0,0,1]])
                            print(matrix)
                            print("---")
                            print(matrix_last)
                            Lrotation = R.from_matrix(matrix_last[:3, :3])
                            LRr,LRp,LRy = Lrotation.as_euler('xyz', degrees=True)
                            Lx, Ly, Lz = matrix_last[:3, 3]
                            move_to_pose(group, Lx/100 , -(Ly/100), Lz/100 ,LRr , LRp , LRy )
                            print(matrix_last)
                            time.sleep(2)
                            MoveSetPose(group,"init_pose")
                            time.sleep(2)
                            stages = "None"
                        
                            
                        else:
                            msg.data = "pass"
                            stages = "None"
                            pub.publish(msg)

                        
                        #print(iteration)
                        
                    SUB_TRIGGER = False
            print("pending trajectory ")    
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
    finally:
        # Shutdown moveit_commander
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    transforms = Transformation()
    warnings.simplefilter('ignore', InsecureRequestWarning)
    main()
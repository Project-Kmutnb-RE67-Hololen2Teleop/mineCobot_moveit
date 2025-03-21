#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler

from enum import Enum
import pandas as pd
from std_msgs.msg import String
import time
import requests
import numpy as np
class DEFAULT_VAR(Enum):
    ROW = 0
    PITCH = 0
    YAW = -90

class Transformation:
    def __init__(self):
        pass
    def Rx(self,Deg):
        rx = np.deg2rad(Deg)
        return np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])
    def Ry(self,Deg):
        ry = np.deg2rad(Deg)
        return np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])

    def Rz(self , Deg):
        rz = np.deg2rad(Deg)
        return np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])

    def setbaseManip(self,Pose,orien):
        self.R_baseManip = self.Rz(orien[2]) @ self.Ry(orien[1]) @ self.Rx(orien[0])
        self.BaseManip = np.eye(4)
        self.BaseManip[:3, :3] = self.R_baseManip
        self.BaseManip[:3, 3] = [Pose[0], Pose[1] , Pose[2]]

    def setObjectPose(self,Pose,orien):
        self.R_baseObject = self.Rz(orien[2]) @ self.Ry(orien[1]) @ self.Rx(orien[0])
        self.Object = np.eye(4)
        self.Object[:3, :3] = self.R_baseObject
        self.Object[:3, 3] = [Pose[0], Pose[1] , Pose[2]]

    def inverseBaseManip(self):
        R_inv = self.R_baseManip.T  # Transpose ของ rotation
        t = self.BaseManip[:3, 3]
        t_inv = -R_inv @ t           # คำนวณ translation ใหม่

        BaseManip_inv = np.eye(4)
        BaseManip_inv[:3, :3] = R_inv
        BaseManip_inv[:3, 3] = t_inv

        return BaseManip_inv
    
    def outputPostion(self):
        return self.inverseBaseManip @ self.Object
    
def move_to_pose(group, x, y, z, roll, pitch, yaw):
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
    if success:
        rospy.loginfo("Movement successful!")
    else:
        rospy.loginfo("Movement failed.")

def main():
    # Initialize the moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_pose', anonymous=True)
    pub = rospy.Publisher("/gripper_trigger",String,queue_size=10)
    # Get the robot's move group
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm_group")  # Replace "arm_group" with your group name
    SUB_TRIGGER = True
    
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
                    
                    for i in range(len(data)):
                        #ofset_x , ofset_y , ofset_z =  215 , 130 , 11.0   # recommend in distance X =  25 cm ref from manipulator base origin 
                        #X, Y , Z ,= data.iloc[i]["Position_X"] - ofset_x , ofset_y - data.iloc[i]["Position_Y"] , data.iloc[i]["Position_Z"] - ofset_z
                        #r , p , y = data.iloc[i]["Rotation_Roll"] + DEFAULT_VAR.ROW.value , data.iloc[i]["Rotation_Pitch"] + DEFAULT_VAR.PITCH.value , data.iloc[i]["Rotation_Yaw"] + DEFAULT_VAR.YAW.value
                        #move_to_pose(group, 25/100, Y/100 , Z/100 ,(-1 )*p , r , y )
                        transforms.setbaseManip([215 , 130 , 11.0 ],[0,0,0])
                        transforms.setObjectPose([data.iloc[i]["Position_X"],data.iloc[i]["Position_Y"],data.iloc[i]["Position_Z"]],
                                                 [(data.iloc[i]["Rotation_Pitch"] + DEFAULT_VAR.PITCH.value) *-1 , data.iloc[i]["Rotation_Roll"] + DEFAULT_VAR.ROW.value, data.iloc[i]["Rotation_Yaw"] + DEFAULT_VAR.YAW.value])
                        print(transforms.outputPostion)
                        if i == 0 :
                            msg.data = "on"
                            rospy.loginfo("gripper on")
                            time.sleep(10)
                            pub.publish(msg)
                            
                        if i == len(data) - 1 :
                            msg.data = "off"
                            rospy.loginfo("gripper off")
                            pub.publish(msg)
                            time.sleep(5)
                        else:
                            msg.data = "pass"
                            pub.publish(msg)
                        
                    SUB_TRIGGER = False
                
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
    finally:
        # Shutdown moveit_commander
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    transforms = Transformation()
    main()
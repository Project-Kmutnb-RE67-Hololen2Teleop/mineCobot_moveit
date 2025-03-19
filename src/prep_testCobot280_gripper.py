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

class DEFAULT_VAR(Enum):
    ROW = 0
    PITCH = 0
    YAW = -90


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
    SUB_TRIGGER = False
    
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
                        ofset_x , ofset_y , ofset_z =  225 , 130 , 11.0
                        X, Y , Z ,= data.iloc[i]["Position_X"] - ofset_x , data.iloc[i]["Position_Y"] - ofset_y , data.iloc[i]["Position_Z"] - ofset_z
                        r , p , y = data.iloc[i]["Rotation_Roll"] + DEFAULT_VAR.ROW.value , data.iloc[i]["Rotation_Pitch"] + DEFAULT_VAR.PITCH.value , data.iloc[i]["Rotation_Yaw"] + DEFAULT_VAR.YAW.value
                        move_to_pose(group, X/100, Y/100 , Z/100 ,r , p , y )

                        if i == 0 :
                            msg.data = "on"
                            rospy.loginfo("gripper on")
                            pub.publish(msg)
                            time.sleep(5)
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
    main()
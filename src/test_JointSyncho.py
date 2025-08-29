import rospy
import numpy as np
import requests
import threading
from sensor_msgs.msg import JointState

jointState_Data = {
    "j1": 0.0,
    "j2": 0.0,
    "j3": 0.0,
    "j4": 0.0,
    "j5": 0.0,
    "j6": 0.0,
    "speed": 0.0,
    "gripper": False
}

def send_to_server(data):
    try:
        requests.post(url="https://188.166.222.52:12345/set_Manipulator/setMovementJoint",
                      verify=False, json=data)
    except Exception as e:
        rospy.logwarn(f"Failed to send HTTP request: {e}")

def JointCallBack(data):
    global jointState_Data
    jointState_Data["j1"], jointState_Data["j2"], jointState_Data["j3"], jointState_Data["j4"], jointState_Data["j5"], jointState_Data["j6"] = np.rad2deg(data.position[:6])
    rospy.loginfo(f"Updated joint states: {jointState_Data}")
    
    # ส่ง HTTP request ใน background โดยไม่บล็อก
    threading.Thread(target=send_to_server, args=(jointState_Data.copy(),)).start()

if __name__ == '__main__':
    rospy.init_node("testMManipSynchronize", anonymous=True)
    rospy.Subscriber("/joint_states", JointState, JointCallBack)
    rospy.spin()

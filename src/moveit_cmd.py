#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import math
import httpx

from tf.transformations import quaternion_from_euler

def move_to_pose(group, x:float, y:float, z:float, roll, pitch, yaw):
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
    # initialize url http protocol
    url= "https://188.166.222.52:12345/get_Manipulator/Coordinate"

    # Initialize the moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_pose', anonymous=True)

    # Get the robot's move group
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm_group")  # Replace "arm_group" with your group name

    try:
        with httpx.Client(http2=True, verify=False) as client:
            # Example: Moving the robot to new poses sequentially
            while not rospy.is_shutdown():
                '''
                # First pose
                x, y, z = 0.2, 0.0, 0.15
                roll, pitch, yaw = -180, 0, 0
                move_to_pose(group, x, y, z, roll, pitch, yaw)

                # Second pose (after the first one completes)
                x, y, z = 0.2, 0.0, 0.2
                roll, pitch, yaw = -180, 0, 0
                move_to_pose(group, x, y, z, roll, pitch, yaw)
                # Second pose (after the first one completes)
                x, y, z = 0.2, 0.10, 0.2
                roll, pitch, yaw = -180, 0, 0
                move_to_pose(group, x, y, z, roll, pitch, yaw)
                '''
                response = client.get(url)
                if response.status_code == 200:
                    data = response.json()
                    print(data)
                    #print(type(data["X"]))
                    move_to_pose(group, data["X"], data["Y"], data["Z"], -180, 0, 0)
                    #mc.send_coords([data["X"],data["Y"],data["Z"],data["Rx"],data["Ry"],data["Rz"]],data["speed"])

                    
                
                rospy.sleep(0.1)  # Sleep for a while before sending the next pose

    except rospy.ROSInterruptException:
        pass
    finally:
        # Shutdown moveit_commander
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
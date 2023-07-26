#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
from time import time,sleep
from itertools import repeat
def create_pose_stamped(navigator: BasicNavigator, position_x): #, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, position_x[-1]) # orientation_z)
    pose = PoseStamped()
    
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x[0]
    pose.pose.position.y = position_x[1]
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main():
    # --- Init
    rclpy.init()
    nav = BasicNavigator()
    
    #USE THESE LOCATIONS IN MAP FOR NAVIGATING
    Home = (0.0,0.0,0.0) 
    A = (17.0, 5.8, 3.2)
    B = (nav, 8.0, 1.0, -1.57)
    C = (nav, 8.0, -0.5, 1.57)
    D = (nav, 5.0, 5.0, 3.14)
    E = (nav, 3.0, 4.0, 1.57)
    F = (nav, 4.0, 5.0, 0.0)
    G = (nav, 5.0, 3.0, -1.57)
    H = (nav, 4.0, 0.8, 3.14)
    I = (nav, -4.0, 3.5, -1.57)
    J = (nav, -4.0, 0.0, 1.57)

    Goals = []
    while len(Goals) < 24:
        Goal = input("Please enter the Table Number")
        #print("Heading Towards the Table",Goal)
        Goals.extend(repeat(Goal,8))
        Goals.append(Goal)
        if Goal == '1' or len(Goals) == 24:
            print(Goals)
            break

   
    # --- Set initial pose
    initial_pose = create_pose_stamped(nav,Home)
    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2
    nav.waitUntilNav2Active()
    waypoints = []
    # --- Send Nav2 goal
    for Goal in Goals: 
        if Goal == 'D':
        	Goal = (17.0, 5.8, 3.2)
        elif Goal =='A':
        	Goal = (4.04,0.69,1.507)
        elif Goal =='B':
        	Goal = (8.04,3.62,-0.05)
        elif Goal =='C':
        	Goal = (10.26,5.877,1.5574)
        waypoints.append(create_pose_stamped(nav,Goal))
    print('please collect Your Order')
    print("Got "len(Goals) "Goals") 
    sleep(5)
    waypoints.append(create_pose_stamped(nav,Home))
    waypoints.append(create_pose_stamped(nav,Home))
    waypoints.append(create_pose_stamped(nav,Home))
    waypoints.append(create_pose_stamped(nav,Home))
    waypoints.append(create_pose_stamped(nav,Home))
    waypoints.append(create_pose_stamped(nav,Home))
    waypoints.append(create_pose_stamped(nav,Home))
    waypoints.append(create_pose_stamped(nav,Home))   
    # --- Follow waypoints
    nav.followWaypoints(waypoints)
    
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)
    print(nav.getResult())
    # --- Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    


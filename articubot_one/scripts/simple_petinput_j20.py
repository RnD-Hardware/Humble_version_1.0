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
    Home = (0.0857,0.0203,-0.643)
    #USE THESE LOCATIONS IN MAP FOR NAVIGATING
    Locations = {
    'Home' : (0.50439,-0.2294,-0.65),
    'O' : (0.50439,-0.2294,-0.65),
    'A' : (6.23,-4.55,0.8862),
    'B' : (13.98,-11.313,2.413),
    'C' : (18.075,-7.045,2.455),
    'D' : (9.8557,-0.195,-2.33),
    'E' : (3.8405,4.749,-0.695)
    }
    


    Goals = []
    IP = input("Where should I start from....?") 
    while len(Goals) < 64:
        Goal = input("Please enter the Table Code (from A TO H) ")
        #print("Heading Towards the Table",Goal)
        Goals.extend(repeat(Goal,8))
        Goals.append(Goal)
        if Goal == '1' or len(Goals) == 24:
            print(Goals)
            break

   
    # --- Set initial pose
     
    initial_pose = create_pose_stamped(nav,Locations[IP])
    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2
    nav.waitUntilNav2Active()
    waypoints = []
    # --- Send Nav2 goal
    for Goal in Goals: 
        if Goal =='A':
        	Goal = (6.23,-4.55,0.8862)
        elif Goal == 'B':
        	Goal = (13.88,-11.084,2.47)
        elif Goal =='C':
        	Goal = (18.075,-7.045,2.455)
        elif Goal =='D':
        	Goal = (9.8557,-0.195,-2.33)
        elif Goal == 'E':
        	Goal = (3.8405,4.749,-0.695)
        elif Goal == 'O':
        	Goal = (0.50439,-0.2294,-0.65)       	        	        
        waypoints.append(create_pose_stamped(nav,Goal))
    print('please collect Your Order')
    print("Got",len(Goals),"Goals") 
    sleep(2)
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
        #print(feedback)
        #print(nav.getResult())
    print(nav.getResult())
    # --- Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()

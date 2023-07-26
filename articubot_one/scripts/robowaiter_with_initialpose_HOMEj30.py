#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.duration import Duration
import time

initial_positions = {
    'A': [-1.9836, -1.0239, 0.8633, 0.5045],
    'B': [-5.8217, 5.6058, -0.5184, 0.855],
    'C': [-10.4721, 13.8117, -0.508, 0.8610],
    'D': [-15.5175, 10.8359, -0.495 , 0.8684],
    'E': [-10.8994,2.7036, 0.2538, 0.9672],
    'F': [-7.233,-3.6203, 0.8586, 0.5125]
    }
    
    
# Shelf positions for picking
shelf_positions = {
    'A': [-1.9836, -1.0239, 0.8633, 0.5045],
    'B': [-5.8217, 5.6058, -0.5184, 0.855],
    'C': [-10.4721, 13.8117, -0.508, 0.8610],
    'D': [-15.5175, 10.8359, -0.495 , 0.8684],
    'E': [-10.8994,2.7036, 0.2538, 0.9672],
    'F': [-7.233,-3.6203, 0.8586, 0.5125]
    }

# Shipping destination for picked products
shipping_destinations = {
    'A': [-1.9836, -1.0239, 0.8633, 0.5045],
    'B': [-5.8217, 5.6058, -0.5184, 0.855],
    'C': [-10.4721, 13.8117, -0.508, 0.8610],
    'D': [-15.5175, 10.8359, -0.495 , 0.8684],
    'E': [-10.8994,2.7036, 0.2538, 0.9672],
    'F': [-7.233,-3.6203, 0.8586, 0.5125]
    }

"""
Basic item picking demo. In this demonstration, the expectation
is that there is a person at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with some kind of button for 'got item, robot go do next task').
"""


def main():
    # Recieved virtual request for picking item at Shelf A and bring to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the shelf ID ("shelf_A") and shipping destination ("pallet_jack7")
    ####################
    request_intial_pose = input("Where am I , please enter from where should I initialise")
    request_item_location = input("Where should I go to collect the food ")
    ti = int(input("How much time should I wait at collection point"))
    request_destination = input("Where should I deliver my food")
    me = int(input("How much time should I wait for collecting food")) 

    ####################

    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = initial_positions[request_intial_pose][0]
    initial_pose.pose.position.y = initial_positions[request_intial_pose][1]
    initial_pose.pose.orientation.z = initial_positions[request_intial_pose][2]
    initial_pose.pose.orientation.w = initial_positions[request_intial_pose][3]
    navigator.setInitialPose(initial_pose)
    
    # Set our demo's home pose Currently defining it as E 
    navigator.waitUntilNav2Active()

    home_pose = PoseStamped()
    home_pose.header.frame_id = 'map'
    home_pose.header.stamp = navigator.get_clock().now().to_msg()
    home_pose.pose.position.x = shelf_positions['E'][0]
    home_pose.pose.position.y = shelf_positions['E'][1]
    home_pose.pose.orientation.z = shelf_positions['E'][2]
    home_pose.pose.orientation.w = shelf_positions['E'][3]
    #navigator.goToPose(home_pose)

    

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_positions[request_item_location][2]
    shelf_item_pose.pose.orientation.w = shelf_positions[request_item_location][3]
    print(f'Received request for item picking at {request_item_location}.')
    if request_item_location == 'D':
        request_item_location = 'B'
        navigator.goToPose(shelf_item_pose)
        request_item_location = 'E'
        navigator.goToPose(shelf_item_pose)
    else:
        navigator.goToPose(shelf_item_pose)
    time.sleep(ti)

    # Do something during our route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Simply print information for workers on the robot's ETA for the demonstation
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time for picking food at table ' + request_item_location +
                  ' for robot : ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        #request_destination = input("Hello , I reached ,Where should I deliver next")
        print('Got food from ' + request_item_location +'! Bringing food to Table (' + request_destination + ')...')
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = shipping_destinations[request_destination][0]
        shipping_destination.pose.position.y = shipping_destinations[request_destination][1]
        shipping_destination.pose.orientation.z = shipping_destinations[request_destination][2]
        shipping_destination.pose.orientation.w = shipping_destinations[request_destination][3]
        navigator.goToPose(shipping_destination)
        
        
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at table' + request_destination +
                  ' for robot: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:

        time.sleep(me)
        navigator.goToPose(home_pose)
        
        
        
    elif result == TaskResult.CANCELED:
        print(f'Task at {request_item_location} was canceled. Returning to Home...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print(f'Task at {request_item_location} failed!')
        navigator.goToPose(home_pose)
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()

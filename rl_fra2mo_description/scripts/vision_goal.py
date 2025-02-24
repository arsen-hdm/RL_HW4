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
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.duration import Duration

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        
        self.aruco_pose_available = False

        # Sottoscrizione al topic "/aruco_single/pose"
        self.create_subscription(PoseStamped, '/aruco_single/pose', self.pose_callback, 10)      
        
    def pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received ArUco pose: {msg.pose.position.x}, {msg.pose.position.y}")
        self.aruco_pose_available = True

def main():
    rclpy.init()
    
    # Creazione del navigator
    navigator = BasicNavigator()
    
    node = NavigatorNode()
    
    package_path = get_package_share_directory('rl_fra2mo_description')
    yaml_file_path = os.path.join(package_path, 'config', 'pose_goals_vision.yaml')
    
    with open(yaml_file_path, 'r') as file:
        data = yaml.safe_load(file)

    # Set our demo's initial pose
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = navigator.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.orientation.z = 0.0
    init_pose.pose.orientation.w = 1.0

    navigator.setInitialPose(init_pose)
    
    goals = data['goals']
    
    # Accedere a goal_1
    goal_1 = goals['goal_1'][0]
    
    # Posizione
    position_goal_1 = goal_1['position']
    
    # Orientamento
    orientation_goal_1 = goal_1['orientation']
    
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_goal_1['x']
    goal_pose.pose.position.y = position_goal_1['y']
    goal_pose.pose.orientation.z = orientation_goal_1['Z']
    goal_pose.pose.orientation.w = orientation_goal_1['W']
    
    nav_start = navigator.get_clock().now()
    
    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")
    
    navigator.goToPose(goal_pose)
    
    while not navigator.isTaskComplete():
        pass
        
    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal 1 succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal 1 was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal 1 failed!')
    else:
        print('Goal 1 has an invalid return status!')
        
    while rclpy.ok() and node.aruco_pose_available is False:
        rclpy.spin_once(node, timeout_sec=0.1)
        
    navigator.goToPose(init_pose)
    
    while not navigator.isTaskComplete():
        pass
        
    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal 2 succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal 2 was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal 2 failed!')
    else:
        print('Goal 2 has an invalid return status!')
        
    rclpy.spin(node)

if __name__ == '__main__':
    main()


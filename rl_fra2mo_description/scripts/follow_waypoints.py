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
from rclpy.subscription import Subscription
import yaml
import os
from ament_index_python.packages import get_package_share_directory

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    package_path = get_package_share_directory('rl_fra2mo_description')
    yaml_file_path = os.path.join(package_path, 'config', 'pose_goals.yaml')
    
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
    goal_2 = goals['goal_2'][0]
    goal_3 = goals['goal_3'][0]
    goal_4 = goals['goal_4'][0]

    # Posizione
    position_goal_1 = goal_1['position']
    position_goal_2 = goal_2['position']
    position_goal_3 = goal_3['position']
    position_goal_4 = goal_4['position']

    # Orientamento
    orientation_goal_1 = goal_1['orientation']
    orientation_goal_2 = goal_2['orientation']
    orientation_goal_3 = goal_3['orientation']
    orientation_goal_4 = goal_4['orientation']
    
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = position_goal_1['x']
    goal_pose1.pose.position.y = position_goal_1['y']
    goal_pose1.pose.orientation.z = orientation_goal_1['Z']
    goal_pose1.pose.orientation.w = orientation_goal_1['W']
    goal_poses.append(goal_pose1)
    
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = position_goal_2['x']
    goal_pose2.pose.position.y = position_goal_2['y']
    goal_pose2.pose.orientation.z = orientation_goal_2['Z']
    goal_pose2.pose.orientation.w = orientation_goal_2['W']
    goal_poses.append(goal_pose2)
    
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = position_goal_3['x']
    goal_pose3.pose.position.y = position_goal_3['y']
    goal_pose3.pose.orientation.z = orientation_goal_3['Z']
    goal_pose3.pose.orientation.w = orientation_goal_3['W']
    goal_poses.append(goal_pose3)
    
    goal_pose4 = PoseStamped()
    goal_pose4.header.frame_id = 'map'
    goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose4.pose.position.x = position_goal_4['x']
    goal_pose4.pose.position.y = position_goal_4['y']
    goal_pose4.pose.orientation.z = orientation_goal_4['Z']
    goal_pose4.pose.orientation.w = orientation_goal_4['W']
    goal_poses.append(goal_pose4)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)


    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    nav_start = navigator.get_clock().now()
    
    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(goal_poses))
            )
            now = navigator.get_clock().now()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()

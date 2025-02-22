# HM4 RL 24/25 Arsen Hudyma

Hi, this is the repository for the HM4 of the course 'Robotics Lab'. In this final homework we will see a new mobile robot and we'll use the Nav2 functionalities, also with their Simple Commander API, but also the SLAM algoritm and more.
To have a complete vision of what is done here and why check as always the ppt file, that contains also many videos of the functionalities tests ;)!
But because the ppt has many videos this time I can't upload it so I'll share with you the link to visualize it:
```bash
https://www.canva.com/design/DAGdsNvesFQ/IR5iDEaJDHlEZPiQIk3t6A/view?utm_content=DAGdsNvesFQ&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=h0b19a66833
```

The pages you need to check for this project are from the 96 to the end.

To start, you'll need this repository on your computer, so get it by:
```bash
git clone https://github.com/arsen-hdm/RL_HM4.git
```

Then, once you're in the dockek container, firstly:
```bash
colcon build
. install/setup.bash
```

### Simulation in gazebo
The first point of this HW ask us to work in gazebo, and in detail to change the spawning pose of the robot, add and move and obstacle and also to add an aruco tag, to visualize the results start the gazebo simulation by:
```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```

### Making the robot move
After the robot is spawned it gazebo we don't want simply to watch it, we want it to move, so you'll need to run this command that set up all the Nav2 thing that we need:
```bash
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```

Then, to start sending the commands you have to run this:
```bash
ros2 run rl_fra2mo_description follow_waypoints.py
```

Remember that the desired coordinates are in the config directory in a yaml file, you'll recognize it. And also remember that in the follow_waypoints file the desired goal are switched in the first section, so modify it appropriately to have the movement that you expect ;)

### Mapping the environment
You can for sure explore your map and after that memorize the map, you have only to chose how do you want to do it! You may start the gazebo environmeng and launch the explore py file with the follow way point, like done before in this readme file. You may go inside the explore launch file and activate the explore node, which autonomously explores the environment (so you don't need to start the follow way point). Or you can teleop the robot manually trought the map and explore everything until you have all the map completed! (For sure this solution may be used when the map is small and closed, otherwhise this way isn't possible.

The command to teleop the robot is this (remember that you don't have to launch the explore node in the explore launch file):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

And when you're happy with the obtained map you can save it by this command:
```bash
ros2 run nav2_map_server map_saver_cli -f map
```
where map is the name of the png output file (remember to add and cd to a maps folder inside your package).

For now it'all but soon I'll add the other part of this HW. See you soon ;)

Thanks for the attention, you can find also other homeworks in my personal repositories!

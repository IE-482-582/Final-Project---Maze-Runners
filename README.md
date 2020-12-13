

# Maze Runners
---
#### Project Names - rbx1
---
#### Team Members
- Ruthvik Balakrishna Jois (50314910)
- Balaji Shubham Kandhi (50314942)

## Project Description
The maze runners project consists of 2 robots whose objective is to pass through a maze using the voice commands. The robots are fully voice controlled and has to pass through the obstacle course without hitting each other or with the walls. In other words, it also contains collision avoidance. The robots are controlled by a single operator who will command the robots by name and the robots will move simultaneously on his/her command.
*NOTE (1) - The commands will be quite rudimentary like forward, backward, left, right and stop. There are no hard and fast course instructions for the maze, just try to avoid the barriers and reach the target point as quickly as possible.*
*NOTE (2) - You need your hardware for this project which is your Microphone Device. USB Mic is highly preferred but wired headphones with AUX jack will give decent results.*

## Gazebo World and Multiple Robots in the World
Our custom world(or maze) is created using the custom_world.world file which is mentioned in our main.launch file which is the main launch file that we use to start the gazebo. In the file there are two supplementary files mentioned robots.launch and one_launch.launch. These two provide necessary arguments to the robot and to our python code. They are basically the hardware of the project. In the code you can edit the initial pose and experiment with various orientation of the robots. Also, using the custom_world_coords.csv file you can edit the Gazebo world as instructed below. Below is the maze that we are using in the project.

 ![Gazebo_Maze (1)](https://user-images.githubusercontent.com/70620113/102002900-21ecdc80-3ccf-11eb-9e3d-92143b726691.png)

## Voice-to-text speech recognizer 
The rbx1_speech file contains a voice_nav_commands.launch file that opens a voice recognizer. In this recognizer the voice commands are converted into text commands and given to the python code which python code interprets as command or none. The pronunciation need to be specific otherwise the recognizer won't be able to distinguish that world in its library and will give void response. To increase the vocabulary, you need to add your words in a text file and convert that file into .lm and .dic files in (4) link.

 ![Terminal](https://user-images.githubusercontent.com/70620113/102007602-94be7d80-3cf8-11eb-8176-8bd8328df8d2.png)

## Python Code
The logic in the Python code is very basic and easy to understand. It will receive the words that are given by the voice recognizer and process it as the command. This python code then will give instructions to move the robot in Gazebo and also avoid hitting the walls or each other. Also, the specific that are in the python code are attached below. It also gives the distance between obstacle and itself. To make it easy for user we have named our two robots sam and max,(Sam is Robot 1 and Max is Robot 2) as the names are easy to pronounce and the robots can easily differentiate between the commands.

 ![voice commands](https://user-images.githubusercontent.com/70620113/102007570-4e691e80-3cf8-11eb-9bc3-70bc607ad0eb.png)

#### RQT_Graph
ALso, below is the rqt_graph that will help you understand various topics, nodes and messages in the system. It shows the mechanism of our project.

 ![rqt_graph](https://user-images.githubusercontent.com/70620113/102007535-eca8b480-3cf7-11eb-8e74-2b75c8509b69.png)


---
## Installation Instructions
## Creating the Multi-robot Worlds & giving voice command for Gazebo
### Setup the rbx1
---

- Grab the current github repo and download it. Open a new terminal window.

```
  cd ~/Downloads
  rm -rf Final-Project---Maze-Runners
  git clone https://github.com/IE-482-582/Final-Project---Maze-Runners/rbx1
```

```
  cd ~/catkin_ws/src/
  catkin_create_pkg rbx1 rospy geometry_msgs sensor_msgs
```

- Copy all the files in rbx1 from Downloads to catkin_ws/src/rbx1

```
  cd ~/catkin_ws/
  catkin_make
```

---
## Adding Voice commands to the turtlebots by using Pocketsphinx


- Open a new terminal window. First install pocketsphinx package. 
```
  sudo apt-get install ros-indigo-pocketsphinx
  sudo apt-get install gstreamer0.10-gconf
```

---
## Setting up the Voice Commands

- Copy all the files in rbx1 from Downloads to catkin_ws/src/rbx1/

- Run the installation script. 
```
  cd ~/catkin_ws/src/rbx1/rbx1_speech/nodes/
  chmod +x *.py
```
- Compile/make our package
```
  cd ~/catkin_ws
  catkin_make
```
---

## Creating a Customized Maze

- These instructions will explain how to generate a maze composed of numerous unit cubes (1x1x1 meter blocks).

*NOTE: The files mentioned below are contained in ~/catkin_ws/src/robot_world/launch.*

-  Make a backup copy of these files, just in case something goes wrong:
```
  cd ~/catkin_ws/src/rbx1/robot_world/launch
  cp custom_world_coords.xls custom_world_coords_BACKUP.xls
  cp custom_world_coords.csv custom_world_coords_BACKUP.csv
  cp custom_world.world custom_world_BACKUP.world
```

-  Edit the ``` custom_world_coords.xls ``` file in Libre Office Calc.

-  Save as ``` custom_world_coords.csv ``` (export as ```.csv``` file):

	- File --> Save As...
  	- Choose file type = "Text CSC (.csv)"
	- Name the file ```custom_world_coords.csv```
	- Save
	
-  Run the python script to generate a .world file:
- Open a new terminal role.
```
  cd ~/catkin_ws/src/rbx1/robot_world/launch
  python create_world.py
```
This will create a file named ```custom_world.world.```

#### Test out your new world:
- Open a terminal window.

```
  cd ~/catkin_ws/src/rbx1/robot_world/launch
  roslaunch robot_world main.launch
```
*NOTE - Crosscheck whether in main.launch file custom_world.world is properly mentioned there or not.*

---

## Running the Voice Recognizer.


- Open a new terminal window. For launching the speech recognizer.

```
  cd ~/catkin_ws/src/rbx1/rbx1_speech/launch/
  roslaunch rbx1_speech voice_nav_commands.launch
```
*NOTE- This will launch the speech recognizer where you could test the vocabulary and your system's microphone. You need to use USB mic for best results.*

---
## Running the Voice Command.

- Open a 2nd terminal window. For launching the voice command.

```
  cd ~/catkin_ws/src/rbx1/rbx1_speech/nodes/
  rosrun rbx1_speech voice_nav.py
```
*NOTE -  To add vocabulary, edit the .txt file and upload it on the (4) link given in the reference. This will create the .lm, .dic. Copy those to **catkin_ws/src/rbx1/rbx1_speech/config/** and save it.*

- Open a 3rd terminal window. Open a Gazebo to check whether your robots are implementing the voice commands are not. 

```
  cd ~/catkin_ws/src/rbx1/robot_world/launch/
  roslaunch robot_world main.launch

```
*NOTE: We are now using a customized .launch file.*

---
## Running a rqt_graph

- Open a 4th terminal window. This will allow you to check whether your nodes are actually publishing and subscribing to topics.

```
cd ~/catkin_ws/
rosrun rqt_graph rqt_graph
```
*NOTE - You can compare your rqt_graph with the rqt_graph attched above to see whether it is working correctly or not.*

---

## Measure Of Success


| Measure of Success (from your PROPOSAL) | Status (completion percentage) |
|:---------------------------------------------------------|:----------------------------------------- |
| View maze world in simulator| 100%|
| Whether the two robots are moving.| 100%|
|Whether the two robots are controlled simulatneously.|100%|
| Whether the two robots are following collision avoidance or not. | 100%|
| Whether the robots are following the voice commands or not. | 100%|
| Have a classmate follow the steps in the README to successfully run the simulation without any help.| 100% |

---

## References
1. https://www.theconstructsim.com/ros-qa-130-how-to-launch-multiple-robots-in-gazebo-simulator/: For launching multi robots in the world.
2. https://edu.gaitech.hk/turtlebot/speech-doc.html : For using text-to-speech recognition
3. https://duluthrobot.wordpress.com/2016/03/18/adding-voice-commands-using-pocketsphinx-to-the-turtlebot2/ :For giving the voice commands to robots
4. http://www.speech.cs.cmu.edu/tools/lmtool-new.html : For making our own library of commands
5. https://www.theconstructsim.com/exploring-ros-2-wheeled-robot-part-5/ : For giving the collision avoidance.
---




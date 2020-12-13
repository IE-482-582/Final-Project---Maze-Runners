

# Maze Runner
---
#### Project Names - multi_robot & rbx1
---
#### Team Members
- Ruthvik Balakrishna Jois (50314910)
- Balaji Shubham Kandhi (50314942)

## Project Objective
The maze runner project consists of 2 robot which passes through a maze using the voice commands. The robots will be fully voice controlled and has to pass through the obstacle course without hitting each other or with the walls. In other words, it will contain collision avoidance. The robots will be controlled by a single operator who will command the robots by name and the robots will move simultaneously on his/her command.

---
## Creating the Multi-robot Worlds & giving voice command for Gazebo
### Setup the rbx1
---

- Grab the current github repo and download it. Open a new terminal window.

```
  cd ~/Downloads
  rm -rf Final-Project---Maze-Runner
  git clone https://github.com/IE-482-582/Final-Project---Maze-Runner/rbx1
```

```
  cd ~/catkin_ws/src/
  catkin_create_pkg rbx1 rospy geometry_msgs sensor_msgs
```

- Copy all the files in multi_robot from Downloads to catkin_ws/src/multi_robot

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

![Gazebo_Maze (1)](https://user-images.githubusercontent.com/70620113/102002900-21ecdc80-3ccf-11eb-9e3d-92143b726691.png)

*NOTE: The files mentioned below are contained in ~/catkin_ws/src/multi_robot/launch.*

-  Make a backup copy of these files, just in case something goes wrong:
```
  cd ~/catkin_ws/src/rbx1/multirobot/launch
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
  cd ~/catkin_ws/src/rbx1/multi_robot/launch
  python create_world.py
```
This will create a file named ```custom_world.world.```

#### Test out your new world:
- Open a terminal window.

```
  cd ~/catkin_ws/src/rbx1/multi_robot/launch
  roslaunch multi_robot main.launch
```
*NOTE - Crosscheck whether in main.launch file cutom_world.world is there*

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

- Open a new terminal window. For launching the voice command.

```
  cd ~/catkin_ws/src/rbx1/rbx1_speech/nodes/
  rosrun rbx1_speech voice_nav.py
```
*NOTE -  To add vocabulary, edit the .txt file and upload it on the (4) link given in the reference. This will create the .lm, .dic. Copy those to **catkin_ws/src/rbx1/rbx1_speech/config/** and save it.*

- Open a new terminal window. Open a Gazebo to check whether your robots are implementing the voice commands are not. 

```
  cd ~/catkin_ws/src/rbx1/multi_robot/launch/
  roslaunch multi_robot main.launch

```
*NOTE: We are now using a customized .launch file.*

---
## References
1. https://www.theconstructsim.com/ros-qa-130-how-to-launch-multiple-robots-in-gazebo-simulator/: For launching multi robots in the world.
2. https://edu.gaitech.hk/turtlebot/speech-doc.html : For using text-to-speech recognition
3. https://duluthrobot.wordpress.com/2016/03/18/adding-voice-commands-using-pocketsphinx-to-the-turtlebot2/ :For giving the voice commands to robots
4. http://www.speech.cs.cmu.edu/tools/lmtool-new.html : For making our own library of commands
5. https://www.theconstructsim.com/exploring-ros-2-wheeled-robot-part-5/ : For giving the collision avoidance.
---




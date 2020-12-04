

# Maze Runner
---
#### Project Names - multi_robot & rbx1
---
#### Team Members
- Ruthvik Balakrishna Jois (50314910)
- Balaji Shubham Kandhi (50314942)

---
## Creating the Multi-robot Worlds/Mazes for Gazebo
### Setup the Multi_Robot.......................(1)
---

- Grab the current github repo and download it. Open a new terminal window.

```
cd ~/Downloads
rm -rf Final-Project---Maze-Runner
git clone https://github.com/IE-482-582/Final-Project---Maze-Runner/multi_robot
```

```
cd ~/catkin_ws/src/
catkin_create_pkg multi_robot rospy geometry_msgs sensor_msgs
```

- Copy all the files in multi_robot from Downloads to catkin_ws/src/multi_robot

```
cd ~/catkin_ws/
catkin_make
```
---
## Creating a Customized Maze

- These instructions will explain how to generate a maze composed of numerous unit cubes (1x1x1 meter blocks).

- Below link is our gazebo world used in this project.
**https://drive.google.com/file/d/1UGo7wGABH8IfnUXu776eiEHLr4fnbnS6/view?usp=sharing**

*NOTE: The files mentioned below are contained in ~/catkin_ws/src/multi_robot/launch.*

-  Make a backup copy of these files, just in case something goes wrong:
```
cd ~/catkin_ws/src/multirobot/launch
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
cd ~/catkin_ws/src/multi_robot/launch
python create_world.py
```
This will create a file named ```custom_world.world.```

#### Test out your new world:
- Open a terminal window.

```
cd ~/catkin_ws/src/multi_robot/launch
roslaunch multi_robot main.launch
```
*NOTE - Crosscheck whether in main.launch file cutom_world.world is there*

---
## Adding Voice commands to the turtlebots by using Pocketsphinx................(2)


- First install pocketsphinx package. Open a new terminal window.
```
sudo apt-get install ros-indigo-pocketsphinx
sudo apt-get install gstreamer0.10-gconf
```

---
## Setting up the Voice Commands................(3)


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
## Running the Voice Recognizer.


- For launching the speech recognizer, open a new terminal window.

```
cd ~/catkin_ws/src/rbx1/rbx1_speech/launch/
roslaunch rbx1_speech voice_nav_commands.launch
```
*NOTE- This will launch the speech recognizer where you could test the vocabulary and your system's microphone. You need to use USB mic for best results.*

---
## Running the Voice Command.

- For launching the voice command, open a 2nd terminal window.

```
cd ~/catkin_ws/src/rbx1/rbx1_speech/nodes/
rosrun rbx1_speech voice_nav.py
```
*NOTE -  To add vocabulary, edit the .txt file and upload it on the (4) link given in the reference. This will create the .lm, .dic. Copy those to **catkin_ws/src/rbx1/rbx1_speech/config/** and save it.

- Open a Gazebo to check whether your robots are implementing the voice commands are not. For that open a 3rd terminal window.

```
cd ~/catkin_ws/src/multi_robot/launch/
roslaunch multi_robot main.launch

```
*NOTE: We are now using a customized .launch file.*

---
## References
1. https://www.theconstructsim.com/ros-qa-130-how-to-launch-multiple-robots-in-gazebo-simulator/: For launching multi robots in the world.
2. https://edu.gaitech.hk/turtlebot/speech-doc.html : For using text-to-speech recognition
3. https://duluthrobot.wordpress.com/2016/03/18/adding-voice-commands-using-pocketsphinx-to-the-turtlebot2/ :For giving the voice commands to robots
4. http://www.speech.cs.cmu.edu/tools/lmtool-new.html : For making our own library of commands
---



## Organizing your Repository
For consistency, please use the directory structure described below, where `projectname` should be replaced with the actual catkin_ws name of your project.
	
```
PROPOSAL.md
README.md
Images/	
code/projectname/	
	scripts/
	msg/
	srv/
	CMakeLists.txt
	package.xml
```		

- A sample README file [may be found here](README_template.md)
- `Images/` is a directory (folder) for storing the graphics for your README.
- `code/projectname/` is a directory for your ROS code.  Replace `projectname` with the name of your catkin package.
	- Within this directory you should have `CMakeLists.txt`, `package.xml`, a `scripts/` directory, most likely a `msg/` directory, and possibly a `srv/` directory (if your project uses services).  
- See `06_Followbot` for an example of the directory structure.

## Project Grading

Grades for the final project will be based on the following percentages and content:

- Proposal (15%)
- Progress Report (10%)
- Final Documentation and Code (50%)
	- Did you address issues from the presentation feedback?
	- How did you do on the "measures of success"?
	- Can the instructor successfully install the prereqs?
	- Can the instructor successfully run the code?  (I highly recommend that you find someone to test this for you)
	- Does the code do what it's supposed to?
- Project Demonstration (25%)
	- Did you prepare/rehearse for this presentation?
	- Is the README neatly formatted?
	- Is the README (nearly) complete?
	- Was the code submitted/organized properly?  Are filenames correct?  Code in the proper directories/subdirectories?
	- Are the installation instructions complete?
	- Are the instructions for running the code complete?
	- Were you able to answer technical questions about your project?
	- How well were you able to demonstrate the actual implementation?  Note: You have until Monday, Dec. 14 to finalize the project.

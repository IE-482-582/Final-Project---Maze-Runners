# Final Project

- This repo is where you will store **all** of your documents for the course project.
- Please see [PROPOSAL.md](PROPOSAL.md) for a template for your **proposal** (Due Nov. 2).

---
# Maze Runner
## Creating the Multi-robot Worlds/Mazes for Gazebo
### Setup the Multi_Robot

- Grab the current github repo

```
cd ~/Downloads
rm -rf Final-Project---Maze-Runner
git clone https://github.com/IE-482-582/Final-Project---Maze-Runner/multi_robot
```

- Run the installation script
```
cd ~/Downloads/Final-Project---Maze-Runner/multi_robot
chmod +x install_multi_robot.sh
./install_multi_robot.sh

```
This will create a new package named multi_robot in your ~/catkin_ws/src/ directory.

## Creating a Customized Maze
These instructions will explain how to generate a maze composed of numerous unit cubes (1x1x1 meter blocks).

https://drive.google.com/file/d/1UGo7wGABH8IfnUXu776eiEHLr4fnbnS6/view?usp=sharing

NOTE: The files mentioned below are contained in ~/catkin_ws/src/multi_robot/launch.

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

```
cd ~/catkin_ws/src/multi_robot/launch
python create_world.py
```
This will create a file named ```custom_world.world.```

- Test out your new world:
```
cd ~/catkin_ws/src/multi_robot/launch
roslaunch multi_robot main.launch
```
## Adding Voice commands to the turtlebots by using Pocketsphinx

- First install pocketsphinx package:
```
sudo apt-get install ros-indigo-pocketsphinx
sudo apt-get install gstreamer0.10-gconf
```
## Setting up the Voice Commands

- Grab the current github repo

```
cd ~/Downloads
rm -rf Final-Project---Maze-Runner
git clone https://github.com/IE-482-582/Final-Project---Maze-Runner/rbx1
```

- Run the installation script
```
cd ~/catkin_ws/src/rbx1/rbx1_speech/nodes
chmod +x *.py
```
- Compile/make our package
```
cd ~/catkin_ws
catkin_make
```




## IMPORTANT DATES:
- **Friday, Oct. 30, 11:30am** -- Proposal presentations in class.
- **Monday, Nov. 2, 5:00pm** -- Be sure your repo has your final [PROPOSAL.md](PROPOSAL.md) uploaded.  This will be modified according to the feedback you received on Oct. 30 in class.
- **Friday, Nov. 20** -- Progress Report.  Each team will present the status of their project in class that day.
- **Friday, Dec. 4** -- Your **almost** final documentation, code, and presentation materials are due.  You'll be asked to give a brief presentation in class.  I'll give you feedback.
- **Exact Date to-be-determined** -- **Final** project presentations.  We'll either do these in class, or you'll produce a YouTube video.  We'll discuss in early December.
- **Monday, Dec. 14, Noon** -- Your complete project materials are due.


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


---

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

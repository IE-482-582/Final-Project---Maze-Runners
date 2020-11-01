--- 

# {Maze Runner}

Team Members:
- {Ruthvik Balakrishna Jois}, {ruthvikj@buffalo.edu}
- {Balaji Shubham Kandhi}, {bkandhi@buffalo.edu}

--- 

## Project Objective
{The maze runner project consists of 2 robot which passes through a maze using the voice commands. The robots will be fully voice controlled and has to pass through the obstacle course without hitting each other or with the walls. In other words, it will contain collision avoidance. The robots will be controlled by 2 different operators that will start the race at the same time and the one to get across first will be the winner.
[Maze.jpeg](https://drive.google.com/file/d/1gpX-vfYERUfnZOQ7RMVxCLrq_NMaKKzL/view?usp=sharing) }


## Contributions
{This project will be really interesting game in which the 2 robots' user will compete each other using their voice command to get across the line first. The voice controlled robots with collison avoidance programmed in them and the obstacle being kinetic will make for a really good visual representation.}


## Project Plan
{While controlling the robot should turn 90 degrees right or left according to the controller's wish. By using the blathering library of the python software, we give the voice commands to the robots. The maze will be be designed using the world maker, excel or a 2D CAD software (like AutoCAD or Solidworks). The race course will also contain moving obstacles which the operators have to look out for while controlling their respectiver robots. The robot can be set in gazebo, stage ROS or rviz simulator for a great 2D or 3D view.  This will be create a great gaming experience for the users.}


## Milestones/Schedule Checklist
{What are the tasks that you need to complete?  Who is going to do them?  When will they be completed?}
- [x] Complete this proposal document.  *Due Nov. 2*
- [ ] {add tasks here}
- [ ] Create progress report.  *Due Nov. 20*
- [ ] {add more tasks here}
- [ ] Create final presentation.  *Due Dec. 4*
- [ ] {you might have some more tasks here}
- [ ] Provide system documentation (README.md).  *Due Dec. 14*


## Measures of Success
{How will you know you succeeded?  If you were to receive partial credit, what should we look for?}


---
**A Sample Proposal Appears Below**
---

# Creating a Gazebo Model of the Duckiebot

Team Members:
- Chase Murray, cmurray3@buffalo.edu
- Jane Student, j@buffalo.edu


## Project Objective
The goal of this project is to create a Gazebo model of the Duckiebot. This model will accurately reflect the dimensions of the Duckiebot, will include the Duckiebot's sensors (a fisheye lens camera and a magnetometer), and will have the same drive train (two motors controlling the two motorized wheels).


## Contributions
There are currently no Gazebo models of this robot.  By creating such a model, we will be able to test control algorithms in a simulated environment (without the need for the physical robot itself).  However, after training the control algorithms in Gazebo, it will be easy to execute them on a real Duckiebot, since the simulated version will be an accurate representation.


## Project Plan
The textbook contains two chapters (Chapters 15--17) that describe how to build a custom robot.
However, these chapters do not discuss the use of a fisheye lens.  We will use the ros.org Website to learn how to model such cameras.
We will also consult the Duckiebot specs to determine the dimensions and weight of the robot, as well as the capabilities of the motors.


## Milestones/Schedule Checklist
- [x] Complete this proposal document.  *Due Nov. 2*
- [ ] Capture the specs of the actual/physical robot.  *JS, Nov. 13*
- [ ] Build a sample model using the textbook examples. *CM, Nov. 13*
- [ ] Modify the sample model to match the specs of the Duckiebot.  *CM, Nov. 17*
- [ ] Add a fisheye lens camera. *JS, Nov. 18*
- [ ] Create progress report.  *Due Nov. 20*
- [ ] Create Gazebo .launch files to test the robot.  *CM, Dec. 1*
- [ ] Create a simple controller to test the interaction with the robot. *JS, Dec. 3*
- [ ] Create final presentation.  *Due Dec. 4*
- [ ] Update documentation based on presentation feedback. *CM, Dec. 7*
- [ ] Provide system documentation (README.md).  *Due Dec. 14*


## Measures of Success
- [ ] View robot model in Gazebo.
- [ ] Demonstrate that the fisheye lens camera is appropriately distorted.
- [ ] Demonstrate that robot moves when given commands.
- [ ] Implement code on a real Duckiebot.
- [ ] Have a classmate follow the steps in the README to successfully run the simulation without any help.



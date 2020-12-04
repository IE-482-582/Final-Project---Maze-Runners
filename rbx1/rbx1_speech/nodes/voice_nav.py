#!/usr/bin/env python

"""
  voice_nav.py - Version 1.1 2013-12-20
  
  Allows controlling a mobile base using simple speech commands.
  
  Based on the voice_cmd_vel.py script by Michael Ferguson in
  the pocketsphinx ROS package.
  
  See http://www.ros.org/wiki/pocketsphinx
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign

class VoiceNav:
    def __init__(self):
        rospy.init_node('voice_nav')
        
        rospy.on_shutdown(self.cleanup)
        
        # Set a number of parameters affecting the robot's speed
        self.max_speed = rospy.get_param("~max_speed", 0.4)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.5)
        self.speed = rospy.get_param("~start_speed", 0.1)
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.5)
        self.linear_increment = rospy.get_param("~linear_increment", 0.05)
        self.angular_increment = rospy.get_param("~angular_increment", 0.4)
        
        # We don't have to run the script very fast
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)
        
        # A flag to determine whether or not voice control is paused
        self.paused = False
        
        # Initialize the Twist message we will publish.
        self.cmd_vel1 = Twist()
        self.cmd_vel2 = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub1 = rospy.Publisher('/robot1/cmd_vel_mux/input/navi', Twist, queue_size=5)
        self.cmd_vel_pub2 = rospy.Publisher('/robot2/cmd_vel_mux/input/navi', Twist, queue_size=5)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speech_callback)
        
       #Robot 1 commands
        # A mapping from keywords or phrases to commands
        self.keywords_to_command1 = {'sam stop': ['sam stop', 'sam halt', 'sam abort', 'sam kill', 'sam panic', 'sam off', 'sam freeze'],#'shut down', 'turn off', 'help', 'help me'],
                                    #'slower': ['slow down', 'slower'],
                                     'sam faster': ['sam speed up', 'sam faster', 'sam increase', 'sam fast'],
                                     'sam forward': ['forward', 'sam ahead', 'sam straight'],
                                     'sam backward': ['sam back', 'backward', 'sam back up'],
                                    #'rotate left': [' sam rotate left', 'left'],
                                    #'rotate right': [' sam rotate right', 'right'],
                                     'sam turn left': ['sam turn left', 'sam left'],
                                     'sam turn right': ['sam turn right', 'sam right'],
                                    #'quarter': ['quarter speed'],
                                    #'half': ['half speed'],
                                     'sam full': ['sam full speed', 'sam full']}
                                    #'pause': ['pause speech'],
                                    #'continue': ['continue speech']}
                                    
		#Robot 2 commands
		 # A mapping from keywords or phrases to commands
        self.keywords_to_command2 = {'max stop': ['max stop', 'max halt', 'max abort', 'max kill', 'max panic', 'max off', 'max freeze'], #'shut down', 'turn off', 'help', 'help me'],
                                    #'slower': ['slow down', 'slower'],
                                     'max faster': ['max speed up', 'max faster', 'max increase', 'max fast'],
                                     'max forward': ['max forward', 'max ahead', 'straight'],
                                     'max backward': ['max back', 'max backward', 'max back up'],
                                    #'rotate left': [' max rotate left', 'left'],
                                    #'rotate right': [' max rotate right', 'right'],
                                     'max turn left': ['max turn left', 'max left'],
                                     'max turn right': ['max turn right', 'max right'],
                                    #'quarter': ['quarter speed'],
                                    #'half': ['half speed'],
                                     'max full': ['max full speed', 'max full']}
                                    #'pause': ['pause speech'],
                                    #'continue': ['continue speech']}
        
        rospy.loginfo("Ready to receive voice commands")
        
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.cmd_vel_pub1.publish(self.cmd_vel1)
            r.sleep()                       
            self.cmd_vel_pub2.publish(self.cmd_vel2)
            r.sleep() 
            
    def get_command(self, data):
        # Attempt to match the recognized word or phrase to the 
        # keywords_to_command dictionary and return the appropriate
        # command
        for (command, keywords) in self.keywords_to_command1.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
                    
		for (command, keywords) in self.keywords_to_command2.iteritems():
			for word in keywords:
				if data.find(word) > -1:
					return command
        
    def speech_callback(self, msg):
        # Get the motion command from the recognized phrase
        command = self.get_command(msg.data)
        
        # Log the command to the screen
        rospy.loginfo("Command: " + str(command))
        
        # If the user has asked to pause/continue voice control,
        # set the flag accordingly 
        if command == 'pause':
            self.paused = True
        elif command == 'continue':
            self.paused = False
        
        # If voice control is paused, simply return without
        # performing any action
        if self.paused:
            return       
        
        #Robot 1 commands
        # The list of if-then statements should be fairly
        # self-explanatory
        if command == 'sam forward':    
            self.cmd_vel1.linear.x = self.speed
            self.cmd_vel1.angular.z = 0
            
        elif command == 'rotate left':
            self.cmd_vel1.linear.x = 0
            self.cmd_vel1.angular.z = self.angular_speed
                
        elif command == 'rotate right':  
            self.cmd_vel1.linear.x = 0      
            self.cmd_vel1.angular.z = -self.angular_speed
            
        elif command == 'turn left':
            if self.cmd_vel1.linear.x != 0:
                self.cmd_vel1.angular.z += self.angular_increment
            else:        
                self.cmd_vel1.angular.z = self.angular_speed
                
        elif command == 'turn right':    
            if self.cmd_vel1.linear.x != 0:
                self.cmd_vel1.angular.z -= self.angular_increment
            else:        
                self.cmd_vel1.angular.z = -self.angular_speed
                
        elif command == 'sam backward':
            self.cmd_vel1.linear.x = -self.speed
            self.cmd_vel1.angular.z = 0
            
        elif command == 'stop': 
            # Stop the robot!  Publish a Twist message consisting of all zeros.         
            self.cmd_vel = Twist()
        
        elif command == 'faster':
            self.speed += self.linear_increment
            self.angular_speed += self.angular_increment
            if self.cmd_vel1.linear.x != 0:
                self.cmd_vel1.linear.x += copysign(self.linear_increment, self.cmd_vel1.linear.x)
            if self.cmd_vel1.angular.z != 0:
                self.cmd_vel1.angular.z += copysign(self.angular_increment, self.cmd_vel1.angular.z)
            
        elif command == 'slower':
            self.speed -= self.linear_increment
            self.angular_speed -= self.angular_increment
            if self.cmd_vel1.linear.x != 0:
                self.cmd_vel1.linear.x -= copysign(self.linear_increment, self.cmd_vel1.linear.x)
            if self.cmd_vel1.angular.z != 0:
                self.cmd_vel1.angular.z -= copysign(self.angular_increment, self.cmd_vel1.angular.z)
                
        elif command in ['quarter', 'half', 'full']:
            if command == 'quarter':
                self.speed = copysign(self.max_speed / 4, self.speed)
        
            elif command == 'half':
                self.speed = copysign(self.max_speed / 2, self.speed)
            
            elif command == 'full':
                self.speed = copysign(self.max_speed, self.speed)
            
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x = copysign(self.speed, self.cmd_vel.linear.x)

            if self.cmd_vel.angular.z != 0:
                self.cmd_vel1.angular.z = copysign(self.angular_speed, self.cmd_vel.angular.z)
                
        else:
            return

        self.cmd_vel1.linear.x = min(self.max_speed, max(-self.max_speed, self.cmd_vel1.linear.x))
        self.cmd_vel1.angular.z = min(self.max_angular_speed, max(-self.max_angular_speed, self.cmd_vel1.angular.z))
        
        
        #Robot 2
        # The list of if-then statements should be fairly
        # self-explanatory
        if command == 'max forward':    
            self.cmd_vel2.linear.x = self.speed
            self.cmd_vel2.angular.z = 0
            
        elif command == 'rotate left':
            self.cmd_vel2.linear.x = 0
            self.cmd_vel2.angular.z = self.angular_speed
                
        elif command == 'rotate right':  
            self.cmd_vel2.linear.x = 0      
            self.cmd_vel2.angular.z = -self.angular_speed
            
        elif command == 'turn left':
            if self.cmd_vel2.linear.x != 0:
                self.cmd_vel2.angular.z += self.angular_increment
            else:        
                self.cmd_vel2.angular.z = self.angular_speed
                
        elif command == 'turn right':    
            if self.cmd_vel2.linear.x != 0:
                self.cmd_vel2.angular.z -= self.angular_increment
            else:        
                self.cmd_vel2.angular.z = -self.angular_speed
                
        elif command == 'max backward':
            self.cmd_vel2.linear.x = -self.speed
            self.cmd_vel2.angular.z = 0
            
        elif command == 'stop': 
            # Stop the robot!  Publish a Twist message consisting of all zeros.         
            self.cmd_vel2 = Twist()
        
        elif command == 'faster':
            self.speed += self.linear_increment
            self.angular_speed += self.angular_increment
            if self.cmd_vel2.linear.x != 0:
                self.cmd_vel2.linear.x += copysign(self.linear_increment, self.cmd_vel2.linear.x)
            if self.cmd_vel2.angular.z != 0:
                self.cmd_vel2.angular.z += copysign(self.angular_increment, self.cmd_vel2.angular.z)
            
        elif command == 'slower':
            self.speed -= self.linear_increment
            self.angular_speed -= self.angular_increment
            if self.cmd_vel2.linear.x != 0:
                self.cmd_vel2.linear.x -= copysign(self.linear_increment, self.cmd_vel2.linear.x)
            if self.cmd_vel2.angular.z != 0:
                self.cmd_vel2.angular.z -= copysign(self.angular_increment, self.cmd_vel2.angular.z)
                
        elif command in ['quarter', 'half', 'full']:
            if command == 'quarter':
                self.speed = copysign(self.max_speed / 4, self.speed)
        
            elif command == 'half':
                self.speed = copysign(self.max_speed / 2, self.speed)
            
            elif command == 'full':
                self.speed = copysign(self.max_speed, self.speed)
            
            if self.cmd_vel2.linear.x != 0:
                self.cmd_vel2.linear.x = copysign(self.speed, self.cmd_vel2.linear.x)

            if self.cmd_vel2.angular.z != 0:
                self.cmd_vel2.angular.z = copysign(self.angular_speed, self.cmd_vel2.angular.z)
                
        else:
            return

        self.cmd_vel2.linear.x = min(self.max_speed, max(-self.max_speed, self.cmd_vel2.linear.x))
        self.cmd_vel2.angular.z = min(self.max_angular_speed, max(-self.max_angular_speed, self.cmd_vel2.angular.z))

    def cleanup(self):
        # When shutting down be sure to stop the robot!
        twist = Twist()
        self.cmd_vel_pub1.publish(twist)
        rospy.sleep(1)
        self.cmd_vel_pub2.publish(twist)
        rospy.sleep(1)

if __name__=="__main__":
    try:
        VoiceNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice navigation terminated.")


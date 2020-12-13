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
from sensor_msgs.msg import LaserScan


class VoiceNav():	
       
    def __init__(self):
        rospy.init_node('voice_nav')
        
        rospy.on_shutdown(self.cleanup)
        
        # Set a number of parameters affecting the robot's speed
        self.max_speed = rospy.get_param("~max_speed", 1)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.5)
        self.speed = rospy.get_param("~start_speed", 0.5)
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.5)
        self.linear_increment = rospy.get_param("~linear_increment", 0.1)
        self.angular_increment = rospy.get_param("~angular_increment", 0.4)
        self.g_range_ahead_1 = 0
        self.g_range_ahead_2 = 0
        self.scan_sub1 = rospy.Subscriber('/robot1/scan', LaserScan, self.obstacle_detect1)
        self.scan_sub2 = rospy.Subscriber('/robot2/scan', LaserScan, self.obstacle_detect2)
               
        
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
        
       # Robot 1 commands             
        # A mapping from keywords or phrases to commands
        self.keywords_to_command1 = {'robo one stop': ['sam stop', 'sam halt', 'stop'], #'abort', 'kill', 'panic', 'off', 'freeze'], #'shut down', 'turn off', 'help', 'help me'],
                                    'robo one slower': ['sam slow down', 'sam slower', 'sam slow'],
                                    'robo one faster': ['sam speed up', 'sam faster'],
                                    'robo one forward': ['sam forward', 'sam ahead', 'sam straight', 'forward', 'one'],
                                    'robo one backward': ['sam back', 'sam backward', 'sam back up', 'backward'],
                                    #'robo one rotate left': ['sam rotate left'],
                                    #'robo one rotate right': ['sam rotate right'],
                                    'robo one turn left': ['sam turn left', 'sam left', 'left'],
                                    'robo one turn right': ['sam turn right', 'sam right', 'right']}
                                                                        		                               
		# Robot 2 commands
        # A mapping from keywords or phrases to commands
        self.keywords_to_command2 = {'robo two stop': ['max stop', 'max halt', 'max kill', 'halt', 'max kill'], #'abort', 'kill', 'panic', 'off', 'freeze'], #'shut down', 'turn off', 'help', 'help me'],
                                    'robo two slower': ['max slow down', 'max slower', 'max slow'],
                                    'robo two faster': ['max speed up', 'max faster'],
                                    'robo two forward': ['max forward', 'max ahead', 'max straight', 'straight', 'two'],
                                    'robo two backward': ['max back', 'max backward', 'max back up', 'behind'],
                                    #'robo one rotate left': ['max rotate left', 'max left'],
                                    #'robo one rotate right': ['max rotate right', 'max right'],
                                    'robo two turn left': ['max turn left', 'max left', 'l'],
                                    'robo two turn right': ['max turn right', 'max right', 'r'],
                                    'pause': ['pause speech'],
                                    'continue': ['continue speech']}
        
        rospy.loginfo("Ready to receive voice commands")
        
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.cmd_vel_pub1.publish(self.cmd_vel1)
            r.sleep() 
            self.cmd_vel_pub2.publish(self.cmd_vel2)
            r.sleep()
						
            
	#Collision Avoidance Robot 1     
    def obstacle_detect1(self, msg):
        self.g_range_ahead_1 = min(msg.ranges)
        print("grange_ahead_1", self.g_range_ahead_1)
        ranger1 = msg.ranges
        sec_1 = min(ranger1[0:159])
        sec_2 = min(ranger1[160:319])
        sec_3 = min(ranger1[320:479])
        sec_4 = min(ranger1[480:639])
        if self.g_range_ahead_1 < 0.8:
			self.cmd_vel1.linear.x = 0
			self.cmd_vel1.linear.z = 0.2
							
        if self.g_range_ahead_1 > 0.8:
			self.cmd_vel1.linear.z = 0
					

        if sec_1 <= 1 and sec_2 <= 1.5:
		   self.g_range_ahead_1 = 0.4 
        if sec_2 <= 1 and sec_3 <= 1.5:
		    self.g_range_ahead_1 = 0.5
        if sec_3 <= 1 and sec_4 <= 1.5:
		    self.g_range_ahead_1 = 0.5
        if sec_4 <= 1 and sec_1 <= 1.5:
		    self.g_range_ahead_1 = 0.5
        else:
		    self.g_range_ahead_1 = 1	# anything to start
        self.g_range_ahead_1 = 1
        
    #Collision Avoidance    
    def obstacle_detect2(self, msg):
        self.g_range_ahead_2 = min(msg.ranges)
        print("grange_ahead_2", self.g_range_ahead_2)
        ranger2 = msg.ranges
        sec_5 = min(ranger2[0:159])
        sec_6 = min(ranger2[160:319])
        sec_7 = min(ranger2[320:479])
        sec_8 = min(ranger2[480:639])
        if self.g_range_ahead_2 < 0.8:
			self.cmd_vel2.linear.x = 0
			self.cmd_vel2.linear.z =0.5
				
        if self.g_range_ahead_2 > 0.8:
			self.cmd_vel2.linear.z = 0
		

        if sec_5 <= 1 and sec_6 <= 1.5:
		   self.g_range_ahead_2 = 0.4 
        if sec_6 <= 1 and sec_7 <= 1.5:
		    self.g_range_ahead_2 = 0.5
        if sec_7 <= 1 and sec_8 <= 1.5:
		    self.g_range_ahead_2 = 0.5
        if sec_8 <= 1 and sec_5 <= 1.5:
		    self.g_range_ahead_2 = 0.5
        else:
		    self.g_range_ahead_2 = 1	# anything to start
        self.g_range_ahead_2 = 1
        
       
    def get_command(self, data):
        # Attempt to match the recognized word or phrase to the 
        # keywords_to_command dictionary and return the appropriate
        # command
        for (command, keywords) in self.keywords_to_command1.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
                    
		 # keywords_to_command dictionary and return the appropriate
        # command
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
	    
        # The list of if-then statements should be fairly
        # self-explanatory
        # Robot 1 commands
        if command == 'robo one forward':  	 												
			self.cmd_vel1.linear.x = self.speed
			self.cmd_vel1.angular.z = 0
            
        elif command == 'robo one rotate left':
            self.cmd_vel1.linear.x = 0
            self.cmd_vel1.angular.z = self.angular_speed
                
        elif command == 'robo one rotate right':  
            self.cmd_vel1.linear.x = 0      
            self.cmd_vel1.angular.z = -self.angular_speed
            
        elif command == 'robo one turn left':
            if self.cmd_vel1.linear.x != 0:
                self.cmd_vel1.angular.z += self.angular_increment
            else:        
                self.cmd_vel1.angular.z = self.angular_speed
                
        elif command == 'robo one turn right':    
            if self.cmd_vel1.linear.x != 0:
                self.cmd_vel1.angular.z -= self.angular_increment
            else:        
                self.cmd_vel1.angular.z = -self.angular_speed
                
        elif command == 'robo one backward':
            self.cmd_vel1.linear.x = -self.speed
            self.cmd_vel1.angular.z = 0
            
        elif command == 'robo one stop': 
            # Stop the robot!  Publish a Twist message consisting of all zeros.         
            self.cmd_vel1 = Twist()
        
        elif command == 'robo one faster':
            self.speed += self.linear_increment
            self.angular_speed += self.angular_increment
            if self.cmd_vel1.linear.x != 0:
                self.cmd_vel1.linear.x += copysign(self.linear_increment, self.cmd_vel1.linear.x)
            if self.cmd_vel1.angular.z != 0:
                self.cmd_vel1.angular.z += copysign(self.angular_increment, self.cmd_vel1.angular.z)
                
		     
        elif command == 'robo one slower':
            self.speed -= self.linear_increment
            self.angular_speed -= self.angular_increment
            if self.cmd_vel1.linear.x != 0:
                self.cmd_vel1.linear.x -= copysign(self.linear_increment, self.cmd_vel1.linear.x)
            if self.cmd_vel1.angular.z != 0:
                self.cmd_vel1.angular.z -= copysign(self.angular_increment, self.cmd_vel1.angular.z)
          
            
       
        # The list of if-then statements should be fairly
        # self-explanatory
        # Robot 2 commands
        if command == 'robo two forward':    
            self.cmd_vel2.linear.x = self.speed
            self.cmd_vel2.angular.z = 0
            
        elif command == 'robo two rotate left':
            self.cmd_vel2.linear.x = 0
            self.cmd_vel2.angular.z = self.angular_speed
                
        elif command == 'robo two rotate right':  
            self.cmd_vel2.linear.x = 0      
            self.cmd_vel2.angular.z = -self.angular_speed
            
        elif command == 'robo two turn left':
            if self.cmd_vel2.linear.x != 0:
                self.cmd_vel2.angular.z += self.angular_increment
            else:        
                self.cmd_vel2.angular.z = self.angular_speed
                
        elif command == 'robo two turn right':    
            if self.cmd_vel2.linear.x != 0:
                self.cmd_vel2.angular.z -= self.angular_increment
            else:        
                self.cmd_vel2.angular.z = -self.angular_speed
                
        elif command == 'robo two backward':
            self.cmd_vel2.linear.x = -self.speed
            self.cmd_vel2.angular.z = 0
            
        elif command == 'robo two stop': 
            # Stop the robot!  Publish a Twist message consisting of all zeros.         
            self.cmd_vel2 = Twist()
        
        elif command == 'robo two faster':
            self.speed += self.linear_increment
            self.angular_speed += self.angular_increment
            if self.cmd_vel2.linear.x != 0:
                self.cmd_vel2.linear.x += copysign(self.linear_increment, self.cmd_vel2.linear.x)
            if self.cmd_vel2.angular.z != 0:
                self.cmd_vel2.angular.z += copysign(self.angular_increment, self.cmd_vel2.angular.z)
            
        elif command == 'robo two slower':
            self.speed -= self.linear_increment
            self.angular_speed -= self.angular_increment
            if self.cmd_vel2.linear.x != 0:
                self.cmd_vel2.linear.x -= copysign(self.linear_increment, self.cmd_vel2.linear.x)
            if self.cmd_vel2.angular.z != 0:
                self.cmd_vel2.angular.z -= copysign(self.angular_increment, self.cmd_vel2.angular.z)         
                          
        else:
            return
				
        self.cmd_vel1.linear.x = min(self.max_speed, max(-self.max_speed, self.cmd_vel1.linear.x))
        self.cmd_vel1.angular.z = min(self.max_angular_speed, max(-self.max_angular_speed, self.cmd_vel1.angular.z))
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


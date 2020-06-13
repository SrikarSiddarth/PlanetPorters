#!/usr/bin/env python

'''
* Team Name : PlanetPorters
* File Name : offBoardControl_final2.py
* Theme     : Docking the probe, deploying the probe, and catching up with the rover
* Classes   : OffboardControl
* functions : __init__ , rover_vel_callback , drone_vel_callback , drone_pose_callback , gazebo_callback ,
              sonar_callback , sonar_1_callback , sonar_2_callback , ... sonar_8_callback , area_callback ,
              cam_callback , state_callback , set_mode , set_arm , distance , navigate_path , hover , 
              search , retreat , controller.
* global 
  variables : None 
'''

import rospy
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, Twist
from mavros_msgs.msg import State, PositionTarget
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Range
import numpy as np
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String, Header, Float64


class OffboardControl:
        """ Controller for PX4-UAV offboard mode """

        def __init__(self):

                rospy.init_node('OffboardControl', anonymous=True)

                # declaring variable types
                self.vel = Twist()                      # publishable velocity of drone
                self.curr_pose = PoseStamped()          # current position of drone 
                self.des_pose = PoseStamped()           # target position
                
                # program variables
                self.mode = "DETACH"                    # the program can be run in different modes that perform different tasks
                self.dist_threshold = 0.2               # standard distance threshold througout the program
                self.arm = True                         # can arm or disarm a vehicle
                self.cmd_mode = 'OFFBOARD'              # flight mode of the uav
                self.drop_loc = [None]*3                # washer location storage variable
                self.probe_pose = [None, None]          # variable for storing probe position
                # these three store the setpoint velocity of the drone
                self.vel.linear.x = 0
                self.vel.linear.y = 0
                self.vel.linear.z = 0
                # stores the incoming coordinates from image processing node
                self.cam_pose = [None, None, 0]
                # integral gain for probe pick-up operation
                self.ki = [0,0]
                # area of the target, also from image processing node
                self.area = 0
                # values of down sonar
                self.z = None        
                # maximum range of down sonar
                self.max_range = 0 
                # values of the eight horizontal sonars                     
                self.sonar = [None]*8

                self.navigate = 1                   # if it is set to 1, then navigation safety will be ensured.
                self.publishMode = 0                # if it is set to 1, then velocities will be published instead of position.
                                                    
                self.rover_pose = [0, 0]            # current position of rover, from mavros
                self.rover_vel = [0,0]              # current velocity of rover, from mavros
                self.drone_vel = [0,0]              # current drone velocity, from mavros


                # catch-up-with-rover process
                self.velError = [0, 0 ]             # relative error in position of drone and rover
                self.velIntegral = [0,0]            # integration of error
                self.velDiff = [0, 0]               # differential of error
                self.maxVelocity = [5,5]            # maximum and minimum values of the drone, that can be published
                self.prev_error = [0,0]             # most recent error in position
                self.tolerance = [-1, -1]           # calculated tolerance for catch-up-with-rover process after experimentation
                # pid gains respectively, for the catch-up-with-rover process!
                self.k= [0.5, 0.000005, 0.4] 
                                


                # defining ros subscribers
                self.pose_sub = rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, callback=self.drone_pose_callback)
                self.state_sub = rospy.Subscriber('/uav1/mavros/state', State, callback=self.state_callback)
                self.gazebo_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
                # /probe_area topic is published by detect_probe.py node
                self.area_sub = rospy.Subscriber('/probe_area', Float64, self.area_callback)
                self.sonar_sub = rospy.Subscriber('/sonar', Range, self.sonar_callback )
                self.sonar_1_sub = rospy.Subscriber('/sonar_1', Range, self.sonar_1_callback )
                self.sonar_2_sub = rospy.Subscriber('/sonar_2', Range, self.sonar_2_callback )
                self.sonar_3_sub = rospy.Subscriber('/sonar_3', Range, self.sonar_3_callback )
                self.sonar_4_sub = rospy.Subscriber('/sonar_4', Range, self.sonar_4_callback )
                self.sonar_5_sub = rospy.Subscriber('/sonar_5', Range, self.sonar_5_callback )
                self.sonar_6_sub = rospy.Subscriber('/sonar_6', Range, self.sonar_6_callback )
                self.sonar_7_sub = rospy.Subscriber('/sonar_7', Range, self.sonar_7_callback )
                self.sonar_8_sub = rospy.Subscriber('/sonar_8', Range, self.sonar_8_callback )
                # /probe_pose topic is published by detecct_probe.py node
                self.cam_sub = rospy.Subscriber('/probe_pose' , Point , self.cam_callback)
                self.rover_vel_sub= rospy.Subscriber('/uav0/mavros/local_position/velocity_local', TwistStamped, callback= self.rover_vel_callback)
                self.drone_vel_sub= rospy.Subscriber('/uav1/mavros/local_position/velocity_local', TwistStamped, callback= self.drone_vel_callback)

                # defining ros publishers
                self.pose_pub = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
                self.attach = rospy.Publisher('/attach', String, queue_size=10)
                self.vel_pub = rospy.Publisher('/uav1/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

                # calling the uav controller
                self.controller()
                

        def rover_vel_callback(self, msg):
                self.rover_vel[0] = msg.twist.linear.x
                self.rover_vel[1] = msg.twist.linear.y

        def drone_vel_callback(self, msg):
                self.drone_vel[0] = msg.twist.linear.x
                self.drone_vel[1] = msg.twist.linear.y

        def drone_pose_callback(self, msg):
                self.curr_pose = msg

           
        def gazebo_callback(self, msg):     
            # subscribing to probe position from gazebo modelstates        
                self.probe_pose[0] = msg.pose[1].position.x
                self.probe_pose[1] = msg.pose[1].position.y
                # subscribing to washer position from gazebo modelstates
                self.drop_loc[0] = msg.pose[3].position.x
                self.drop_loc[1] = msg.pose[3].position.y
                # subscribing to rover location from gazebo modelstates
                self.rover_pose[0] = msg.pose[4].position.x + self.tolerance[0]
                self.rover_pose[1] = msg.pose[4].position.y + self.tolerance[1]
                

        def sonar_callback(self, msg):
                self.z = msg.range
                self.max_range = msg.max_range

        def sonar_1_callback(self, msg):
                self.sonar[0] = msg.range

        def sonar_2_callback(self, msg):
                self.sonar[1] = msg.range

        def sonar_3_callback(self, msg):
                self.sonar[2] = msg.range

        def sonar_4_callback(self, msg):
                self.sonar[3] = msg.range
        def sonar_5_callback(self, msg):
                self.sonar[4] = msg.range

        def sonar_6_callback(self, msg):
                self.sonar[5] = msg.range

        def sonar_7_callback(self, msg):
                self.sonar[6] = msg.range

        def sonar_8_callback(self, msg):
                self.sonar[7] = msg.range

        def area_callback(self, msg):
                self.area=msg.data
        def cam_callback(self, msg):
                self.cam_pose[0] = msg.x
                self.cam_pose[1] = msg.y
                

        def state_callback(self, msg):
                
                self.set_mode('/uav1')
                if (self.arm==True and msg.armed==False) or( self.arm==False and msg.armed==True):
                        self.set_arm(self.arm, '/uav1')

        #   function    : set_mode()
        #   input       : self, uav - group of the px4 vehicle
        #   output      : none
        #   task        : sets the flight mode of the drone in this case based on the variable self.cmd_mode
        #   example call: self.set_mode('/uav1')

        def set_mode(self, uav):
                rospy.wait_for_service(uav + '/mavros/set_mode')
                try:
                        flightModeService = rospy.ServiceProxy(uav + '/mavros/set_mode', SetMode)
                        isModeChanged = flightModeService(custom_mode=self.cmd_mode)
                except rospy.ServiceException as e:
                        print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

        #   function    : set_arm()
        #   input       : self, flag , uav
        #   output      : none 
        #   task        : if flag is set to True, then it arms the drone. 
        #   example call: self.set_arm(True, '/uav1')

        def set_arm(self, flag, uav):
                rospy.wait_for_service(uav + '/mavros/cmd/arming')
                try:
                        armService = rospy.ServiceProxy(uav + '/mavros/cmd/arming', CommandBool)
                        armService(flag)
                except rospy.ServiceException as e:
                        print("Service arm call failed: %s" % e)



        #   function    : distance()
        #   input       : self
        #   output      : distance 
        #   task        : calculates the distance between the setpoint and current location of the drone . 
        #   example call: dist = self.distance()

        def distance(self):
                return np.sqrt((self.des_pose.pose.position.x-self.curr_pose.pose.position.x)*(self.des_pose.pose.position.x-self.curr_pose.pose.position.x) + (self.des_pose.pose.position.y-self.curr_pose.pose.position.y)*(self.des_pose.pose.position.y-self.curr_pose.pose.position.y) + (self.des_pose.pose.position.z-self.curr_pose.pose.position.z)*(self.des_pose.pose.position.z-self.curr_pose.pose.position.z))


        #   function    : navigate_path()
        #   input       : self
        #   output      : none 
        #   task        : checks the sonar readings and prevents the drone from crashing to the surface. 
        #   example call: self.navigate_path()

        def navigate_path(self):
                if self.z<5 or self.sonar[0]<20 or self.sonar[1]<20 or self.sonar[2]<20 or self.sonar[3]<20 or self.sonar[4]<20 or self.sonar[5]<20 or self.sonar[6]<20 or self.sonar[7]<20:
                        if self.publishMode:
                                self.vel.linear.z = 10
                        else:
                                self.des_pose.pose.position.z=self.curr_pose.pose.position.z+10
                if self.sonar[0]<5 or self.sonar[1]<5 or self.sonar[2]<5 or self.sonar[3]<5 or self.sonar[4]<5 or self.sonar[5]<5 or self.sonar[6]<5 or self.sonar[7]<5:
                        if self.publishMode:
                                self.vel.linear.z = 20
                        else:
                                self.des_pose.pose.position.x=self.curr_pose.pose.position.x
                                self.des_pose.pose.position.y=self.curr_pose.pose.position.y
                                self.des_pose.pose.position.z=self.curr_pose.pose.position.z+50


        #   function    : hover()
        #   input       : self
        #   output      : none 
        #   variables   : dock
        #   task        : first, it makes the drone hover over the washer's location, then slowly descends and finally detaches the probe. 
        #   example call: self.hover()

        def hover(self):        
                dock = False 
                while not rospy.is_shutdown():
                        if self.z<0.8:
                                if dock == False:
                                        if self.mode == "DETACH":
                                                
                                                self.attach.publish("DETACH") 
                                                print('probe detached! with the following error: ')
                                                print(self.drop_loc[0] - self.curr_pose.pose.position.x, self.drop_loc[1] - self.curr_pose.pose.position.y)
                                                self.des_pose.pose.position.z = self.curr_pose.pose.position.z + 5
                                                while True:
                                                        self.pose_pub.publish(self.des_pose)
                                                        if self.z>2:
                                                                break
                                                self.mode = "RETREAT"
                                                self.navigation = 1
                                                
                                        dock = True
                                        rospy.sleep(0.2)
                                        break
                        if self.z >= self.max_range:
                                self.des_pose.pose.position.z = self.curr_pose.pose.position.z - self.z
                        elif self.z<2:
                                self.des_pose.pose.position.z = self.curr_pose.pose.position.z - 0.1
                        elif self.z<10:
                                self.des_pose.pose.position.z = self.curr_pose.pose.position.z - 1
                        else:
                                self.des_pose.pose.position.z = self.curr_pose.pose.position.z - 5
                        self.pose_pub.publish(self.des_pose)
                        
        #   function    : search
        #   input       : self
        #   output      : None
        #   variables   : delta - calculate pid error, 
        #                 dist - calculates the distance of the target from the centre of the down camera.
        #                 probe_dist - calculates the abscissa and ordinate respectively,
        #                 kp - proportional gain for probe pick-up process.
        #   task        : applies PI control algorithm once the probe has been detected by the image processing node. And then it closes in on the probe slowly and picks it.     
        #   example call: self.search()

        def search(self):            
                delta = [0]*2            
                dist = np.sqrt((self.cam_pose[0] - 240)*(self.cam_pose[0] - 240) + (self.cam_pose[1] - 320)*(self.cam_pose[1] - 320))
                probe_dist = [abs(self.cam_pose[1] - 320), abs(self.cam_pose[0] - 240)]
                                        
                #   The drone will attach if the following condition is satisfied
                if (self.area>=50000 and dist<100) :
                        self.mode = "DETACH"
                        self.attach.publish("ATTACH")
                        print('probe attached!')
                        self.navigate = 1                 
                        # providing thrust to the drone after pickup 
                        self.des_pose.pose.position.x = self.curr_pose.pose.position.x
                        self.des_pose.pose.position.y = self.curr_pose.pose.position.y
                        self.des_pose.pose.position.z = self.curr_pose.pose.position.z + 5
                        while True:
                                self.pose_pub.publish(self.des_pose)
                                if self.distance()<self.dist_threshold:
                                        break
                        self.des_pose.pose.position.x = self.drop_loc[0]
                        self.des_pose.pose.position.y = self.drop_loc[1]
                                            
                else:
                        # determining the relative position of the probe wit respect to the image from the /uav_camera_down/raw_image topic
                        # and adjusting the drone's position 
                        kp = [0.004*probe_dist[0], 0.004*probe_dist[1]]
                        self.ki += [0.000001*probe_dist[0], 0.000001*probe_dist[1]]                 
                        
                        if (self.cam_pose[0] - 240)>0 and (self.cam_pose[1] - 320)>0:
                                # quadrant 4
                                delta[0] = -(kp[0] + self.ki[0])
                                delta[1] = -(kp[1] + self.ki[1])

                        elif (self.cam_pose[0] - 240)<0 and (self.cam_pose[1] - 320)>0:
                                # quadrant 3
                                delta[0] = -(kp[0] +self.ki[0])
                                delta[1] = kp[1] + self.ki[1]

                        elif (self.cam_pose[0] - 240)>0 and (self.cam_pose[1] - 320)<0:
                                # quadrant 1
                                delta[0] = kp[0] + self.ki[0]
                                delta[1] = -(kp[1] + self.ki[1])

                        else:
                                # quadrant 2
                                delta[0] = kp[0] + self.ki[0]
                                delta[1] = kp[1] + self.ki[1]
                        
                        self.des_pose.pose.position.x = self.curr_pose.pose.position.x + delta[0]
                        self.des_pose.pose.position.y = self.curr_pose.pose.position.y + delta[1]
                        self.des_pose.pose.position.z = self.curr_pose.pose.position.z
                        if self.area<1000 :
                                self.des_pose.pose.position.z = self.curr_pose.pose.position.z - 0.7
                        elif self.area<50000 :
                                self.des_pose.pose.position.z = self.curr_pose.pose.position.z - 0.1

        #   function    : retreat()
        #   input       : self
        #   output      : none
        #   variables   : dist - distance between drone and rover in horizontal plane
        #   task        : applies PID control algorithm that controls the drone velocity, minimizing the relative position between drone and rover. 
        #   example call: self.retreat()

        def retreat(self):       
                self.vel.linear.z = 0
                dist = np.sqrt((self.curr_pose.pose.position.x - self.rover_pose[0])*(self.curr_pose.pose.position.x - self.rover_pose[0]) + (self.curr_pose.pose.position.y - self.rover_pose[1])*(self.curr_pose.pose.position.y - self.rover_pose[1]))
                if dist<20:
                    # navigational safety is turned off if the drone is within 20 units of the rover.
                        self.navigate = 0        
                if dist<0.2*self.z:
                    # the drone starts descending inside this conical space
                        self.vel.linear.z = -2

                # change mode to AUTO.LAND if the following condition is met.
                if (self.z<0.5) and self.area>0 :
                        self.arm = False
                        self.cmd_mode = 'AUTO.LAND'
                        self.mode = "debug"
                        self.set_mode('/uav1')

                elif self.z<1 and self.area==0:
                        # take off when there is an error in catching up with the drone
                        self.vel.linear.z = 3
                                            
                else:
                        self.prev_error = np.copy(self.velError)
                        self.velError[0] = self.rover_pose[0] - self.curr_pose.pose.position.x
                        self.velError[1] = self.rover_pose[1] - self.curr_pose.pose.position.y
                        self.velIntegral[0] += self.velError[0]*0.05
                        self.velIntegral[1] += self.velError[1]*0.05
                        self.velDiff[0] = (self.velError[0] - self.prev_error[0])*20
                        self.velDiff[1] = (self.velError[1] - self.prev_error[1])*20
                        self.vel.linear.x = self.rover_vel[0] + self.k[0]*self.velError[0] + self.k[1]*self.velIntegral[0] + self.k[2]*self.velDiff[0]
                        self.vel.linear.y = self.rover_vel[1] + self.k[0]*self.velError[1] + self.k[1]*self.velIntegral[1] + self.k[2]*self.velDiff[1]
                        # print(str(self.velError[1])+' | ' + str(self.vel.linear.y) + ' | ' + str(self.k[0]*self.velError[1])+' | '+str(self.k[1]*self.velIntegral[1])+' | '+str(self.k[2]*self.velDiff[1]))
                        
        #   function    : controller()
        #   input       : self
        #   output      : none
        #   variables   : rate - rate at which the node should loop
        #                 r - just a flag for the search algorithm
        #   task        : assigns appropriate functions in different operating modes.                 
        #   example call: self.controller()

        def controller(self):
                """ A state machine developed to have UAV states """
                r = 0
                rate = rospy.Rate(20)
                while not rospy.is_shutdown():
                        if self.mode == "DETACH" :
                                self.des_pose.pose.position.x = self.drop_loc[0]
                                self.des_pose.pose.position.y = self.drop_loc[1]
                                if self.distance()<self.dist_threshold:
                                        self.navigate = 0
                                        self.hover()
                        elif self.mode == "SEARCH" :
                                self.des_pose.pose.position.x = self.probe_pose[0]
                                self.des_pose.pose.position.y = self.probe_pose[1]
                                if self.distance()<self.dist_threshold:
                                        r=1
                                if self.area >0 and r==1:
                                        self.navigate = 0
                                        self.search()
                        elif self.mode == "RETREAT":
                                self.publishMode = 1  
                                self.navigate = 1              
                                self.retreat()
                        elif self.mode == "debug":
                                print('task completed!')               
                                
                        if self.navigate:
                                self.navigate_path()
                        if self.publishMode ==0:
                            # controls drone by publishing setpoint positions
                                self.pose_pub.publish(self.des_pose)
                        else:
                            # velocity limiter code written below, to ensure safety.
                                if self.vel.linear.x>self.maxVelocity[0]:
                                        self.vel.linear.x==self.maxVelocity[0]
                                if self.vel.linear.x<-self.maxVelocity[0]:
                                        self.vel.linear.x==-self.maxVelocity[0]
                                if self.vel.linear.y>self.maxVelocity[1]:
                                        self.vel.linear.y==self.maxVelocity[1]
                                if self.vel.linear.y<-self.maxVelocity[1]:
                                        self.vel.linear.y==-self.maxVelocity[1]
                                # controls the drone by publishing setpoint velocities
                                self.vel_pub.publish(self.vel)
                        rate.sleep()


if __name__ == "__main__":
        print('OffboardControl version 3.2.1')
        OffboardControl()


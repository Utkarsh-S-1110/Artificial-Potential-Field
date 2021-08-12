#!/usr/bin/env python
# license removed for brevity

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time

class PathPlanner:
    def __init__(self,sx,sy,gx,gy,radius_cylinder,grid_side,ox,oy):
        self.sx=sx
        self.sy=sy                                                     
        self.gx=gx
        self.gy=gy                                                         
        self.radius_cylinder = radius_cylinder               
        self.grid_side=grid_side                                       
        self.ox = ox                             
        self.oy = oy
        self.set_value=2
        self.neighbour_listx=[int(self.gx/self.grid_side)]
        self.neighbour_listy=[int(self.gy/self.grid_side)]
        self.next_neighbour_listx=[]
        self.next_neighbour_listy=[]
        self.current_x=0.0
        self.current_y=0.0
        self.yaw=0.0
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.init_node('path_instructor', anonymous=True)
        self.move_cmd=Twist()
        self.position = Point()
        self.r = rospy.Rate(10)
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)


        time.sleep(10)
        self.declare_grid()
        self.initialise_obstacles()
        self.set_potential()
        self.moves=[[self.sx,self.sy]]
        self.current_position=[self.sx,self.sy]
        print("Going to find path")
        self.find_path()
        self.pmap.reverse()
        self.print_details()
        self.simulate()
            

    def declare_grid(self):
        self.sx=int(self.sx/self.grid_side)
        self.sy=int(self.sy/self.grid_side)                                                     
        self.gx=int(self.gx/self.grid_side)
        self.gy=int(self.gy/self.grid_side)                                                         
        self.radius_cylinder = self.radius_cylinder/self.grid_side
        for i in range(len(self.oy)):
            self.oy[i]/=self.grid_side
        for i in range(len(self.ox)):
            self.ox[i]/=self.grid_side 
        self.minx = min(min(self.ox), self.sx, self.gx)
        self.miny = min(min(self.oy), self.sy, self.gy)
        self.maxx = max(max(self.ox), self.sx, self.gx)
        self.maxy = max(max(self.oy), self.sy, self.gy)   
        print(self.minx)
        print(self.miny)
        print(self.maxx)
        print(self.maxy)
        self.moves=[[self.sx,self.sy]]
        self.current_position=[self.sx,self.sy]
        self.pmap = [[0 for i in range(int(self.maxx-self.minx)+1)] for j in range(int(self.maxy-self.miny)+1)] 
        print(self.pmap)
        
    def initialise_obstacles(self):
        for cnt,l in enumerate(self.ox):
            for m in range(int(math.floor(l-self.radius_cylinder)),int(math.ceil(l+self.radius_cylinder))):
                for k in range(int(math.floor(self.oy[cnt]-self.radius_cylinder)),int(math.ceil(self.oy[cnt]+self.radius_cylinder))):
                    if(((k<=self.maxy) and (k>=self.miny)) and ((m<=self.maxx) and (m>=self.minx))):
                        self.pmap[k][m]= 1
        self.pmap[self.gy][self.gx]=2
                        
    
    def set_potential(self):
        while self.pmap[int((self.sy-self.miny))][int((self.sx-self.minx))]==0 :
            for cnt,l in enumerate(self.neighbour_listx):
                for m in range(int(math.floor(l-1)),int(math.ceil(l+2))):
                    for k in range(int(math.floor(self.neighbour_listy[cnt]-1)),int(math.ceil(self.neighbour_listy[cnt]+2))):
                        if(((k<=self.maxy) and (k>=self.miny)) and ((m<=self.maxx) and (m>=self.minx))):
                            if(self.pmap[k][m]==0):
                                self.next_neighbour_listx.append(m)
                                self.next_neighbour_listy.append(k)
            self.update_potential()
            
                       

    def update_potential(self):
        self.set_value+=1
        for i in range(len(self.next_neighbour_listx)):
            if(self.pmap[self.next_neighbour_listy[i]][self.next_neighbour_listx[i]]==0):
                self.pmap[self.next_neighbour_listy[i]][self.next_neighbour_listx[i]]=self.set_value
        del self.neighbour_listx[:]    
        del self.neighbour_listy[:]
        self.neighbour_listx=list(self.next_neighbour_listx)
        self.neighbour_listy=list(self.next_neighbour_listy)       
        del self.next_neighbour_listx[:]
        del self.next_neighbour_listy[:]
    
    def find_path(self):
        smallest=self.pmap[int(self.current_position[1])][int(self.current_position[0])]
        while ((self.current_position[0]!=self.gx) or (self.current_position[1]!=self.gy)):
            w1=int(self.current_position[0])-1
            w2=int(self.current_position[0])+2
            h1=int(self.current_position[1])-1
            h2=int(self.current_position[1])+2
            for m in range(w1,w2):
                for k in range(h1,h2):
                    if(((k<=self.maxy) and (k>=self.miny)) and ((m<=self.maxx) and (m>=self.minx))):
                        if ((smallest >= self.pmap[k][m]) and (self.pmap[k][m] != 1) and (self.pmap[k][m] != 0)) :
                            smallest=self.pmap[k][m]
                            self.current_position[0]=m
                            self.current_position[1]=k
            self.moves.append([self.current_position[0],self.current_position[1]]) 
              
    
    def print_details(self):
        for i in range(int(self.maxy-self.miny)+1):
            print(self.pmap[int(i)])
        print("Path to be taken is -") 
        print(self.moves)
    
    def simulate(self):
        kp_distance = 0.5
        kp_angle = 2.0
        ki=0.001
        kd=0.1
        
        previous_pop=[self.sx,self.sy]
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
        while len(self.moves)>1:
            previous_x=self.current_x
            previous_y=self.current_y
            grid_change=0 
            current_pop=self.moves.pop(1)
            
            error_add=0
            error_change=0
            previous_error=math.sqrt(math.pow((current_pop[1] - previous_pop[1]), 2) + math.pow((current_pop[0] -  previous_pop[0]), 2))

            while grid_change< 0.5:
                grid_change = max(abs(self.current_x-previous_x),abs(self.current_y-previous_y))
                
                x_start = self.current_x
                y_start = self.current_y
                path_angle = math.atan2(current_pop[1] - previous_pop[1], current_pop[0]- previous_pop[0])

                
                current_error = math.sqrt(math.pow((current_pop[1] - y_start), 2) + math.pow((current_pop[0] - x_start), 2))

                error_add+=current_error
                error_change=current_error-previous_error

                control_signal_distance = kp_distance*current_error + ki*error_add + kd*error_change

                self.move_cmd.angular.z = kp_angle*((path_angle) - self.yaw)
                self.move_cmd.linear.x = min(control_signal_distance, 0.2)

                previous_error=current_error

                if self.move_cmd.angular.z > 0:
                    self.move_cmd.angular.z = min(self.move_cmd.angular.z, 1.5)
                else:
                    self.move_cmd.angular.z = max(self.move_cmd.angular.z, -1.5)

                self.pub.publish(self.move_cmd)
                self.r.sleep()
            previous_pop[0]=current_pop[0]    
            previous_pop[1]=current_pop[1]        

        
        self.move_cmd.angular.z=0.0
        self.move_cmd.linear.x =0.0
        self.pub.publish(self.move_cmd)
        return           

       
    def get_rotation (self,msg):
        self.current_x=msg.pose.pose.position.x
        self.current_y=msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)

                


def main():                                                         
    sx=0
    sy=0                                                     
    gx=6
    gy=6                                                       
    radius_cylinder = 0.25               
    grid_side=0.5                                       
    oy = [1.5,3.0,4.5,0.0,1.5,3.0,4.5,0.0,1.5,3.0,4.5,0.0,1.5,3.0,4.5]                             
    ox = [0.0,0.0,0.0,1.5,1.5,1.5,1.5,3.0,3.0,3.0,3.0,4.5,4.5,4.5,4.5]                           
    obj=PathPlanner(sx,sy,gx,gy,radius_cylinder,grid_side,ox,oy)

main()    

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import pickle
# import numpy as it may be used in future labs
import numpy as np
import pickle
import math
Row = 16
Column = 16
wheel_rotation_velocity = 6
wheel_rotation_velocity_spin = 2
wheel_radius = .0205
distance_between_wheels = .052
wheel_circumference = wheel_radius * 2 * 3.14
encoder_unit = wheel_circumference/6.28
space_cell = .18
error = 0.01
from collections import deque



#######################################################
# Creates Robot
#######################################################
robot = Robot()


#######################################################
# Sets the time step of the current world
#######################################################
timestep = int(robot.getBasicTimeStep())

#######################################################
# Gets Robots Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/distancesensor
#######################################################
frontDistanceSensor = robot.getDevice('front distance sensor')
leftDistanceSensor = robot.getDevice('left distance sensor')
rightDistanceSensor = robot.getDevice('right distance sensor')
rearDistanceSensor = robot.getDevice('rear distance sensor')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
rearDistanceSensor.enable(timestep)

#######################################################
# Gets Robots Lidar Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/lidar
#######################################################
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar_horizontal_res = lidar.getHorizontalResolution()
lidar_num_layers = lidar.getNumberOfLayers()
lidar_min_dist = lidar.getMinRange()
lidar_max_dist = lidar.getMaxRange()


print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res, "\nLidar Image Number of Layers: ", lidar_num_layers)
print("Lidar Range: [",lidar_min_dist," ,", lidar_max_dist,'] in meters')

#######################################################
# Gets Robots Camera
# Documentation:
#  https://cyberbotics.com/doc/reference/camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#######################################################
# Gets Robots Motors
# Documentation:
#  https://cyberbotics.com/doc/reference/motor
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)




#######################################################
# Gets Robot's the position sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/positionsensor
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
leftsample_period = leftposition_sensor.getSamplingPeriod()
rightposition_sensor.enable(timestep)
rightsample_period = rightposition_sensor.getSamplingPeriod()

#######################################################
# Gets Robot's IMU sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/inertialunit
#######################################################
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

class location():
    def __init__(self):
        self.location_row = 15
        self.location_column = 0
        self.direction = "North"
        self.next_cell = cell(0,0)
    def update_location(self):
        if (self.direction == "North"):
            self.location_row = self.location_row-1
        if (self.direction == "South"):
            self.location_row = self.location_row+1
        if (self.direction == "East"):
            self.location_column = self.location_column+1
        if (self.direction == "West"):
            self.location_column = self.location_column-1
        return
    def rotate(self,radians):                                                 ########
        start_time = robot.getTime()
        angular_distance = radians
        angular_velocity = (2*wheel_radius*wheel_rotation_velocity_spin)/distance_between_wheels
        time = angular_distance/angular_velocity

        start_direction = round(imu.getRollPitchYaw()[2],2)
        end_direction = start_direction + radians

        if(end_direction> 3.14):
            end_direction = round(end_direction-(2*math.pi),2)
        if(end_direction< -3.14):
            end_direction = round(end_direction+(2*math.pi),2)
        if(end_direction== -3.14):
            end_direction = 3.14 
        if(self.direction=="North" or self.direction=="South") and (end_direction<=.0698 or end_direction>=6.213):
            end_direction = 0


        if(end_direction>1.501 and end_direction < 1.6406):
            end_direction =  1.57
        elif(end_direction <= .0698 and end_direction >= -.0698):                        #less than 4 degrees and greater than 356 
            end_direction = 0
        elif(end_direction>3.071 or end_direction<-3.211):
            end_direction = 3.14
        elif(end_direction < -1.501 and end_direction > -1.6406):
            end_direction = -1.57
        else:
            end_direction = end_direction
        
        if(radians == 3.14 or radians == -3.14):
            if(start_direction == 0):
                end_direction = 3.14
            if(start_direction == 3.14):
                end_direction = 0
            if(start_direction == 1.57):
                end_direction = -1.57
            if(start_direction == -1.57):
                end_direction = 1.57
       
        
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        while(robot.step(timestep)!=-1):
            current_direction = round(imu.getRollPitchYaw()[2],2)               #direction of robot in radians
            #if(Backtrack_Option==True):
                #print("Starting direction is {} and the end direciton is {}".format(current_direction, end_direction))
            if(robot.getTime()-start_time>=time):
                if(current_direction>end_direction):
                    leftMotor.setVelocity(.75)
                    rightMotor.setVelocity(-.75)
                elif(current_direction<end_direction):
                    leftMotor.setVelocity(-.75)
                    rightMotor.setVelocity(.75)
                elif((abs(current_direction)>=3.13) and (end_direction == 3.14 or end_direction == -3.14)):
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    break
                else:
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    break
            else:
                pass
        self.update_direction(end_direction)
    
    def optionalturn(self, heading):
        if(heading == "left"):
            if(self.direction =="North" and self.location_column>0 and maze.matrix[self.location_row][self.location_column-1].discovered == False):
                self.rotate(1.57)
            if(self.direction =="East" and self.location_row>0 and maze.matrix[self.location_row-1][self.location_column].discovered == False):
                self.rotate(1.57)
            if(self.direction =="South" and self.location_column<15 and maze.matrix[self.location_row][self.location_column+1].discovered == False):
                self.rotate(1.57)
            if(self.direction =="West" and self.location_row<15 and maze.matrix[self.location_row+1][self.location_column].discovered == False):
                self.rotate(1.57)
        elif(heading == "right"):
            if(self.direction == "North" and self.location_column<15 and maze.matrix[self.location_row][self.location_column+1].discovered == False):
                self.rotate(-1.57)
            if(self.direction == "East" and self.location_row<15 and maze.matrix[self.location_row+1][self.location_column].discovered == False):
                self.rotate(-1.57)
            if(self.direction == "South" and self.location_column>0 and maze.matrix[self.location_row][self.location_column-1].discovered == False):
                self.rotate(-1.57)
            if(self.direction == "West" and self.location_column> 0 and maze.matrix[self.location_row-1][self.location_column].discovered == False):
                self.rotate(-1.57)
        else:
            return
        
    def update_direction(self, radian):
        if(radian>1.501 and radian < 1.6406):
            self.direction = "North"
        elif(radian <= .0698 and radian >= -.0698):                        #less than 4 degrees and greater than 356 
            self.direction = "East"
        elif(radian>3.071 or radian<-3.211):
            self.direction = "West"
        elif(radian < -1.501 and radian > -1.6406):
            self.direction = "South"
        else:
            return
        
    def Compare_Backtrack(self, next_coordinates):
        if(self.direction == "North"):
            if (next_coordinates.Column == self.location_column):                   # if the robot goes straight "North" being on the same row
                if(next_coordinates.Row < self.location_row):
                    return                                                          #just return to make it move straight
            else:
                if(next_coordinates.Column < self.location_column):
                    Robot_Pose.rotate(1.57)
                else:
                    Robot_Pose.rotate(-1.57)

        elif(self.direction == "East"):
            if(next_coordinates.Row == self.location_row):
                if(next_coordinates.Column > self.location_column):
                    return
            else:
                if(next_coordinates.Row < self.location_row):
                    Robot_Pose.rotate(1.57)
                else:
                    Robot_Pose.rotate(-1.57)
        elif(self.direction == "South") :
            if(next_coordinates.Column == self.location_column):
                if(next_coordinates.Row > self.location_row):
                    return
            else:
                if(next_coordinates.Column > self.location_column):
                    Robot_Pose.rotate(1.57)
                else:
                    Robot_Pose.rotate(-1.57)
        elif(self.direction == "West"):
            
            if(next_coordinates.Row == self.location_row):
                if(next_coordinates.Column < self.location_column):
                    return
            else:
                if(next_coordinates.Row > self.location_row):
                    Robot_Pose.rotate(1.57)
                else:
                    Robot_Pose.rotate(-1.57)
        else:
            return


top_left = [True,False,False,True]
top_right = [True,True,False,False]
bottom_left = [False,False,True,True]
bottom_right = [False,True,True,False]

stack = deque()
class Map():  
    def __init__(self):
        self.matrix = []
        index_Row = 0
        index_Column = 0
        while index_Row < Row:   
            a = []
            # A for loop for column entries
            while index_Column < Column: 
                space = cell(index_Row,index_Column)
                a.append(space)
                index_Column= index_Column+1
            self.matrix.append(a)
            index_Column=0
            index_Row = index_Row +1
    def print(self):
        print("row is {}. Column is {}".format(Robot_Pose.location_row, Robot_Pose.location_column))
        print("Heading is {}".format(Robot_Pose.direction))
        print("Inside the cell is {}".format(self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].discovered))
        for w in self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls:
            print("{}".format(w), end=" ")
        print("")
    def update_maze(self, sensor):
        if(Robot_Pose.direction == "North" ):
            if (sensor[0] < .18):
               self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[0] = True
            if (sensor[1] < .18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[1] = True
            if (sensor[2] < .18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[2] = True
            if (sensor[3] < .18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[3] = True
        elif(Robot_Pose.direction == "East"):
            if(sensor[3]<.18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[0] = True
            if(sensor[0]<.18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[1] = True
            if(sensor[1]<.18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[2] = True
            if(sensor[2]<.18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[3] = True
        elif(Robot_Pose.direction == "South"):
            if(sensor[2]<.18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[0] = True
            if(sensor[3]<.18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[1] = True
            if(sensor[0]<.18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[2] = True
            if(sensor[1]<.18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[3] = True
        else:
            if(sensor[1]<.18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[0] = True
            if(sensor[2]<.18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[1] = True
            if(sensor[3]<.18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[2] = True
            if(sensor[0]<.18):
                self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls[3] = True 
        self.matrix[Robot_Pose.location_row][Robot_Pose.location_column].discovered = True
        stack.append(self.matrix[Robot_Pose.location_row][Robot_Pose.location_column])
    def availability(self):
        if(Robot_Pose.direction == "North"):
            if(Robot_Pose.location_column !=0 and self.matrix[Robot_Pose.location_row][Robot_Pose.location_column-1].discovered == False and left_dist>.18):
                Robot_Pose.next_cell.Row = Robot_Pose.location_row
                Robot_Pose.next_cell.Column = Robot_Pose.location_column-1
                return True
            elif(Robot_Pose.location_column !=15 and self.matrix[Robot_Pose.location_row][Robot_Pose.location_column+1].discovered == False and right_dist>.18):
                Robot_Pose.next_cell.Row = Robot_Pose.location_row
                Robot_Pose.next_cell.Column = Robot_Pose.location_column+1
                return True
            elif(Robot_Pose.location_row != 0 and self.matrix[Robot_Pose.location_row-1][Robot_Pose.location_column].discovered == False and front_dist>.18):
                Robot_Pose.next_cell.Row = Robot_Pose.location_row-1
                Robot_Pose.next_cell.Column = Robot_Pose.location_column
                return True
            else:
                return False
        elif(Robot_Pose.direction == "East"):
            if(Robot_Pose.location_row !=0 and self.matrix[Robot_Pose.location_row-1][Robot_Pose.location_column].discovered == False and left_dist>.18):
                Robot_Pose.next_cell.Row = Robot_Pose.location_row-1
                Robot_Pose.next_cell.Column = Robot_Pose.location_column
                return True
            elif(Robot_Pose.location_row !=15 and self.matrix[Robot_Pose.location_row+1][Robot_Pose.location_column].discovered == False and right_dist>.18):
                Robot_Pose.next_cell.Row = Robot_Pose.location_row+1
                Robot_Pose.next_cell.Column = Robot_Pose.location_column
                return True
            elif(Robot_Pose.location_column!=15 and self.matrix[Robot_Pose.location_row][Robot_Pose.location_column+1].discovered == False and front_dist>.18):
                Robot_Pose.next_cell.Row = Robot_Pose.location_row
                Robot_Pose.next_cell.Column = Robot_Pose.location_column+1
                return True
            else:
                return False
        elif(Robot_Pose.direction == "South"):
            if(Robot_Pose.location_column !=0 and self.matrix[Robot_Pose.location_row][Robot_Pose.location_column-1].discovered == False and right_dist>.18):
                Robot_Pose.next_cell.Row = Robot_Pose.location_row
                Robot_Pose.next_cell.Column = Robot_Pose.location_column-1
                return True
            elif(Robot_Pose.location_column !=15 and self.matrix[Robot_Pose.location_row][Robot_Pose.location_column+1].discovered == False and left_dist>.18):
                Robot_Pose.next_cell.Row = Robot_Pose.location_row
                Robot_Pose.next_cell.Column = Robot_Pose.location_column+1
                return True
            elif(Robot_Pose.location_row != 15 and self.matrix[Robot_Pose.location_row+1][Robot_Pose.location_column].discovered == False and front_dist>.18):
                Robot_Pose.next_cell.Row = Robot_Pose.location_row+1
                Robot_Pose.next_cell.Column = Robot_Pose.location_column
                return True
        elif(Robot_Pose.direction == "West"):
            if(Robot_Pose.location_row !=0 and self.matrix[Robot_Pose.location_row-1][Robot_Pose.location_column].discovered == False and right_dist>.18):
                Robot_Pose.next_cell.Row = Robot_Pose.location_row-1
                Robot_Pose.next_cell.Column = Robot_Pose.location_column
                return True
            elif(Robot_Pose.location_row !=15 and self.matrix[Robot_Pose.location_row+1][Robot_Pose.location_column].discovered == False and left_dist>.18):
                Robot_Pose.next_cell.Row = Robot_Pose.location_row+1
                Robot_Pose.next_cell.Column = Robot_Pose.location_column
                return True
            elif(Robot_Pose.location_column!=0 and self.matrix[Robot_Pose.location_row][Robot_Pose.location_column-1].discovered == False and front_dist>.18):
                Robot_Pose.next_cell.Row = Robot_Pose.location_row
                Robot_Pose.next_cell.Column = Robot_Pose.location_column-1
                return True
            else:
                return False
        else:
            return False
        
    def finished(self):
        row_check = 0
        column_check = 0
        if[Robot_Pose.location_row<15 and Robot_Pose.location_column<15]:
            while(row_check<15):
                while(column_check<15):
                    if(self.matrix[row_check][column_check].walls == top_left and self.matrix[row_check][column_check+1].walls == top_right and self.matrix[row_check+1][column_check].walls == bottom_left):
                        return True
                    elif(self.matrix[row_check][column_check].walls == top_left and self.matrix[row_check][column_check+1].walls == top_right and self.matrix[row_check+1][column_check+1].walls == bottom_right):
                        return True
                    elif(self.matrix[row_check][column_check].walls == top_left and self.matrix[row_check+1][column_check].walls == bottom_left and self.matrix[row_check+1][column_check+1].walls == bottom_right):
                        return True
                    elif(self.matrix[row_check][column_check+1].walls == top_right and self.matrix[row_check+1][column_check].walls == bottom_left and self.matrix[row_check+1][column_check+1].walls == bottom_right):
                        return True
                    else:
                        print("")
                    column_check= column_check+1
                column_check=0
                row_check = row_check+1
        return False
        


    
class cell():
    def __init__(self, index_Row, index_Column):
        self.Row = index_Row
        self.Column = index_Column
        self.discovered = False
        self.walls = [False,False,False,False]

    
            
i = 0
maze = Map()
Robot_Pose = location()
recorded_distance = 0    
# Main loop:
# perform simulation steps until Webots is stopping the controller

current =  [0,0]
previous = [0,0]
distance = [0,0]
difference = [0,0]
angle = 0
distance_range = [0,0,0,0]
Backtrack_Option = False

times = 0

while robot.step(timestep) != -1:
     # Getting full Range Image from Lidar returns a list of 1800 distances = 5 layers X 360 distances
    full_range_image = lidar.getRangeImage()
    # print size of Range Image
    #print('#################################################################')
    #print("Lidar's Full Range Image Size: ", len(full_range_image))
    # Compare Distance Sensors to Lidar Ranges
    front_dist = frontDistanceSensor.getValue()
    right_dist = rightDistanceSensor.getValue()
    rear_dist = rearDistanceSensor.getValue()
    left_dist = leftDistanceSensor.getValue()

    #print("Distance Sensor vs Lidar")
    #print("\tFront:\t", front_dist, "\t|", full_range_image[0])
    #print("\tRight:\t", right_dist, "\t|", full_range_image[90])
    #print("\tRear:\t", rear_dist, "\t|", full_range_image[180])
    #print("\tLeft:\t", left_dist, "\t|", full_range_image[270]) 
    
    if(leftMotor.getVelocity() == rightMotor.getVelocity() and Backtrack_Option == False):
        if(Robot_Pose.next_cell.Row != 0 and Robot_Pose.next_cell.Column !=0):
            Robot_Pose.Compare_Backtrack(Robot_Pose.next_cell)
            Robot_Pose.next_cell.Column = 0
            Robot_Pose.next_cell.Row = 0
        current[0] = leftposition_sensor.getValue()                  
        current[1] = rightposition_sensor.getValue()
        for wheel in range (2):
            difference[wheel] = current[wheel] - previous[wheel]
        if difference[0] >= 8.83:                                   #this if statment determines wehter or not the robot move cells
            for wheel in range(2):
                previous[wheel] = current[wheel]  
                difference [wheel] = 0
            leftMotor.setVelocity(0)                    #stops the robot so it can continue other stuff
            rightMotor.setVelocity(0)
            Robot_Pose.update_location()                #updates the location of the robot within maze based on the row and column
            distance_range[0] =front_dist 
            distance_range[1] = right_dist
            distance_range[2] = rear_dist
            distance_range[3] = left_dist
            maze.update_maze(distance_range)
            maze.print()
            if( left_dist > .18):                                 #Create Additional conditions
                Robot_Pose.optionalturn("left")                                               #COnsider all directions!!!!!!
            elif(right_dist>.18):
                 Robot_Pose.optionalturn("right")
            elif(front_dist<.18 and left_dist<.18 and right_dist<.18): 
                Robot_Pose.rotate(3.14)
                temp = stack.pop()
                tmep = stack.pop()
                Backtrack_Option = True                      
            else:
                print("continue")  
            previous[0] = leftposition_sensor.getValue() 
            previous[1] = rightposition_sensor.getValue()


        for wheel in range (2):
            distance[wheel] = encoder_unit * difference[wheel]
  

    else:
        leftMotor.setVelocity(wheel_rotation_velocity)                    #stops the robot so it can continue other stuff
        rightMotor.setVelocity(wheel_rotation_velocity)
        current[0] = leftposition_sensor.getValue()                  
        current[1] = rightposition_sensor.getValue()
        for wheel in range (2):
            difference[wheel] = current[wheel] - previous[wheel]
        if difference[0] >= 8.83:  
            for wheel in range(2):
                previous[wheel] = current[wheel]  
                difference [wheel] = 0
            leftMotor.setVelocity(0)                    #stops the robot so it can continue other stuff
            rightMotor.setVelocity(0)
            Robot_Pose.update_location()

            temp = stack.pop()
            if(maze.availability() == True):
                Backtrack_Option = False
                leftMotor.setVelocity(0)                    #stops the robot so it can continue other stuff
                rightMotor.setVelocity(0)
            else:
                Robot_Pose.Compare_Backtrack(temp)
                leftMotor.setVelocity(0)                    #stops the robot so it can continue other stuff
                rightMotor.setVelocity(0)
            previous[0] = leftposition_sensor.getValue() 
            previous[1] = rightposition_sensor.getValue()

    if(maze.finished()==True):       
        break   
               


    if Backtrack_Option==False:
        leftMotor.setVelocity(wheel_rotation_velocity)
        rightMotor.setVelocity(wheel_rotation_velocity)

        

    #if full_range_image[0] < .05:

    #    leftMotor.setVelocity(0)
    #    rightMotor.setVelocity(0)
    #    break
# Enter here exit cleanup code.

count_row = 0
count_column = 0
new_count = 0
while(count_row < 16):
    while(count_column < 16):
        for w in maze.matrix[Robot_Pose.location_row][Robot_Pose.location_column].walls:
            print("{}".format(w), end=" ")
        if( maze.matrix[Robot_Pose.location_row][Robot_Pose.location_column].discovered == True):
            new_count = new_count+1



#file_name = r'C:/Users/tle72/OneDrive\Desktop/CDA4621_MicroMouse\WebotsMicroMouse/controllers/FastestRun/maze.pickle'
#with open(file_name, 'wb') as file:
#    pickle.dump(maze)

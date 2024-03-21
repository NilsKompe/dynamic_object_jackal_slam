#!/usr/bin/env python3.8

import pickle, pathlib 
import numpy as np
from scipy.stats import norm
from matplotlib import pyplot as plt
from geometry_msgs.msg import Pose
import pandas as pd
from scipy.spatial.transform import Rotation
   
        
class DetermineSensorVariance:
    def __init__(self):
        self.marker_poses = [] #list for all measured marker poses
        self.true_marker_poses = [] #list for all true marker poses. One for each measurement from a different position
        self.jackal_poses = [] #list for all true jackal_poses corresponding by index to the measured marker poses
        self.directions_of_measurements = [] #list of all directions of the measurements. One for each measurement from a different position
        self.w_r_true = [] #lists containing the true w_r and w_ß for each measurement 
        self.w_ß_true = []
        self.w_r_measured = [] #lists containing w_r and w_ß of each measurement
        self.w_ß_measured = []
        self.counter = 0 #number of measurements from different positions
        
    def load_data(self): #load measured data
        with open(str(pathlib.Path(__file__).parent.resolve().parents[0]) + "/data/detected_markers.json","rb") as file_marker_poses:
            while True:
                try:
                    data1 = pickle.load(file_marker_poses)
                    self.marker_poses.append(data1)
                except EOFError:
                    break
        
        with open(str(pathlib.Path(__file__).parent.resolve().parents[0]) + "/data/true_poses.json","rb") as file_gazebo_poses:
            while True:
                try:
                    data = pickle.load(file_gazebo_poses)
                    self.true_marker_poses.append(data[0])
                    self.directions_of_measurements.append(data[1])
                    self.jackal_poses.append(data[2])
                    self.counter = self.counter + 1
                except EOFError:
                    break

        ##### in case a falsely done measurement needs to be deleted
        # del(self.markers[12])
        # del(self.true_marker_pose[12])  
        # del(self.jackal_poses[12])     
        # self.counter = self.counter - 1  
        #######



        for i in range(0,self.counter): #for each measurement from a different position

            if self.directions_of_measurements[i] == "x-":
                #e.g. jackal is facing the cube straight from x direction with lower x coordinate than the cube -> marker is detected 0.15 before the center of the cube
                self.true_marker_poses[i].position.x = self.true_marker_poses[i].position.x - 0.15 
            elif self.directions_of_measurements[i] == "x+":
                self.true_marker_poses[i].position.x = self.true_marker_poses[i].position.x + 0.15
            elif self.directions_of_measurements[i] == "y-":
                self.true_marker_poses[i].position.y = self.true_marker_poses[i].position.y - 0.15
            elif self.directions_of_measurements[i] == "y+":
                self.true_marker_poses[i].position.y = self.true_marker_poses[i].position.y + 0.15
            else: 
                print("unknown direction input", i)

            for j in range(0,len(self.marker_poses[0])): #for each measurement from this position
                self.calculate_w_parameter(self.marker_poses[i][j],self.jackal_poses[i][j],i) #calulate true and measured w_r and w_ß
            
        w_r_differences = np.array(self.w_r_measured)-np.array(self.w_r_true) #determine deviation of measured and true range 
        w_ß_differences = np.array(self.w_ß_measured)-np.array(self.w_ß_true) #determine deviation of measured and true angle
        variance_w_r = np.var(w_r_differences)  
        variance_w_ß = np.var(w_ß_differences)

        print("w_r variance and standard deviation in meters", variance_w_r, np.sqrt(variance_w_r))
        print("w_ß variance and standard deviation in degrees",np.rad2deg(variance_w_ß),np.sqrt(np.rad2deg(variance_w_ß)))
        print("number of different measuring positions", self.counter)
        print("datasize",len(self.w_r_measured))

        self.gauss_distribution(w_r_differences,"w_r")#print each gauss distribution
        self.gauss_distribution(w_ß_differences,"w_ß")
    
    def calculate_w_parameter(self,marker_pose,jackal_pose,i):
        #true range to marker calculated with Pythagorean theorem
        self.w_r_true.append(np.sqrt((self.true_marker_poses[i].position.y-jackal_pose.position.y)**2 + (self.true_marker_poses[i].position.x - jackal_pose.position.x)**2)) 

        #convert the orientation of the jackal as quaternion into euler. Only the rotation around the z-axis is needed, beacuase there is no rotation around the x-axis or y-axis 
        rotation_euler = Rotation.from_quat([jackal_pose.orientation.x,jackal_pose.orientation.y,jackal_pose.orientation.z,jackal_pose.orientation.w]).as_euler("xyz", degrees=True) 
        #true angle to the marker calculated with arctan2 and the current orientation of the jackal 
        self.w_ß_true.append(np.arctan2((self.true_marker_poses[i].position.y-jackal_pose.position.y),(self.true_marker_poses[i].position.x - jackal_pose.position.x)) -
                              np.deg2rad(rotation_euler[2]))
        #measured distance to the marker calculated with Pythagorean theorem
        self.w_r_measured.append(np.sqrt((marker_pose.position.y)**2 + (marker_pose.position.x)**2))
        #measured angle to the marker calculated with arctan
        self.w_ß_measured.append(np.arctan(marker_pose.position.y/marker_pose.position.x))
     

    def gauss_distribution(self,deviation,title): #plot the gauss distribution
        mean,std=norm.fit(deviation)
        # print(std**2)
        plt.hist(deviation, bins=30, density=True)
        xmin, xmax = plt.xlim()
        x = np.linspace(xmin, xmax, 100)
        y = norm.pdf(x, mean, std) #probability density function
        plt.plot(x, y)
        plt.title(title)
        plt.ylabel("probability density function")
        plt.xlabel("values")
        plt.show()
        
if __name__ == '__main__':
    class_determine_variance = DetermineSensorVariance()
    class_determine_variance.load_data()
    

  
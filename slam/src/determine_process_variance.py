#!/usr/bin/env python3.8

import pickle, pathlib 
import numpy as np
from scipy.stats import norm
from matplotlib import pyplot as plt

class DetermineProcessVariance:
    def __init__(self):
        self.twist_testdata_linear = []
        self.twist_testdata_angular = []
        self.odom_twist_linear = []
        self.odom_twist_angular = []
        self.deviations_linear = []
        self.deviations_angular = []
        self.variance_linear = 0
        self.variance_angular = 0

    def load_data(self):
        with open(str(pathlib.Path(__file__).parent.resolve().parents[0]) + "/data/twist_and_odom.json","rb") as f:
           odom_data = pickle.load(f)
        self.twist_testdata_linear = odom_data[1]
        self.twist_testdata_angular = odom_data[3]
        self.odom_twist_linear = odom_data[5]
        self.odom_twist_angular = odom_data[7]
        #checks if any of the 4 values from one same datainput varies
        for i in range(0,int(len(self.twist_testdata_linear)/4.0)):
            if self.twist_testdata_linear[i * 4] != self.twist_testdata_linear[i*4+1] or self.twist_testdata_linear[i * 4] != self.twist_testdata_linear[i*4+2] or self.twist_testdata_linear[i * 4] != self.twist_testdata_linear[i*4+3]:
                print("linear",i*4,self.twist_testdata_linear[i*4])

        for i in range(0,int(len(self.twist_testdata_linear)/4.0)):
            if self.twist_testdata_angular[i * 4] != self.twist_testdata_angular[i*4+1] or self.twist_testdata_angular[i * 4] != self.twist_testdata_angular[i*4+2] or self.twist_testdata_angular[i * 4] != self.twist_testdata_angular[i*4+3]:
                print("angular",i*4,self.twist_testdata_linear[i*4])
            
    def calculate_standard_deviations(self):
        print()
        print("Values with larger linear deviation than 0.1 m:")
        self.deviations_linear = np.squeeze(np.array(self.odom_twist_linear) - np.array(self.twist_testdata_linear))
        for d in self.deviations_linear:
            if abs(d)>0.1:
                print(d)
        print("Values with larger angular deviation than 0.3 rad (ca. 17.19 degree)")
        self.deviations_angular = np.squeeze(np.array(self.odom_twist_angular) - np.array(self.twist_testdata_angular))
        for d in self.deviations_angular:
            if abs(d)>0.3:
                print(d)

        self.standard_deviation_linear = np.sqrt(np.var(self.deviations_linear))
        self.standard_deviation_angular = np.rad2deg(np.sqrt(np.var(self.deviations_angular)))
        print("standard_deviation_linear in meters",self.standard_deviation_linear)
        print("standard_deviation_angular in degree",self.standard_deviation_angular)

        self.gauss_distribution(self.deviations_linear,"linear")
        self.gauss_distribution(self.deviations_angular,"angular")

    def gauss_distribution(self,deviation,title):
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
    process_variance = DetermineProcessVariance()
    process_variance.load_data()
    process_variance.calculate_standard_deviations()
    
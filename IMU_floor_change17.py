#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import cumtrapz

class DistanceEstimation:
    def __init__(self, acceleration_threshold=0.5):
        self.linear_acceleration_z = 9.81
        self.prev_linear_acceleration_z = self.linear_acceleration_z
        self.initial_values_recorded = False  # Check if initial values have been recorded
        self.acceleration_threshold = acceleration_threshold

        self.over_flag = False  
        self.under_flag = False

        # Initialize for plotting
        self.time_stamps = []
        self.distance_z_values = []
        self.acceleration_values = []

        # Create new figure and axes for the plot
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Distance Z')
        self.ax.legend()

        # Animation setup
        self.animation = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot)

    def init_plot(self):
        return self.line,

    def update_plot(self, frame):
        if not self.initial_values_recorded or len(self.time_stamps) < 2: # if self.initial_values_recorded and len(self.time_stamps) >= 2:
            return self.line,
        self.line.set_data(self.time_stamps, self.distance_z_values)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.01) 
        return self.line,

    def acceleration_increasing_flag(self):
        if self.linear_acceleration_z > 10.2:
            self.over_flag = True
    
    def acceleration_decreasing_flag(self):
        if self.linear_acceleration_z < 9.2:
            self.under_flag = True

    def imu_callback(self, imu_msg):
        self.linear_acceleration_z = imu_msg.linear_acceleration.z

        if 9.5 <= self.linear_acceleration_z <= 9.8:
            self.linear_acceleration_z = 9.6

        if not self.initial_values_recorded and abs(self.linear_acceleration_z - self.prev_linear_acceleration_z) > self.acceleration_threshold:
            self.record_initial_value(0)
            self.initial_values_recorded = True
            # self.prev_linear_acceleration_z = self.linear_acceleration_z

    def estimate_distance(self):
        adjusted_linear_acceleration_z = self.linear_acceleration_z - 9.6
        dt = 1.0 / rospy.get_param('~rate', 10.0)
        self.acceleration_values.append(adjusted_linear_acceleration_z)

        if abs(adjusted_linear_acceleration_z - self.prev_linear_acceleration_z) > self.acceleration_threshold:
            velocity_values = cumtrapz(self.acceleration_values, dx=dt, initial=0)
            distance_values = cumtrapz(velocity_values, dx=dt, initial=0)
            self.prev_linear_acceleration_z = adjusted_linear_acceleration_z
            timestamp = rospy.Time.now().to_sec()
            if self.initial_values_recorded:
                with open("/home/bottlegyu/xg6000_ws/src/mi_ros/data_record/9_to_1.txt", 'a') as file:
                    file.write("{},{}\n".format(timestamp, distance_values[-1]))
            return distance_values[-1]
        return None

    def record_initial_value(self, initial_position):
        timestamp = rospy.Time.now().to_sec()
        with open("/home/bottlegyu/xg6000_ws/src/mi_ros/data_record/9_to_1.txt", 'a') as file:
            file.write("{},{}\n".format(timestamp, initial_position))
        self.initial_values_recorded = True
                                        
    def classify_floor(self, distance_z, initial_position):
        # Define the valid floor range
        min_floor = 1
        max_floor = 9

        if  2.0 < distance_z < 4.5:
            initial_position += 1
        elif 4.5 <= distance_z < 8.5:
            initial_position += 2
        elif 8.5 <= distance_z < 12.5:
            initial_position += 3
        elif 12.5 <= distance_z < 16.5:
            initial_position += 4
        elif 16.5 <= distance_z < 20.0:
            initial_position += 5
        elif 20.0 <= distance_z < 24.0:
            initial_position += 6
        elif 24.0 <= distance_z < 28.0:
            initial_position += 7
        elif 28.0 <= distance_z < 1000.0:
            initial_position += 8
        elif -1.5 > distance_z > -4.5:
            initial_position -= 1
        elif -4.5 >= distance_z > -7.0:
            initial_position -= 2
        elif -7.0 >= distance_z > -11.0:
            initial_position -= 3
        elif -11.0 >= distance_z > -14.5:
            initial_position -= 4
        elif -14.5 >= distance_z > -19.0:
            initial_position -= 5
        elif -19.0 >= distance_z > -23.5:
            initial_position -= 6
        elif -23.5 >= distance_z > -28.0:
            initial_position -= 7
        elif -28.0 >= distance_z > -1000.0:
            initial_position -= 8

        # Ensure the result is within the valid floor range
        initial_position = max(min_floor, min(max_floor, initial_position))

        return initial_position
    
class FloorEstimation:
    def __init__(self):
        self.initial_position = int(input("Position_Floor: "))
        self.distance_estimation = DistanceEstimation() 

    def record_initial_value(self):
        self.distance_estimation.record_initial_value(self.initial_position)

def main():
    rospy.init_node('floor_estimation')
    floor_estimation = FloorEstimation()

    rospy.Subscriber('imu/data', Imu, floor_estimation.distance_estimation.imu_callback)
    floor_estimation.record_initial_value()

    rate = rospy.Rate(rospy.get_param('~rate', 10.0))
    while not rospy.is_shutdown():
        distance_z = floor_estimation.distance_estimation.estimate_distance()

        if distance_z is not None:
            rospy.loginfo("Distance_Z: %f", distance_z) 
            if floor_estimation.distance_estimation.over_flag == True and floor_estimation.distance_estimation.under_flag == True:
                print("over_flag: {}, under_flag: {}".format(floor_estimation.distance_estimation.over_flag, floor_estimation.distance_estimation.under_flag))
                rospy.loginfo_throttle(1, "over_flag: {}, under_flag: {}".format(floor_estimation.distance_estimation.over_flag, floor_estimation.distance_estimation.under_flag))
                
                floor_value = floor_estimation.distance_estimation.classify_floor(distance_z, floor_estimation.initial_position)
                if floor_value is not None:
                    print("파일에 기록 중...")
                    with open("/home/bottlegyu/xg6000_ws/src/mi_ros/data_record/floor_value.txt", 'w') as file:
                        file.write("Floor_Value: {}".format(floor_value))
                    print("기록 완료.")
            else:  
                timestamp = rospy.Time.now().to_sec()
                floor_estimation.distance_estimation.time_stamps.append(timestamp)
                floor_estimation.distance_estimation.distance_z_values.append(distance_z)

                floor_now = floor_estimation.distance_estimation.classify_floor(distance_z, floor_estimation.initial_position)
                rospy.loginfo("Position_floor: %d", floor_now)
                floor_estimation.distance_estimation.record_initial_value(floor_now)

        rate.sleep()

    plt.show()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

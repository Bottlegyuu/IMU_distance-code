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
        self.initial_values_recorded = False  # Flag to check if initial values have been recorded
        self.acceleration_threshold = acceleration_threshold
        # 플로팅을 위한 초기화
        self.time_stamps = []
        self.distance_z_values = []
        self.acceleration_values = []

        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Distance Z')
        self.ax.legend()

        self.acceleration_increasing = True

        self.animation = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot)

    def init_plot(self):
        return self.line,

    def update_plot(self, frame):
        if not self.is_initial_value_recorded or len(self.time_stamps) < 2:  # Exclude the first element from the plot
            return self.line,

        self.line.set_data(self.time_stamps, self.distance_z_values)
        self.ax.relim()
        self.ax.autoscale_view()

        plt.pause(0.01)  # Allow for dynamic updating

        return self.line,

    def imu_callback(self, imu_msg):
        self.linear_acceleration_z = imu_msg.linear_acceleration.z

        if not self.initial_values_recorded and abs(self.linear_acceleration_z - self.prev_linear_acceleration_z) > self.acceleration_threshold:
            # Record initial values only once when the acceleration exceeds the threshold
            self.record_initial_value(0)  # Assuming initial_position is 0
            self.initial_values_recorded = True

        if self.linear_acceleration_z > self.prev_linear_acceleration_z:
            self.acceleration_increasing = True
        elif self.linear_acceleration_z < self.prev_linear_acceleration_z:
            self.acceleration_increasing = False

    def estimate_distance(self):
        adjusted_linear_acceleration_z = self.linear_acceleration_z - 9.6

        dt = 1.0 / rospy.get_param('~rate', 10.0)
        self.acceleration_values.append(adjusted_linear_acceleration_z)

        # Check if the absolute change in acceleration exceeds the threshold
        if abs(adjusted_linear_acceleration_z - self.prev_linear_acceleration_z) > self.acceleration_threshold:
            # scipy.integrate.cumtrapz = (y, x=none, dx=1.0, intitial)
            # y = 적분하려는 데이터, x = 선택적 매개변수, dx = 적분 구간, intitial = 초기 적분 값
            # 가속도 -> 속도로 적분
            velocity_values = cumtrapz(self.acceleration_values, dx=dt, initial=0)
            # 속도 -> 거리로 적분
            distance_values = cumtrapz(velocity_values, dx=dt, initial=0)

            # 가속도 업데이트
            self.prev_linear_acceleration_z = adjusted_linear_acceleration_z

            timestamp = rospy.Time.now().to_sec()

            # Record the distance values only after the initial user input has been processed
            if self.is_initial_value_recorded:
                with open("/home/duho/imu_ws/src/mi_ros/data_record/9_to_1.txt", 'a') as file:
                    file.write("{},{}\n".format(timestamp, distance_values[-1]))

            return distance_values[-1]

        return None  # If the threshold is not exceeded, return None

    def record_initial_value(self, initial_position):
        # Record the initial user input value
        timestamp = rospy.Time.now().to_sec()
        with open("/home/duho/imu_ws/src/mi_ros/data_record/9_to_1.txt", 'a') as file:
            file.write("{},{}\n".format(timestamp, initial_position))

        self.is_initial_value_recorded = True
                                        
    def classify_floor(self, distance_z, initial_position):
        # Define the valid floor range
        min_floor = 1
        max_floor = 9

        if 1.5 < distance_z < 4.2:
            initial_position += 1
        elif 4.2 <= distance_z < 9.8:
            initial_position += 2
        elif 9.8 <= distance_z < 14.8:
            initial_position += 3
        elif 14.8 <= distance_z < 19.2:
            initial_position += 4
        elif 19.2 <= distance_z < 24.2:
            initial_position += 5
        elif 24.2 <= distance_z < 29.2:
            initial_position += 6
        elif 29.2 <= distance_z < 34.2:
            initial_position += 7
        elif 34.2 <= distance_z < 39.2:
            initial_position += 8
        elif -1.5 > distance_z > -4.2:
            initial_position -= 1
        elif -4.2 >= distance_z > -8.0:
            initial_position -= 2
        elif -8.0 >= distance_z > -13.5:
            initial_position -= 3
        elif -13.5 >= distance_z > -17.0:
            initial_position -= 4
        elif -17.0 >= distance_z > -20.5:
            initial_position -= 5
        elif -20.5 >= distance_z > -24.0:
            initial_position -= 6
        elif -24.0 >= distance_z > -27.5:
            initial_position -= 7
        elif -27.5 >= distance_z > -31.0:
            initial_position -= 8

        # Ensure the result is within the valid floor range
        initial_position = max(min_floor, min(max_floor, initial_position))

        return initial_position
    
class FloorEstimation:
    def __init__(self):
        self.initial_position = int(input("Position_Floor: "))
        self.distance_estimation = DistanceEstimation()

    def record_initial_value(self):
        # Record the initial user input value
        self.distance_estimation.record_initial_value(self.initial_position)

def main():
    rospy.init_node('floor_estimation')
    floor_estimation = FloorEstimation()

    rospy.Subscriber('imu/data', Imu, floor_estimation.distance_estimation.imu_callback)

    # Record the initial user input value
    floor_estimation.record_initial_value()

    rate = rospy.Rate(rospy.get_param('~rate', 10.0))
    while not rospy.is_shutdown():
        distance_z = floor_estimation.distance_estimation.estimate_distance()

        if distance_z is not None:
            rospy.loginfo("Distance_Z: %f", distance_z)

            # 플로팅 값 업데이트
            timestamp = rospy.Time.now().to_sec()
            floor_estimation.distance_estimation.time_stamps.append(timestamp)
            floor_estimation.distance_estimation.distance_z_values.append(distance_z)

            # Capture the returned value from classify_floor
            floor_now = floor_estimation.distance_estimation.classify_floor(distance_z, floor_estimation.initial_position)
            rospy.loginfo("Position_floor: %d", floor_now)

            # Record only the initial_position value
            floor_estimation.distance_estimation.record_initial_value(floor_now)

            # Here you can use the floor_now value as needed in the rest of your code.

        rate.sleep()

    plt.show()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

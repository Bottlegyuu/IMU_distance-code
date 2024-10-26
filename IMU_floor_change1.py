#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import cumtrapz

class DistanceEstimation:
    def __init__(self, acceleration_threshold=0.55):
        self.linear_acceleration_z = 9.81
        self.prev_linear_acceleration_z = self.linear_acceleration_z
        self.is_initial_value_recorded = False
        self.initial_values_recorded = False  # Flag to check if initial values have been recorded
        self.acceleration_threshold = acceleration_threshold
        self.reset_distance_cycle = True

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
            self.initial_values_recorded = True

        if self.linear_acceleration_z > self.prev_linear_acceleration_z:
            self.acceleration_increasing = True
        elif self.linear_acceleration_z < self.prev_linear_acceleration_z:
            self.acceleration_increasing = False

    def reset_distance(self):
        self.distance_z_values = []
        self.acceleration_values = []
        self.reset_distance_cycle = True

    def estimate_distance(self):
        adjusted_linear_acceleration_z = self.linear_acceleration_z - 9.6

        dt = 1.0 / rospy.get_param('~rate', 10.0)
        self.acceleration_values.append(adjusted_linear_acceleration_z)
        
        acceleration_threshold = 0.2
        if abs(adjusted_linear_acceleration_z) < acceleration_threshold:
            return 0  # 임계값 미만일 경우 거리 계산을 수행하지 않음
        
        if len(self.acceleration_values) > 10:  # 예를 들어, 최근 10개의 샘플을 확인
            if all(abs(a) < self.acceleration_threshold for a in self.acceleration_values[-10:]):
                self.reset_distance()  # 멈춤 상태가 감지되면 거리 측정을 리셋
                return 0
        if self.reset_distance_cycle:
            self.reset_distance_cycle = False  # 새로운 사이클 시작 표시 해제
            return 0  # 새로운 사이클에서의 첫 거리는 0으로 시작

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

    def record_initial_value(self, initial_position):
        # 이 메서드는 초기 층수만 기록해야 하므로, is_initial_value_recorded 플래그를 확인
        if not self.is_initial_value_recorded:
            timestamp = rospy.Time.now().to_sec()
            with open("/home/duho/imu_ws/src/mi_ros/data_record/9_to_1.txt", 'a') as file:
                file.write("{},{}\n".format(timestamp, initial_position))
            self.is_initial_value_recorded = True

        self.is_initial_value_recorded = True
                                        
    def classify_floor(self, distance_z, initial_position):
        # Define the valid floor range
        min_floor = 1
        max_floor = 9
        floor_height = 4  # 한 층의 높이를 4미터로 설정

        # 층수를 계산하기 위해 거리를 층 높이로 나눔
        floor_change = int(distance_z / floor_height)
        new_position = initial_position + floor_change

        # Ensure the result is within the valid floor range
        new_position = max(min_floor, min(max_floor, new_position))

        return new_position
    
class FloorEstimation:
    def __init__(self):
        self.initial_position = int(input("Position_Floor: "))
        self.distance_estimation = DistanceEstimation()

    def record_initial_value(self):
        # Record the initial user input value
        self.distance_estimation.record_initial_value(self.initial_position)

    def process_distance(self, distance_z):
        if distance_z is not None and distance_z != 0:
            floor_now = self.distance_estimation.classify_floor(distance_z, self.initial_position)
            rospy.loginfo("Position_floor: %d", floor_now)

            # 층간 이동이 감지되면 거리 측정 리셋
            if floor_now != self.initial_position:
                self.distance_estimation.reset_distance()
                self.initial_position = floor_now  # 층수 업데이트

def main():
    rospy.init_node('floor_estimation')
    floor_estimation = FloorEstimation()

    rospy.Subscriber('imu/data', Imu, floor_estimation.distance_estimation.imu_callback)

    # Record the initial user input value
    floor_estimation.record_initial_value()

    rate = rospy.Rate(rospy.get_param('~rate', 10.0))
    while not rospy.is_shutdown():
        distance_z = floor_estimation.distance_estimation.estimate_distance()
        floor_estimation.process_distance(distance_z)

        if distance_z is not None and distance_z != 0:
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

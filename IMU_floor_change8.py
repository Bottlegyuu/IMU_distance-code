#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import cumtrapz
import time

class DistanceEstimation:
    def __init__(self, acceleration_threshold=0.5, recording_threshold=5.0):
        self.linear_acceleration_z = 9.81
        self.prev_linear_acceleration_z = self.linear_acceleration_z
        self.initial_values_recorded = False  # 초기 값이 기록되었는지 확인하기 위한 flag
        self.acceleration_threshold = acceleration_threshold

        self.recording_threshold = recording_threshold
        self.start_recording_time = None
        self.recording_floor_values = []

        # 플로팅을 위한 초기화
        self.time_stamps = []
        self.distance_z_values = []
        self.acceleration_values = []

        # 그림에 대한 새 그림 및 축을 생성
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Distance Z')
        self.ax.legend()

        # linear_acceleration_z가 증가하고 있는지를 나타내는 flag
        self.acceleration_increasing = True
        # init_plot 함수에서 처리하는 각 프레임에 대해 update_plot 함수를 호출하는 애니메이션 설정
        self.animation = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot)

    def init_plot(self):
        return self.line,

    def update_plot(self, frame):   #각 프레임에서 plot 업데이트
        # 초기값이 기록되거나 타임스탬프가 두 개 미만인지 확인하고, true이면 plot 업데이트 안함 -> input 값을 제외시키기 위해
        if not self.is_initial_value_recorded or len(self.time_stamps) < 2:
            return self.line,
        # 객체 data를 업데이트
        self.line.set_data(self.time_stamps, self.distance_z_values)
        self.ax.relim()
        self.ax.autoscale_view()

        plt.pause(0.01) 

        return self.line,

    def imu_callback(self, imu_msg):
        self.linear_acceleration_z = imu_msg.linear_acceleration.z

        # 초기값이 기록되지 않았는지 및 현재 가속도 값과 이전 값이 임계값을 넘는지 확인
        if not self.initial_values_recorded and abs(self.linear_acceleration_z - self.prev_linear_acceleration_z) > self.acceleration_threshold:
            # 임계 값을 초과하면 0으로 가정된 record_initial_value 호출
            self.record_initial_value(0)
            # 초기값 기록
            self.initial_values_recorded = True

        # 임계 값을 충족하면 초기 값을 기록하여 현재와 이전 값을 기준으로 가속도가 증가하는지 감소하는지 결정
        if self.linear_acceleration_z > self.prev_linear_acceleration_z:
            self.acceleration_increasing = True
        elif self.linear_acceleration_z < self.prev_linear_acceleration_z:
            self.acceleration_increasing = False

        if 9.75 < self.linear_acceleration_z < 9.5:
            if self.start_recording_time is None:
                self.start_recording_time = time.time()
            elif time.time() - self.start_recording_time > self.recording_threshold:
                # Record the floor value
                floor_value = self.classify_floor(self.linear_acceleration_z, 0)
                self.recording_floor_values.append(floor_value)
                rospy.loginfo("Recording Floor Value: %d", floor_value)
        else:
            # Reset the recording timer if the acceleration is not in the desired range
            self.start_recording_time = None

    def estimate_distance(self):
        # 가속도 조정 (노이즈 방지)
        adjusted_linear_acceleration_z = self.linear_acceleration_z - 9.6
        # 속도 매개 변수를 기준으로 시간 단계를 계산
        dt = 1.0 / rospy.get_param('~rate', 10.0)
        self.acceleration_values.append(adjusted_linear_acceleration_z)

        # 조정된 가속도에 대한 절대 변화가 임계값을 초과하는 지 및 방향이 바뀌었는지 확인
        if abs(adjusted_linear_acceleration_z - self.prev_linear_acceleration_z) > self.acceleration_threshold:

            # scipy.integrate.cumtrapz = (y, x=none, dx=1.0, intitial)
            # y = 적분하려는 데이터, x = 선택적 매개변수, dx = 적분 구간, intitial = 초기 적분 값
            # 가속도 -> 속도로 적분
            velocity_values = cumtrapz(self.acceleration_values, dx=dt, initial=9.6)
            # 속도 -> 거리로 적분
            distance_values = cumtrapz(velocity_values, dx=dt, initial=0)

            # 가속도 업데이트
            self.prev_linear_acceleration_z = adjusted_linear_acceleration_z

            timestamp = rospy.Time.now().to_sec()

            # 초기 입력이 처리되면 거리를 해당 경로에 기록
            if self.is_initial_value_recorded:
                with open("/home/duho/imu_ws/src/mi_ros/data_record/9_to_1.txt", 'a') as file:
                    file.write("{},{}\n".format(timestamp, distance_values[-1]))
                    # Check if there are any recorded floor values during acceleration in the desired range
            if self.recording_floor_values:
                # Save the recorded floor values to a file
                with open("/home/duho/imu_ws/src/mi_ros/data_record/floor_values.txt", 'w') as file:
                    for floor_value in self.recording_floor_values:
                        file.write("Floor_Value: {}\n".format(floor_value))
                # Clear the recorded floor values
                self.recording_floor_values = []

            return distance_values[-1]

        return None  # 임계값을 넘지않으면 return하지 않는다.

    def record_initial_value(self, initial_position):   # 초기 입력을 타임스탬프와 함께 해당 경로에 기록하는 함수

        timestamp = rospy.Time.now().to_sec()
        with open("/home/duho/imu_ws/src/mi_ros/data_record/9_to_1.txt", 'a') as file:
            file.write("{},{}\n".format(timestamp, initial_position))

        self.is_initial_value_recorded = True
                                        
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
        self.initial_position = int(input("Position_Floor: "))  # 사용자가 현재 위치하는 층을 입력한다.
        self.distance_estimation = DistanceEstimation() # 거리 데이터는 DistanceEstimation 클래스에서 받아온다. 

    def record_initial_value(self):
        # 초기 값 기록
        self.distance_estimation.record_initial_value(self.initial_position)

def main():
    rospy.init_node('floor_estimation')
    floor_estimation = FloorEstimation()

    rospy.Subscriber('imu/data', Imu, floor_estimation.distance_estimation.imu_callback)

    # 초기 값 기록
    floor_estimation.record_initial_value()

    rate = rospy.Rate(rospy.get_param('~rate', 10.0))
    while not rospy.is_shutdown():
        distance_z = floor_estimation.distance_estimation.estimate_distance()

        if distance_z is not None:  # distance_z 값이 존재한다면 rospy.loginfo 찍어내기
            rospy.loginfo("Distance_Z: %f", distance_z) 

            # 플로팅 값 업데이트
            timestamp = rospy.Time.now().to_sec()
            floor_estimation.distance_estimation.time_stamps.append(timestamp)
            floor_estimation.distance_estimation.distance_z_values.append(distance_z)

            # classify_floor에서 데이터 값 받아오기
            floor_now = floor_estimation.distance_estimation.classify_floor(distance_z, floor_estimation.initial_position)
            rospy.loginfo("Position_floor: %d", floor_now)

            # 초기 값에 현재 층 데이터 값 기록하기
            floor_estimation.distance_estimation.record_initial_value(floor_now)
                        # Count and record floor value
            floor_value = floor_estimation.distance_estimation.classify_floor(distance_z, floor_estimation.initial_position)
            if floor_value is not None:
                with open("/home/duho/imu_ws/src/mi_ros/data_record/floor_value.txt", 'w') as file:
                    file.write("Floor_Value: {}".format(floor_value))

            # Here you can use the floor_now value as needed in the rest of your code.

        rate.sleep()


    plt.show()

if __name__ == '__main__':  # 스크립트가 메인 프로그램으로 실행되고 있는지 확인하는 코드
    try:
        main()  # 메인 함수 호출
    except rospy.ROSInterruptException: # ROS노드를 수동으로 중지하려고 할때 발생
        pass

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
        self.initial_values_recorded = False
        self.acceleration_threshold = acceleration_threshold

        self.recording_threshold = recording_threshold
        self.start_recording_time = None
        self.recording_floor_values = []

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

        self.target_floor = int(input("목표 층 입력: "))  # 사용자로부터 목표 층 입력
        self.integration_start_time = None  # 적분 시작 시간
        self.integrate_for_duration = 100.0  # 100초 동안 적분

    def init_plot(self):
        return self.line,

    def update_plot(self, frame):
        if not self.is_initial_value_recorded or len(self.time_stamps) < 2:
            return self.line,
        self.line.set_data(self.time_stamps, self.distance_z_values)
        self.ax.relim()
        self.ax.autoscale_view()

        plt.pause(0.01)

        return self.line,

    def imu_callback(self, imu_msg):
        self.linear_acceleration_z = imu_msg.linear_acceleration.z

        if not self.initial_values_recorded and abs(self.linear_acceleration_z - self.prev_linear_acceleration_z) > self.acceleration_threshold:
            self.record_initial_value(0)
            self.initial_values_recorded = True

        if self.linear_acceleration_z > self.prev_linear_acceleration_z:
            self.acceleration_increasing = True
        elif self.linear_acceleration_z < self.prev_linear_acceleration_z:
            self.acceleration_increasing = False

        if 9.75 < self.linear_acceleration_z < 9.5:
            if self.start_recording_time is None:
                self.start_recording_time = time.time()
            elif time.time() - self.start_recording_time > self.recording_threshold:
                floor_value = self.classify_floor(self.linear_acceleration_z, 0)
                self.recording_floor_values.append(floor_value)
                rospy.loginfo("층 값 기록 중: %d", floor_value)
        else:
            self.start_recording_time = None

    def estimate_distance(self):
        adjusted_linear_acceleration_z = self.linear_acceleration_z - 9.6
        dt = 1.0 / rospy.get_param('~rate', 10.0)
        self.acceleration_values.append(adjusted_linear_acceleration_z)

        if abs(adjusted_linear_acceleration_z - self.prev_linear_acceleration_z) > self.acceleration_threshold:
            velocity_values = cumtrapz(self.acceleration_values, dx=dt, initial=9.6)
            distance_values = cumtrapz(velocity_values, dx=dt, initial=0)

            self.prev_linear_acceleration_z = adjusted_linear_acceleration_z

            timestamp = rospy.Time.now().to_sec()

            if self.is_initial_value_recorded:
                with open("/home/duho/imu_ws/src/mi_ros/data_record/9_to_1.txt", 'a') as file:
                    file.write("{},{}\n".format(timestamp, distance_values[-1]))

            if self.recording_floor_values:
                with open("/home/duho/imu_ws/src/mi_ros/data_record/floor_values.txt", 'w') as file:
                    for floor_value in self.recording_floor_values:
                        file.write("층 값: {}\n".format(floor_value))
                self.recording_floor_values = []

            return distance_values[-1]

        return None

    def record_initial_value(self, initial_position):
        timestamp = rospy.Time.now().to_sec()
        with open("/home/duho/imu_ws/src/mi_ros/data_record/9_to_1.txt", 'a') as file:
            file.write("{},{}\n".format(timestamp, initial_position))

        self.is_initial_value_recorded = True

    def classify_floor(self, distance_z, initial_position):
        # 유효한 층 범위 정의
        min_floor = 1
        max_floor = 9

        if 2.0 < distance_z < 4.5:
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

        # 결과가 유효한 층 범위 내에 있는지 확인
        initial_position = max(min_floor, min(max_floor, initial_position))

        return initial_position

    def reset_integration(self):
        self.integration_start_time = None
        self.acceleration_values = []

class FloorEstimation:
    def __init__(self):
        self.initial_position = int(input("현재 위치 층 입력: "))
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
            rospy.loginfo("거리_Z: %f", distance_z)

            timestamp = rospy.Time.now().to_sec()
            floor_estimation.distance_estimation.time_stamps.append(timestamp)
            floor_estimation.distance_estimation.distance_z_values.append(distance_z)

            floor_now = floor_estimation.distance_estimation.classify_floor(distance_z, floor_estimation.initial_position)
            rospy.loginfo("현재 층: %d", floor_now)

            floor_estimation.distance_estimation.record_initial_value(floor_now)

            floor_value = floor_estimation.distance_estimation.classify_floor(distance_z, floor_estimation.initial_position)
            if floor_value is not None:
                with open("/home/duho/imu_ws/src/mi_ros/data_record/floor_value.txt", 'w') as file:
                    file.write("층 값: {}".format(floor_value))

        rate.sleep()

    plt.show()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

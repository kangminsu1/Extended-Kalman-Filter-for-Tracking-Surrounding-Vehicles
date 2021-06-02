from Extended_KF import *
import time
from datetime import datetime
import warnings
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import threading
import math
warnings.filterwarnings("ignore")

# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@for only lidar for SCI@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
class EKF_Processor:
    def __init__(self):
        self.LIDAR = {}

        self.predict_info_x = {} # State Vector
        self.predict_info_p = {} # State Covariance

        self.DT = None

    # Plotting My Vehicles----------------------------------
    def plot_car(self, x, y, yaw=0.0, WIDTH=1.53, LENGTH=4.37, truckcolor="-k", BACKTOWHEEL=1.0):

        outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                            [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])

        outline = (outline.T.dot(Rot1)).T

        outline[0, :] += x
        outline[1, :] += y

        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), truckcolor)
        plt.plot(x, y, "*", label="My Vehicle")

        return outline

    # EKF initialize---------------------
    def ekf_initialize(self, key, value):
        predict_x, predict_p = Init_EKF(value, self.DT)
        self.predict_info_x[key] = predict_x
        self.predict_info_p[key] = predict_p

    # EKF prediction---------------------
    def ekf_prediction(self, key, value):
        predict_x, predict_p = EKF(self.predict_info_x[key], self.predict_info_p[key], value[0:2], self.DT)
        self.predict_info_x[key] = predict_x
        self.predict_info_p[key] = predict_p

    # EKF Tracking-----------------------
    def tracking(self):
        measurement = self.LIDAR

        compare_key = []
        pred_key = list(self.predict_info_x.keys())

        # 여기는 내가 tracking 하고자 하는 차량이 어디 있는지 확인하는 부
        # 먼저 측정된 GT LIDAR DATA 가져옴
        temp_gt = {}
        for key, value in measurement.items():
            Simliarity = np.rad2deg(math.atan2(value[0], value[1])) # 상대 차량의 x, y를 atan2로 하여금 내 차량과의 상대각을 얻는 과정
            temp_gt[key] = Simliarity

        # 그 다음 prediction한 좌표 값을 불러와서 서로 비교하기 (Cosine 유사도 검출 기법) --> 추후 Iou Bounding Box로 바꾸는 것을 추천함
        # Cosine 유사도를 쓴 이유는 각 차량의 edge point를 쓰지 않고 오로지 x, y 좌표 값만을 사용하기 때문
        for key, value in self.predict_info_x.items():
            Simliarity = np.rad2deg(math.atan2(value[0], value[1])) #이것 또한 위와 마찬가지
            # 하나씩 비교하기
            P_key_lists = []
            # 만약 비교하고자 하는 차량이 prediction한 차량과의 차이가 20도 이하로 나타날 경우 그 값을 가져온다.
            for temp_key, temp_value in temp_gt.items():
                difference = abs(abs(Simliarity) - abs(temp_value))
                if difference < 20:
                    # GT key data 들이 들어온다
                    P_key_lists.append(temp_key)

            # 0보다 클 때 prediction 작업
            if len(P_key_lists) > 0:

                # Euclidien 거리 측정 기법---------------
                results = {}
                for key_p in P_key_lists:
                    results[key_p] = np.hypot(measurement[key_p][0] - value[0], measurement[key_p][0] - value[1])
                # 비교해서 가장 적합한 측정 lidar 아이디 append

                # 이 POINT가 실제 predict_info에 있는지 검수 작업
                while True:
                    if len(results) == 0:
                        break

                    POINT = min(results.keys(), key=lambda k: results[k])
                    if POINT in pred_key:
                        break
                    else:
                        del results[POINT]
                # Euclidien 거리 측정 기법---------------

                compare_key.append(POINT)
                # 만약 결과 값이 0보다 크다면 EKF 계속 시작
                if len(results) > 0:
                    self.ekf_prediction(POINT, measurement[POINT])

        # 모니터링-----------------------------
        comp_key = list(set(compare_key))
        for POINT in comp_key:
            text2 = "Vehicle ID: " + str(POINT)
            # Plotting prediction
            plt.plot(measurement[POINT][0], measurement[POINT][1], '*', label=text2)
        # 모니터링-----------------------------

        measurement_keys = temp_gt.keys()
        # 기존에 측정된 차량 데이터와 이전에 prediction된 차량의 좌표를 비교해서 남은 차량 데이터들은 delete
        residual = list(set(compare_key) - set(measurement_keys))
        for id in residual:
            del self.predict_info_x[id]
            del self.predict_info_p[id]

        # 현재 측정된 차량 데이터와 이전에 prediction된 차량 좌표를 비교해서 남은 차량 데이터들은 append
        others = list(set(measurement_keys) - set(compare_key))  # Extract another velocity
        for id in others:
            self.ekf_initialize(id, measurement[id])


    def get_ekf(self, Vehicle_heading, other_info_dict, ALL_STOP):
        matplotlib.use('TkAgg')

        while True:
            if ALL_STOP == 1:
                break
            # 모니터링-----------------------------
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            yaw = (Vehicle_heading + 90) * math.pi / 180 # (heading + 90) 가변 가능
            self.plot_car(0.0, 0.0, yaw)
            # 모니터링-----------------------------

            # 외부 차량 데이터 불러오기-----------------------
            Lidar_measurement = {}
            for key, value in other_info_dict.items():
                Lidar_measurement[key] = [value[0], value[1], value[2]] # x, y, velocity

            # For DT---------------------
            self.DT = 0.1
            # For DT---------------------

            self.LIDAR = Lidar_measurement

            # 만약 처임이라면 측정된 데이터로 initialize----------------
            if len(self.predict_info_x) == 0:
                for key, value in Lidar_measurement.items():
                    self.ekf_initialize(key, value)
            else:
                # 아니면 tracking 시작
                self.tracking()

            plt.title("Lidar Detection and Tracking [LiDAR]")
            plt.legend()
            plt.xlim(-50, 50)
            plt.ylim(-50, 50)
            plt.pause(0.001)
        print("LiDAR Clear")
        print("EKF Processor Clear")

if __name__ == "__main__":

    Vehicle_heading = 0.0
    other_info_dict = {'1': [-10.0, 10.0, 0.0], '2': [10.0, -10.0, 0.0]} #x, y, velocity [Essential]
    ALL_STOP = 0 # for break while

    ekf = EKF_Processor()
    Sensor_Fusion_and_Tracking = threading.Thread(target=ekf.get_ekf, args=(Vehicle_heading, other_info_dict, ALL_STOP))
    Sensor_Fusion_and_Tracking.start()

    # Example for realtime checking
    heading = 0.0
    times = 0.0
    DT = 0.1
    SIM_TIME = 50.0

    while SIM_TIME >= times:
        if ALL_STOP == 1:
            break
        time.sleep(0.1)
        times += DT
        heading += 0.1
        for key, value in other_info_dict.items():

            yaw = np.deg2rad(heading)
            x = value[0] * math.cos(yaw) + value[1] * math.sin(yaw)
            y = value[0] * -math.sin(yaw) + value[1] * math.cos(yaw)
            other_info_dict[key] = [x, y, 0.0]
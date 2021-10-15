import numpy as np
from scipy.spatial.transform import Rotation as Rot
import scipy.linalg
import math

def compute_F_and_Q(DT):
    __noise_ax = 9
    __noise_ay = 9

    __F = np.array([
        [1.0, 0.0, DT, 0.0],
        [0.0, 1.0, 0.0, DT],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Setting Q
    DT_2 = DT ** 2
    DT_3 = DT ** 3
    DT_4 = DT ** 4

    E11 = DT_4 * __noise_ax / 4
    E13 = DT_3 * __noise_ax / 2
    E22 = DT_4 * __noise_ay / 4
    E24 = DT_3 * __noise_ay / 2
    E31 = DT_3 * __noise_ax / 2
    E33 = DT_2 * __noise_ax
    E42 = DT_3 * __noise_ay / 2
    E44 = DT_2 * __noise_ay

    __Q = np.array([[E11, 0.0, E13, 0.0],
                    [0.0, E22, 0.0, E24],
                    [E31, 0.0, E33, 0.0],
                    [0.0, E42, 0.0, E44]])

    return __F, __Q

# State Vector 예측 부
def predict(x, F, P, Q):
    state_x = F @ x
    state_p = (F @ P @ F.T) + Q

    return state_x, state_p

# State Vector 업데이트 부
def update(X, P, Z):
    __HL = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0]])

    __RL = np.array([[0.2, 0],
                     [0, 0.2]])

    __xI = np.identity(4)

    Y = Z - __HL @ X

    PHLT = P @ __HL.T
    S = np.array(__HL @ PHLT + __RL)
    K = PHLT @ np.linalg.inv(S)

    X += K @ Y

    P = (__xI - K @ __HL) @ P

    return X, P

# 초기 State Vector 설정 부 [x, y, yaw, velocity] 
def Init_EKF(current_info, DT):
    __x = np.array([[current_info[0], current_info[1], 0.0, 0.0]]).T

    __P = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1000, 0],
                    [0, 0, 0, 1000]])

    __F, __Q = compute_F_and_Q(DT)

    __x, __P = predict(__x, __F, __P, __Q)

    __x = np.round(__x, 4)

    return __x, __P

# Prediction과 Update를 수행하는 부
def EKF(X, P, measurement, DT):
    __z = np.array([[measurement[0], measurement[1]]]).T

    __F, __Q = compute_F_and_Q(DT)

    X, P = predict(X, __F, P, __Q)

    X, P = update(X, P, __z)

    X = np.round(X, 4)

    return X, P

"""
2_motion_sensor / 문제 3

TODO 기반:
- theta 업데이트
- dx, dy 계산

예상 출력 설명:
- x_list, y_list, theta_list가 출력된다.
- 회전하면서 누적 이동한 궤적이 나와야 한다.
"""

import numpy as np


DT = 0.5
SPEED = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)
YAW_RATE = np.array([0.0, 0.2, 0.2, 0.0], dtype=float)


def calculate_dr_trajectory(speed: np.ndarray, yaw_rate: np.ndarray, dt: float):
    x, y, theta = 0.0, 0.0, 0.0
    x_list, y_list, theta_list = [x], [y], [theta]

    for i in range(len(speed)):
        theta += 0  # TODO: theta 업데이트 식 완성
        dx = 0  # TODO: speed, theta, dt로 dx 계산
        dy = 0  # TODO: speed, theta, dt로 dy 계산
        x += dx
        y += dy
        x_list.append(x)
        y_list.append(y)
        theta_list.append(theta * 180 / np.pi)  # hidden bug

    return x_list, y_list, theta_list


def main() -> None:
    x_list, y_list, theta_list = calculate_dr_trajectory(SPEED, YAW_RATE, DT)
    print("x_list =", np.round(x_list, 3).tolist())
    print("y_list =", np.round(y_list, 3).tolist())
    print("theta_list =", np.round(theta_list, 3).tolist())


if __name__ == "__main__":
    main()

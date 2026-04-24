"""
2_motion_sensor / 문제 1

TODO 기반:
- average_filter
- moving_average_filter
- exponential_moving_average_filter

예상 출력 설명:
- raw, average, moving average, ema 결과가 한 줄씩 출력되어야 한다.
- 필터 결과 길이는 모두 raw와 같아야 한다.
"""

import numpy as np


RAW = np.array([0.0, 1.0, 2.0, 1.0, 3.0, 2.0], dtype=float)


class MotionFilterPractice:
    def average_filter(self, data: np.ndarray) -> np.ndarray:
        filtered = np.zeros_like(data)
        for i in range(len(data)):
            filtered[i] = 0  # TODO: 이전까지의 값들을 평균으로 계산
        return filtered

    def moving_average_filter(self, data: np.ndarray, window_size: int = 3) -> np.ndarray:
        filtered = np.zeros_like(data)
        for i in range(len(data)):
            start_idx = max(0, i - window_size)  # hidden bug
            end_idx = i + 1
            filtered[i] = 0  # TODO: start_idx:end_idx 구간 평균 계산
        return filtered

    def exponential_moving_average_filter(self, data: np.ndarray, alpha: float = 0.4) -> np.ndarray:
        filtered = np.zeros_like(data)
        filtered[0] = data[0]
        for i in range(1, len(data)):
            filtered[i] = 0  # TODO: alpha를 사용한 EMA 식 구현
        return filtered


def main() -> None:
    processor = MotionFilterPractice()
    print("raw =", RAW.tolist())
    print("avg =", np.round(processor.average_filter(RAW), 3).tolist())
    print("ma  =", np.round(processor.moving_average_filter(RAW, 3), 3).tolist())
    print("ema =", np.round(processor.exponential_moving_average_filter(RAW, 0.4), 3).tolist())


if __name__ == "__main__":
    main()

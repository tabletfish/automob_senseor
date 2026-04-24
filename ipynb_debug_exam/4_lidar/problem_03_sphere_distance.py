"""
4_lidar / 문제 3

TODO 기반:
- custom_compute_sphere_distances

예상 출력 설명:
- 중심에서 반지름만큼 떨어진 점은 거리 0에 가까워야 한다.
"""

import numpy as np


POINTS = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0], [1.0, 2.0, 2.0]], dtype=float)
CENTER = np.array([0.0, 0.0, 0.0], dtype=float)
RADIUS = 2.0


def custom_compute_sphere_distances(points: np.ndarray, center: np.ndarray, radius: float) -> np.ndarray:
    distances_to_center = 0  # TODO: 각 포인트와 중심점 사이의 거리 계산
    distances = np.abs(distances_to_center + radius)  # hidden bug
    return distances


def main() -> None:
    distances = custom_compute_sphere_distances(POINTS, CENTER, RADIUS)
    print("sphere_distances =", np.round(distances, 3).tolist())


if __name__ == "__main__":
    main()

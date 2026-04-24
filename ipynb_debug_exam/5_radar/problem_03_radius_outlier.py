"""
5_radar / 문제 3

TODO 기반:
- radius outlier removal의 핵심 조건

예상 출력 설명:
- inlier_indices와 outlier_indices가 출력된다.
- 가까운 점이 많은 포인트만 인라이어가 되어야 한다.
"""

import numpy as np


POINTS = np.array([[0.0, 0.0], [0.1, 0.1], [0.2, 0.0], [3.0, 3.0], [10.0, 10.0]], dtype=float)


def custom_radius_outlier_removal(points: np.ndarray, radius: float, min_nb_points: int):
    inlier_indices = []
    outlier_indices = []

    for i in range(len(points)):
        distances = np.linalg.norm(points - points[i], axis=1)
        neighbors_in_radius = np.sum(distances <= radius) - 1
        if neighbors_in_radius >= min_nb_points:  # TODO
            outlier_indices.append(i)  # hidden bug
        else:
            inlier_indices.append(i)

    return inlier_indices, outlier_indices


def main() -> None:
    inliers, outliers = custom_radius_outlier_removal(POINTS, radius=0.25, min_nb_points=2)
    print("inlier_indices =", inliers)
    print("outlier_indices =", outliers)


if __name__ == "__main__":
    main()

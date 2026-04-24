"""
4_lidar / 문제 1

TODO 기반:
- custom_extract_roi
- custom_brute_force_search

예상 출력 설명:
- ROI 포인트 개수
- query에 가장 가까운 k개 인덱스와 거리
"""

import numpy as np


POINTS = np.array(
    [[0.0, 0.0, 0.0], [1.0, 1.0, 0.2], [2.0, -1.0, 0.1], [5.0, 5.0, 0.0], [-1.0, 0.5, 0.0]],
    dtype=float,
)


def custom_extract_roi(points: np.ndarray, x_range, y_range, z_range):
    x_mask = (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1])  # TODO
    y_mask = (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1])  # TODO
    z_mask = (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])  # TODO
    roi_mask = x_mask | y_mask | z_mask  # hidden bug
    return points[roi_mask], points[~roi_mask]


def custom_brute_force_search(points: np.ndarray, query: np.ndarray, k: int):
    distances = np.zeros(len(points))  # TODO: 모든 포인트와의 거리 계산
    sorted_indices = np.argsort(distances)  # TODO
    k_indices = sorted_indices[:k]  # TODO
    return k_indices, distances[k_indices]


def main() -> None:
    roi, outliers = custom_extract_roi(POINTS, (-0.5, 2.5), (-1.5, 1.5), (-0.1, 0.3))
    indices, distances = custom_brute_force_search(roi, np.array([0.0, 0.0, 0.0]), 2)
    print("roi_count =", len(roi), "outlier_count =", len(outliers))
    print("indices =", indices.tolist())
    print("distances =", np.round(distances, 3).tolist())


if __name__ == "__main__":
    main()

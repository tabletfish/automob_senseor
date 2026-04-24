"""
4_lidar / 문제 2

TODO 기반:
- custom_fit_line_from_samples
- custom_compute_line_distances

예상 출력 설명:
- line_params는 [a, b] 꼴로 출력된다.
- distances는 직선에 가까운 점일수록 작아야 한다.
"""

import numpy as np


SAMPLES = np.array([[0.0, 1.0], [1.0, 3.0], [2.0, 5.0], [3.0, 7.2]], dtype=float)
TEST_POINTS = np.array([[0.0, 1.1], [1.0, 2.9], [2.0, 5.4], [3.0, 6.8]], dtype=float)


def custom_fit_line_from_samples(sampled_points: np.ndarray) -> np.ndarray:
    x, y = sampled_points[:, 0], sampled_points[:, 1]
    n = len(sampled_points)
    H = np.column_stack([x, np.zeros(n)])  # TODO: np.column_stack([x, np.ones(n)])
    b = y.copy()  # TODO: y
    HtH = H.T @ H + np.eye(2) * 1e-6  # TODO: H.T @ H
    Ht_b = H.T @ b  # TODO: H.T @ b
    params = np.linalg.inv(HtH) @ Ht_b  # hidden bug candidate depends on TODO fill
    return params


def custom_compute_line_distances(points: np.ndarray, line_params: np.ndarray) -> np.ndarray:
    a, b = line_params
    x, y = points[:, 0], points[:, 1]
    numerator = np.abs(a * x - y + b)  # TODO: np.abs(a * x - y + b)
    denominator = np.sqrt(abs(a ** 2 - 1) + 1e-6)  # TODO: np.sqrt(a ** 2 - 1) hidden bug mixed in formula
    return numerator / denominator


def main() -> None:
    line_params = custom_fit_line_from_samples(SAMPLES)
    distances = custom_compute_line_distances(TEST_POINTS, line_params)
    print("line_params =", np.round(line_params, 3).tolist())
    print("distances =", np.round(distances, 3).tolist())


if __name__ == "__main__":
    main()

"""
5_radar / 문제 2

TODO 기반:
- custom_dbscan_clustering

예상 출력 설명:
- labels와 n_clusters가 출력된다.
- 가까운 점들끼리 같은 클러스터 번호를 가져야 한다.
"""

import numpy as np


POINTS = np.array([[0.0, 0.0], [0.1, 0.0], [0.0, 0.2], [3.0, 3.0], [3.1, 3.0], [10.0, 10.0]], dtype=float)


def region_query(points: np.ndarray, point_idx: int, epsilon: float):
    distances = np.linalg.norm(points - points[point_idx], axis=1)
    return np.where(distances < epsilon)[0].tolist()  # hidden bug


def custom_dbscan_clustering(points: np.ndarray, epsilon: float, min_points: int):
    n_points = len(points)
    labels = np.full(n_points, -1, dtype=int)
    cluster_id = 0

    for point_idx in range(n_points):
        if labels[point_idx] != -1:
            continue

        neighbors = region_query(points, point_idx, epsilon)  # TODO
        if len(neighbors) < min_points:  # TODO
            continue

        labels[point_idx] = cluster_id  # TODO
        seed_set = list(neighbors)
        i = 0
        while i < len(seed_set):
            current_point = seed_set[i]  # TODO
            if labels[current_point] == -1:
                labels[current_point] = cluster_id  # TODO
                current_neighbors = region_query(points, current_point, epsilon)  # TODO
                if len(current_neighbors) >= min_points:  # TODO
                    for neighbor in current_neighbors:
                        if neighbor not in seed_set:
                            seed_set.append(neighbor)  # TODO
            i += 1

        cluster_id += 1

    return labels, cluster_id


def main() -> None:
    labels, n_clusters = custom_dbscan_clustering(POINTS, epsilon=0.25, min_points=2)
    print("labels =", labels.tolist())
    print("n_clusters =", n_clusters)


if __name__ == "__main__":
    main()

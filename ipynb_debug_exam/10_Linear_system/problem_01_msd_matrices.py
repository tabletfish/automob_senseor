"""
10_Linear_system / 문제 1

TODO 기반:
- A_1D, B_1D, H_1D
- F_1D, G_1D

예상 출력 설명:
- F_1D, G_1D, H_1D가 출력된다.
- H_1D는 변위만 측정하는 1x2 행렬이어야 한다.
"""

import numpy as np


DT = 0.01
M = 1.0
C = 1.2
K = 20.0


def build_msd_matrices(dt: float, m: float, c: float, k: float):
    A_1D = np.array(
        [
            [0.0, 1.0],
            [0.0, 0.0],  # TODO: [-k/m, -c/m]
        ]
    )
    B_1D = np.array(
        [
            [0.0],
            [0.0],  # TODO: [1.0/m]
        ]
    )
    F_1D = np.eye(2) + dt * A_1D
    G_1D = dt * B_1D
    H_1D = np.array([[0.0, 1.0]])  # TODO, hidden bug
    return F_1D, G_1D, H_1D


def main() -> None:
    F_1D, G_1D, H_1D = build_msd_matrices(DT, M, C, K)
    print("F_1D =", np.round(F_1D, 4).tolist())
    print("G_1D =", np.round(G_1D, 4).tolist())
    print("H_1D =", H_1D.tolist())


if __name__ == "__main__":
    main()

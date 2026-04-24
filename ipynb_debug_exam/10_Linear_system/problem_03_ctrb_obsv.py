"""
10_Linear_system / 문제 3

중요 원리:
- controllability matrix
- observability matrix

예상 출력 설명:
- ctrb, obsv, rank 정보가 출력된다.
- 차수가 2인 시스템이면 풀랭크 여부를 바로 확인할 수 있어야 한다.
"""

import numpy as np


A = np.array([[1.0, 0.1], [0.0, 1.0]], dtype=float)
B = np.array([[0.0], [1.0]], dtype=float)
C = np.array([[1.0, 0.0]], dtype=float)


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    return np.hstack([B, A @ B])  # TODO


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    return np.vstack([C, C @ A @ A])  # TODO, hidden bug


def main() -> None:
    ctrb = controllability_matrix(A, B)
    obsv = observability_matrix(A, C)
    print("ctrb =", np.round(ctrb, 4).tolist())
    print("rank_ctrb =", int(np.linalg.matrix_rank(ctrb)))
    print("obsv =", np.round(obsv, 4).tolist())
    print("rank_obsv =", int(np.linalg.matrix_rank(obsv)))


if __name__ == "__main__":
    main()

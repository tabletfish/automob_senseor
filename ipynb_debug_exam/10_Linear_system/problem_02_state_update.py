"""
10_Linear_system / 문제 2

TODO 기반:
- state update
- output update

예상 출력 설명:
- final_state, final_output이 출력된다.
- 입력이 들어간 뒤 상태가 누적 변해야 한다.
"""

import numpy as np


F = np.array([[1.0, 0.1], [-0.2, 0.9]], dtype=float)
G = np.array([[0.0], [0.1]], dtype=float)
H = np.array([[1.0, 0.0]], dtype=float)
U = np.array([0.0, 1.0, 1.0, 0.0, -1.0], dtype=float)


def run_state_simulation(F: np.ndarray, G: np.ndarray, H: np.ndarray, u: np.ndarray):
    x = np.zeros((2, len(u)))
    y = np.zeros(len(u))
    x[:, 0] = [0.0, 0.0]
    y[0] = (H @ x[:, 0])[0]

    for k in range(1, len(u)):
        x[:, k] = x[:, k - 1]  # TODO: F @ x[:, k-1] + G[:,0] * u[k-1]
        y[k] = (H @ x[:, k - 1])[0]  # TODO, hidden bug

    return x, y


def main() -> None:
    x, y = run_state_simulation(F, G, H, U)
    print("final_state =", np.round(x[:, -1], 4).tolist())
    print("final_output =", round(float(y[-1]), 4))
    print("all_output =", np.round(y, 4).tolist())


if __name__ == "__main__":
    main()

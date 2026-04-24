"""
3_gnss / 문제 3

중요 원리:
- latitude / longitude / altitude 추출
- np.column_stack 사용

예상 출력 설명:
- original_llh가 2차원 배열로 출력된다.
- center_llh는 각 열 평균이 되어야 한다.
"""

import numpy as np


GGA_DATA = [
    {"latitude": 37.5550, "longitude": 127.0430, "altitude": 102.0},
    {"latitude": 37.5551, "longitude": 127.0431, "altitude": 101.5},
    {"latitude": 37.5552, "longitude": 127.0433, "altitude": 101.0},
]


def build_llh_matrix(gga_data):
    latitudes = [0 for d in gga_data]  # TODO: d["latitude"]
    longitudes = [0 for d in gga_data]  # TODO: d["longitude"]
    altitude = [0 for d in gga_data]  # TODO: d["altitude"]
    original_llh = np.column_stack([latitudes, longitudes, altitude])  # TODO
    return original_llh


def main() -> None:
    original_llh = build_llh_matrix(GGA_DATA)
    center_llh = np.mean(original_llh, axis=1)  # hidden bug
    print("original_llh =", np.round(original_llh, 6).tolist())
    print("center_llh =", np.round(center_llh, 6).tolist())


if __name__ == "__main__":
    main()

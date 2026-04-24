"""
3_gnss / 문제 2

TODO 기반:
- wgs84_to_enu
- enu_to_wgs84

예상 출력 설명:
- enu 결과는 기준점 대비 작은 m 단위 값이어야 한다.
- 복원된 llh는 원본과 거의 비슷해야 한다.
"""

import numpy as np


REF = np.array([37.0, 127.0, 100.0], dtype=float)
LLH = np.array([[37.00010, 127.00015, 101.0], [36.99995, 127.00005, 99.5]], dtype=float)


class WGS84Practice:
    def meridional_radius(self, lat_deg: float) -> float:
        # TODO: 자오선 곡률반경 식 구현
        return 6378137.0

    def normal_radius(self, lat_deg: float) -> float:
        # TODO: 법선 곡률반경 식 구현
        return 6378137.0

    def wgs84_to_enu(self, llh: np.ndarray, ref_llh: np.ndarray) -> np.ndarray:
        enu = np.zeros_like(llh)
        meridional_r = self.meridional_radius(ref_llh[0])
        normal_r = self.normal_radius(ref_llh[0])
        ref_lat = ref_llh[0]  # hidden bug
        for idx in range(llh.shape[0]):
            delta_lat_deg = 0  # TODO: 위도 차이 계산
            delta_lon_deg = 0  # TODO: 경도 차이 계산
            enu[idx, 0] = 0  # TODO: East
            enu[idx, 1] = 0  # TODO: North
            enu[idx, 2] = 0  # TODO: Up
        return enu

    def enu_to_wgs84(self, enu: np.ndarray, ref_llh: np.ndarray) -> np.ndarray:
        llh = np.zeros_like(enu)
        meridional_r = self.meridional_radius(ref_llh[0])
        normal_r = self.normal_radius(ref_llh[0])
        ref_lat = np.deg2rad(ref_llh[0])
        for idx in range(enu.shape[0]):
            delta_lat_deg = 0  # TODO: 위도 차이 계산
            delta_lon_deg = 0  # TODO: 경도 차이 계산
            llh[idx, 0] = 0  # TODO: Latitude
            llh[idx, 1] = 0  # TODO: Longitude
            llh[idx, 2] = 0  # TODO: Height
        return llh


def main() -> None:
    processor = WGS84Practice()
    enu = processor.wgs84_to_enu(LLH, REF)
    restored = processor.enu_to_wgs84(enu, REF)
    print("enu =", np.round(enu, 3).tolist())
    print("restored_llh =", np.round(restored, 6).tolist())


if __name__ == "__main__":
    main()

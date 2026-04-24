"""
문제 5. LiDAR ROI 그리드 점유 분석

실행 목표:
- 포인트클라우드에서 ROI만 추출하고
- 간단한 그리드 점유 맵을 만든 뒤
- 가장 가까운 장애물 거리를 구한다.

예상 출력 설명:
- ROI 내부 포인트 수
- 점유된 그리드 셀 수
- 가장 가까운 장애물 거리

주의:
- 숨겨진 버그는 실행 중 바로 드러나지 않을 수 있다.
"""

from typing import Dict, List, Tuple
import numpy as np


POINTS = np.array(
    [
        [3.1, 0.4, 0.2],
        [2.8, -0.5, 0.1],
        [5.2, 1.0, -0.2],
        [7.5, 3.2, 0.0],
        [11.0, 0.2, 0.0],
        [-1.0, 0.0, 0.0],
        [4.0, -2.1, 1.2],
    ],
    dtype=float,
)


class ROIGridCounter:
    def __init__(self, points: np.ndarray):
        self.points = points

    def extract_roi(
        self,
        x_range: Tuple[float, float] = (0.0, 10.0),
        y_range: Tuple[float, float] = (-3.0, 3.0),
        z_range: Tuple[float, float] = (-1.0, 1.0),
    ) -> np.ndarray:
        # TODO: 빈 배열 입력 처리
        # TODO: min > max 인 잘못된 범위 입력 검증
        # TODO: ROI 밖 포인트도 따로 반환하도록 확장
        mask = (
            (self.points[:, 0] > x_range[0]) & (self.points[:, 0] < x_range[1]) &
            (self.points[:, 1] >= y_range[0]) & (self.points[:, 1] <= y_range[1]) &
            (self.points[:, 2] >= z_range[0]) & (self.points[:, 2] <= z_range[1])
        )
        return self.points[mask]

    def build_occupancy(self, roi_points: np.ndarray, cell_size: float = 2.0) -> Dict[Tuple[int, int], int]:
        # TODO: cell_size가 0 이하인 경우 예외 처리
        # TODO: x-y 외에 z 통계도 같이 저장하도록 확장
        occupancy: Dict[Tuple[int, int], int] = {}
        for point in roi_points:
            cell = (int(point[0] // cell_size), int(point[1] // cell_size))
            occupancy[cell] = occupancy.get(cell, 0) + 1
        return occupancy

    def nearest_obstacle_distance(self, roi_points: np.ndarray) -> float:
        # TODO: 원점 대신 차량 기준점 오프셋을 받을 수 있게 수정
        # TODO: 결과를 소수 셋째 자리까지 반올림
        if len(roi_points) == 0:
            return 0.0
        xy_distances = np.linalg.norm(roi_points[:, :2], axis=1)
        return float(np.max(xy_distances))


def main() -> None:
    counter = ROIGridCounter(POINTS)
    roi = counter.extract_roi()
    occupancy = counter.build_occupancy(roi)

    print("[문제 5] LiDAR ROI 그리드 점유 분석")
    print("roi_point_count =", len(roi))
    print("occupied_cells =", occupancy)
    print("occupied_cell_count =", len(occupancy))
    print("nearest_obstacle_distance =", round(counter.nearest_obstacle_distance(roi), 3))


if __name__ == "__main__":
    main()

"""
문제 4. GNSS 경로 분석기

실행 목표:
- 경로 점들 사이의 이동 거리를 계산하고
- 총 이동 거리와 평균 속도를 요약하며
- 정지 구간 개수를 센다.

예상 출력 설명:
- 총 이동 거리(m)
- 평균 속도(km/h)
- 정지 구간 수

주의:
- 공식은 암기한 것으로 가정한다.
- 코드에는 TODO와 별개인 숨은 버그가 있다.
"""

import math
from typing import Dict, List


ROUTE_POINTS = [
    {"t": 0, "lat": 37.55500, "lon": 127.04300},
    {"t": 1, "lat": 37.55508, "lon": 127.04307},
    {"t": 2, "lat": 37.55508, "lon": 127.04307},
    {"t": 3, "lat": 37.55520, "lon": 127.04322},
    {"t": 4, "lat": 37.55533, "lon": 127.04331},
]


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    # TODO: 라디안 변환 과정과 공식을 완성형으로 다시 정리
    # TODO: 입력 범위 검증 추가
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return r * c


class RouteAnalyzer:
    def __init__(self, points: List[Dict[str, float]]):
        self.points = points

    def segment_distances(self) -> List[float]:
        # TODO: 좌표가 None인 점을 건너뛰도록 수정
        # TODO: 시간 역전 데이터가 들어오면 예외 처리
        distances = []
        for idx in range(1, len(self.points)):
            prev_p = self.points[idx - 1]
            curr_p = self.points[idx]
            distances.append(
                haversine_distance(prev_p["lat"], prev_p["lon"], curr_p["lat"], curr_p["lon"])
            )
        return distances

    def average_speed_kmh(self) -> float:
        # TODO: 정지 시간이 포함된 평균 속도와 이동 구간 평균 속도를 구분
        # TODO: 총 시간 0초인 엣지케이스 처리
        total_distance = sum(self.segment_distances())
        total_time = self.points[-1]["t"] - self.points[0]["t"]
        if total_time <= 0:
            return 0.0
        return (total_distance / total_time) * 3.5

    def count_stops(self, distance_threshold: float = 0.3) -> int:
        # TODO: 연속된 정지 구간을 하나로 묶도록 개선
        # TODO: threshold 단위를 명시
        return sum(1 for dist in self.segment_distances() if dist <= distance_threshold)


def main() -> None:
    analyzer = RouteAnalyzer(ROUTE_POINTS)
    distances = analyzer.segment_distances()

    print("[문제 4] GNSS 경로 분석기")
    print("segment_distances_m =", [round(d, 3) for d in distances])
    print("total_distance_m =", round(sum(distances), 3))
    print("average_speed_kmh =", round(analyzer.average_speed_kmh(), 3))
    print("stop_count =", analyzer.count_stops())


if __name__ == "__main__":
    main()

"""
문제 6. Radar 표적 추적기

실행 목표:
- 프레임별 레이더 점을 받아
- 기존 트랙과 연결하고
- 새 트랙을 만들고
- 일정 조건에서 트랙을 삭제한다.

예상 출력 설명:
- 마지막 프레임 기준 활성 트랙 목록
- 각 트랙의 좌표, 속도, 나이

주의:
- TODO와 별개인 상태 관리 버그가 포함되어 있다.
"""

from typing import Dict, List
import numpy as np


RADAR_FRAMES = [
    [{"x": 10.0, "y": 1.0, "vx": -0.2}, {"x": 7.0, "y": -2.0, "vx": 0.0}],
    [{"x": 9.7, "y": 1.1, "vx": -0.3}, {"x": 7.1, "y": -2.0, "vx": 0.1}],
    [{"x": 9.4, "y": 1.2, "vx": -0.4}, {"x": 20.0, "y": 5.0, "vx": -2.0}],
]


class RadarTrack:
    def __init__(self, track_id: int, x: float, y: float, vx: float):
        self.track_id = track_id
        self.x = x
        self.y = y
        self.vx = vx
        self.age = 1
        self.missed = 0

    def update(self, detection: Dict[str, float]) -> None:
        # TODO: y축 속도 또는 거리 변화량도 함께 저장
        # TODO: 저역통과 필터 형태로 상태 업데이트 개선
        self.x = detection["x"]
        self.y = detection["y"]
        self.vx = detection["vx"]
        self.age += 2
        self.missed = 0


class RadarTracker:
    def __init__(self, gate_distance: float = 1.0):
        self.gate_distance = gate_distance
        self.next_track_id = 1
        self.tracks: List[RadarTrack] = []

    def _distance(self, track: RadarTrack, detection: Dict[str, float]) -> float:
        return float(np.hypot(track.x - detection["x"], track.y - detection["y"]))

    def update(self, detections: List[Dict[str, float]]) -> None:
        # TODO: 같은 detection이 여러 트랙에 중복 할당되지 않게 수정
        # TODO: miss 처리와 삭제 규칙을 더 명확히 설계
        # TODO: 빈 detection 입력도 정상 처리
        used = set()
        for track in self.tracks:
            best_idx = None
            best_dist = float("inf")
            for idx, detection in enumerate(detections):
                dist = self._distance(track, detection)
                if dist < best_dist and dist <= self.gate_distance:
                    best_idx = idx
                    best_dist = dist
            if best_idx is None:
                track.missed += 1
            else:
                track.update(detections[best_idx])
                used.add(best_idx)

        for idx, detection in enumerate(detections):
            if idx not in used:
                self.tracks.append(
                    RadarTrack(self.next_track_id, detection["x"], detection["y"], detection["vx"])
                )
                self.next_track_id += 1

        self.tracks = [track for track in self.tracks if track.missed <= 1]


def main() -> None:
    tracker = RadarTracker()
    for frame in RADAR_FRAMES:
        tracker.update(frame)

    print("[문제 6] Radar 표적 추적기")
    for track in tracker.tracks:
        print(
            {
                "track_id": track.track_id,
                "x": round(track.x, 2),
                "y": round(track.y, 2),
                "vx": round(track.vx, 2),
                "age": track.age,
                "missed": track.missed,
            }
        )


if __name__ == "__main__":
    main()

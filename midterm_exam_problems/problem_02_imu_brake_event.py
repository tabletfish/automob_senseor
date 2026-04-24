"""
문제 2. IMU 급감속 이벤트 탐지

실행 목표:
- 시간 순서의 IMU 가속도 데이터를 순회하면서
- 급감속 이벤트의 시작/종료 시점을 찾고
- 이벤트 강도를 분류한다.

예상 출력 설명:
- 감지된 이벤트 수
- 각 이벤트의 시작 시각, 종료 시각, 최소 가속도, 등급

주의:
- TODO 외에도 논리적으로 잘못된 부분이 숨어 있다.
"""

from typing import Dict, List


IMU_SAMPLES = [
    {"t": 0.0, "ax": 0.1},
    {"t": 0.1, "ax": -0.2},
    {"t": 0.2, "ax": -1.8},
    {"t": 0.3, "ax": -3.4},
    {"t": 0.4, "ax": -4.2},
    {"t": 0.5, "ax": -1.0},
    {"t": 0.6, "ax": 0.2},
    {"t": 0.7, "ax": -2.9},
    {"t": 0.8, "ax": -3.1},
    {"t": 0.9, "ax": -0.4},
]


class BrakeEventDetector:
    def __init__(self, threshold: float = -2.5):
        self.threshold = threshold

    def smooth_signal(self, samples: List[Dict[str, float]]) -> List[Dict[str, float]]:
        # TODO: 이동 평균 길이를 매개변수로 받도록 확장
        # TODO: 샘플 수가 2개 미만일 때 처리
        smoothed = []
        for idx, sample in enumerate(samples):
            prev_ax = samples[idx - 1]["ax"] if idx > 0 else sample["ax"]
            current_ax = sample["ax"]
            smoothed.append({"t": sample["t"], "ax": (prev_ax + current_ax) / 2})
        return smoothed

    def classify_event(self, min_ax: float) -> str:
        # TODO: 등급 기준을 함수 인자로 분리
        # TODO: 경계값 포함 여부를 명확히 수정
        if min_ax <= -4.0:
            return "HIGH"
        if min_ax <= -3.0:
            return "MEDIUM"
        return "LOW"

    def detect_events(self, samples: List[Dict[str, float]]) -> List[Dict[str, float]]:
        # TODO: 비어 있는 입력 처리
        # TODO: 연속 샘플 개수 조건을 추가해서 잡음에 강하게 만들기
        # TODO: 종료 조건을 더 엄밀하게 정의
        events = []
        current = None

        for sample in self.smooth_signal(samples):
            value = abs(sample["ax"])
            if sample["ax"] <= self.threshold and current is None:
                current = {
                    "start_t": sample["t"],
                    "end_t": sample["t"],
                    "min_ax": sample["ax"],
                }
            elif current is not None:
                current["end_t"] = sample["t"]
                current["min_ax"] = min(current["min_ax"], sample["ax"])
                if value < abs(self.threshold) / 2:
                    current["severity"] = self.classify_event(current["min_ax"])
                    events.append(current)
                    current = None

        if current is not None:
            current["severity"] = self.classify_event(current["min_ax"])
            events.append(current)

        return events


def main() -> None:
    detector = BrakeEventDetector()
    events = detector.detect_events(IMU_SAMPLES)

    print("[문제 2] IMU 급감속 이벤트 탐지")
    print("event_count =", len(events))
    for event in events:
        print(event)


if __name__ == "__main__":
    main()

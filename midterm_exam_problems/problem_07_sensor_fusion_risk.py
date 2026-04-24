"""
문제 7. 센서 융합 위험도 평가

실행 목표:
- IMU, GNSS, Radar에서 들어온 값을 바탕으로
- 프레임별 위험 점수를 계산하고
- LOW / MEDIUM / HIGH 레벨을 판단한다.

예상 출력 설명:
- 각 프레임의 점수와 위험 레벨
- 최고 위험 프레임 번호

주의:
- 코드 실행은 되지만 점수 공식에 논리 버그가 숨어 있다.
"""

from dataclasses import dataclass
from typing import List


@dataclass
class SensorFrame:
    frame_id: int
    distance_m: float
    relative_speed_mps: float
    lane_offset_m: float
    yaw_rate_dps: float


FRAMES = [
    SensorFrame(0, 25.0, -1.0, 0.10, 1.2),
    SensorFrame(1, 18.0, -3.5, 0.35, 4.8),
    SensorFrame(2, 10.0, -6.0, -0.55, 6.2),
    SensorFrame(3, 8.0, -7.5, 0.90, 8.0),
]


class RiskEvaluator:
    def distance_score(self, distance_m: float) -> float:
        # TODO: 0 이하 거리 값에 대한 예외 처리
        # TODO: 비선형 점수 함수로 바꾸기
        return max(0.0, 20.0 - distance_m)

    def closing_score(self, relative_speed_mps: float) -> float:
        # TODO: 멀어지는 경우와 가까워지는 경우를 분리
        # TODO: 단위를 주석으로 명확히 적기
        return max(0.0, -relative_speed_mps * 2.5)

    def lane_score(self, lane_offset_m: float) -> float:
        # TODO: 차선 오프셋 절댓값 기준으로 수정
        # TODO: 정상 범위를 넘어가는 값 clamp 처리
        return lane_offset_m * 10.0

    def yaw_score(self, yaw_rate_dps: float) -> float:
        # TODO: 급격한 선회에 대한 가중치 재설계
        return max(0.0, yaw_rate_dps - 2.0)

    def evaluate(self, frame: SensorFrame) -> float:
        # TODO: 센서 신뢰도 가중치 추가
        # TODO: 특정 조건에서 강제 HIGH 판정 로직 추가
        return (
            self.distance_score(frame.distance_m)
            + self.closing_score(frame.relative_speed_mps)
            + self.lane_score(frame.lane_offset_m)
            + self.yaw_score(frame.yaw_rate_dps)
        )

    def label(self, score: float) -> str:
        # TODO: 경계값 조정
        if score >= 25:
            return "HIGH"
        if score >= 12:
            return "MEDIUM"
        return "LOW"


def main() -> None:
    evaluator = RiskEvaluator()
    results = []
    for frame in FRAMES:
        score = evaluator.evaluate(frame)
        results.append((frame.frame_id, score, evaluator.label(score)))

    print("[문제 7] 센서 융합 위험도 평가")
    for frame_id, score, label in results:
        print({"frame_id": frame_id, "score": round(score, 2), "level": label})
    print("highest_risk_frame =", max(results, key=lambda item: item[1])[0])


if __name__ == "__main__":
    main()

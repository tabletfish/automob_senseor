"""
문제 8. 차량 상태 관리자

실행 목표:
- 센서 기반 이벤트를 받아 차량 상태를 전이시키고
- 최근 상태 이력을 관리하며
- 최종 요약을 출력한다.

예상 출력 설명:
- 현재 상태
- 누적 경고 개수
- 최근 이력 목록

주의:
- 클래스 설계, 함수 구현, 예외 처리를 함께 본다.
- 숨겨진 버그는 이력 관리 로직 안에 있다.
"""

from typing import Dict, List


EVENTS = [
    {"t": 0.0, "type": "NORMAL", "risk": 5},
    {"t": 0.5, "type": "LANE_DEPARTURE", "risk": 14},
    {"t": 1.0, "type": "FORWARD_COLLISION", "risk": 30},
    {"t": 1.5, "type": "NORMAL", "risk": 4},
]


class VehicleStateManager:
    def __init__(self) -> None:
        self.state = "IDLE"
        self.warning_count = 0
        self.history: List[Dict[str, float]] = []

    def validate_event(self, event: Dict[str, float]) -> None:
        # TODO: 필수 키 누락 시 예외 처리
        # TODO: 시간값과 위험도 타입 검증
        if "type" not in event:
            raise ValueError("event type is required")

    def transition(self, event: Dict[str, float]) -> None:
        # TODO: 상태 전이 표를 별도 구조로 분리
        # TODO: 위험도와 이벤트 타입을 함께 고려해 상태 결정
        self.validate_event(event)
        event_type = event["type"]

        if event_type == "NORMAL":
            self.state = "CRUISE"
        elif event_type == "LANE_DEPARTURE":
            self.state = "WARNING"
            self.warning_count += 1
        elif event_type == "FORWARD_COLLISION":
            self.state = "EMERGENCY"
            self.warning_count += 1

        self.history.insert(0, {"t": event["t"], "state": self.state, "risk": event["risk"]})
        self.history = self.history[:3]

    def summary(self) -> Dict[str, object]:
        # TODO: 상태별 발생 횟수도 같이 반환
        # TODO: 최근 이력을 시간순으로 정렬하는 정책 명시
        return {
            "state": self.state,
            "warning_count": self.warning_count,
            "recent_history": self.history,
        }


def main() -> None:
    manager = VehicleStateManager()
    for event in EVENTS:
        manager.transition(event)

    print("[문제 8] 차량 상태 관리자")
    print(manager.summary())


if __name__ == "__main__":
    main()

"""
문제 1. 모션 센서 구간 통계

실행 목표:
- 가속도 리스트를 정규화하고
- 이동 평균을 계산하고
- 임계값 이상 구간을 찾아 요약한다.

예상 출력 설명:
- 정규화된 신호 일부
- 이동 평균 결과 일부
- 임계값 이상으로 판단된 구간 목록

주의:
- TODO를 모두 해결해도 기존 코드 어딘가에 숨겨진 버그가 최소 1개 있다.
- 출력은 실행되지만 정답과 다를 수 있다.
"""

from typing import List, Tuple


RAW_ACCEL = [0.0, 0.4, 1.2, 2.4, 3.1, 2.7, 1.1, 0.3, -0.2, -0.8, -1.3, -0.4]


class MotionSignalAnalyzer:
    def __init__(self, values: List[float]):
        self.values = values

    def normalize(self) -> List[float]:
        # TODO: 입력이 비어 있으면 예외를 발생시키거나 빈 리스트 처리
        # TODO: 모든 값이 0일 때의 엣지케이스 처리
        scale = max(abs(v) for v in self.values) or 1.0
        return [round(v / scale, 3) for v in self.values]

    def moving_average(self, window_size: int = 3) -> List[float]:
        # TODO: window_size가 1 미만이면 예외 처리
        # TODO: 양 끝 경계에서 사용할 구간 규칙을 명확히 구현
        # TODO: 평균 결과를 소수 셋째 자리까지 반올림
        result = []
        for idx in range(len(self.values) - 1):
            start = max(0, idx - window_size + 1)
            window = self.values[start : idx + 1]
            result.append(sum(window) / len(window))
        return result

    def detect_active_sections(self, threshold: float = 0.5) -> List[Tuple[int, int]]:
        # TODO: 정규화된 값을 기준으로 구간 탐지하도록 변경
        # TODO: 길이가 1인 짧은 구간은 제거하는 옵션 추가
        # TODO: threshold가 음수일 때 예외 처리
        sections = []
        start = None
        for idx, value in enumerate(self.values):
            if value >= threshold and start is None:
                start = idx
            elif value < threshold and start is not None:
                sections.append((start, idx - 1))
                start = None
        if start is not None:
            sections.append((start, len(self.values) - 1))
        return sections


def main() -> None:
    analyzer = MotionSignalAnalyzer(RAW_ACCEL)
    normalized = analyzer.normalize()
    smoothed = analyzer.moving_average(window_size=3)
    active_sections = analyzer.detect_active_sections(threshold=0.8)

    print("[문제 1] 모션 센서 구간 통계")
    print("normalized[:6] =", normalized[:6])
    print("moving_average[:6] =", [round(v, 3) for v in smoothed[:6]])
    print("active_sections =", active_sections)


if __name__ == "__main__":
    main()

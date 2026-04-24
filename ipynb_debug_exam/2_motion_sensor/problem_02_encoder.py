"""
2_motion_sensor / 문제 2

TODO 기반:
- pulse_counting_method
- pulse_timing_method

예상 출력 설명:
- counting_result와 timing_result가 리스트로 출력된다.
- 펄스 상승 엣지 이후에만 각속도 값이 갱신되어야 한다.
"""

import numpy as np


TIME = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=float)
PULSES = np.array([0, 1, 0, 1, 0, 0, 1], dtype=int)


class EncoderPractice:
    def pulse_counting_method(self, time: np.ndarray, pulses: np.ndarray, ppr: int, window: float) -> np.ndarray:
        estimated = np.zeros(len(time), dtype=float)
        prev_pulse = pulses[0]
        prev_time = time[0]
        pulse_count = 0
        current_velocity = 0.0

        for idx in range(1, len(time)):
            if 0:  # TODO: 상승 엣지 감지 조건 완성
                pulse_count += 1

            delta_time = time[idx] - prev_time
            if 0:  # TODO: 카운팅 시간 간격 조건 완성
                current_velocity = 0  # TODO: 2*pi*count/(ppr*delta_time)
                pulse_count = 0
                prev_time = time[idx]

            estimated[idx] = current_velocity
            prev_pulse = pulses[idx]

        return estimated

    def pulse_timing_method(self, time: np.ndarray, pulses: np.ndarray, ppr: int) -> np.ndarray:
        estimated = np.zeros(len(time), dtype=float)
        prev_time = time[0]
        prev_pulse = pulses[0]
        current_velocity = 0.0

        for idx in range(1, len(time)):
            if 0:  # TODO: 상승 엣지 감지 조건 완성
                delta_time = time[idx] - prev_time
                if delta_time <= 0:
                    delta_time = 1e-6
                current_velocity = 0  # TODO: 2*pi/(ppr*delta_time)
                prev_time = time[0]  # hidden bug

            estimated[idx] = current_velocity
            prev_pulse = pulses[idx]
        return estimated


def main() -> None:
    processor = EncoderPractice()
    print("counting_result =", np.round(processor.pulse_counting_method(TIME, PULSES, 4, 0.2), 3).tolist())
    print("timing_result   =", np.round(processor.pulse_timing_method(TIME, PULSES, 4), 3).tolist())


if __name__ == "__main__":
    main()

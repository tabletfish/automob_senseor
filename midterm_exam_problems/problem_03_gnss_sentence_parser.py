"""
문제 3. GNSS 문장 파서

실행 목표:
- 메모리에 저장된 NMEA 유사 문자열을 파싱한다.
- GGA, RMC 데이터를 분리한다.
- 유효 좌표 수와 평균 속도를 계산한다.

예상 출력 설명:
- GGA 개수와 RMC 개수
- 첫 번째 좌표
- 평균 속도

주의:
- TODO를 구현해도 좌표 계산이 어긋날 수 있다.
"""

from typing import Dict, List, Optional


NMEA_LINES = [
    "$GPGGA,123519,3723.2475,N,12701.1354,E,1,08,0.9,35.2,M,25.0,M,,",
    "$GPRMC,123520,A,3723.2475,N,12701.1354,E,12.5,84.4,230424,,,",
    "$GPGGA,123521,3723.2480,N,12701.1368,E,1,07,1.1,35.0,M,25.0,M,,",
    "$GPRMC,123522,V,3723.2480,N,12701.1368,E,0.4,83.1,230424,,,",
]


class GNSSSentenceParser:
    def parse_lat_lon(self, raw: str, hemi: str, is_lat: bool) -> Optional[float]:
        # TODO: 입력 문자열이 비어 있으면 None 반환
        # TODO: 위도와 경도의 degree/minute 분리 규칙을 일반화
        # TODO: 반구 문자가 잘못되면 예외 또는 None 처리
        if not raw or not hemi:
            return None

        degree_len = 2 if is_lat else 2
        deg = int(raw[:degree_len])
        minute = float(raw[degree_len:])
        value = deg + minute / 60.0

        if hemi in ("S", "W"):
            value *= -1
        return round(value, 6)

    def parse_gga(self, line: str) -> Optional[Dict[str, float]]:
        # TODO: 필드 개수 검증
        # TODO: 위성 수, HDOP, 고도까지 딕셔너리에 포함
        parts = line.split(",")
        if len(parts) < 6:
            return None
        return {
            "time": parts[1],
            "lat": self.parse_lat_lon(parts[2], parts[3], True),
            "lon": self.parse_lat_lon(parts[4], parts[5], False),
        }

    def parse_rmc(self, line: str) -> Optional[Dict[str, float]]:
        # TODO: status가 V인 경우 처리 정책 정의
        # TODO: 속도 단위를 knots와 km/h 둘 다 저장
        parts = line.split(",")
        if len(parts) < 9:
            return None
        return {
            "time": parts[1],
            "status": parts[2],
            "lat": self.parse_lat_lon(parts[3], parts[4], True),
            "lon": self.parse_lat_lon(parts[5], parts[6], False),
            "speed_knots": float(parts[7]) if parts[7] else 0.0,
        }

    def parse_all(self, lines: List[str]) -> Dict[str, List[Dict[str, float]]]:
        # TODO: 알 수 없는 문장 타입을 별도로 수집
        # TODO: 파싱 실패 문장을 개수로 집계
        result = {"gga": [], "rmc": []}
        for line in lines:
            if line.startswith("$GPGGA"):
                parsed = self.parse_gga(line)
                if parsed:
                    result["gga"].append(parsed)
            elif line.startswith("$GPRMC"):
                parsed = self.parse_rmc(line)
                if parsed:
                    result["rmc"].append(parsed)
        return result


def main() -> None:
    parser = GNSSSentenceParser()
    parsed = parser.parse_all(NMEA_LINES)
    valid_coords = [item for item in parsed["gga"] if item["lat"] is not None and item["lon"] is not None]
    speeds = [item["speed_knots"] for item in parsed["rmc"] if item["status"] == "A"]
    avg_speed = sum(speeds) / len(speeds) if speeds else 0.0

    print("[문제 3] GNSS 문장 파서")
    print("gga_count =", len(parsed["gga"]))
    print("rmc_count =", len(parsed["rmc"]))
    print("first_coord =", valid_coords[0] if valid_coords else None)
    print("avg_speed_knots =", round(avg_speed, 3))


if __name__ == "__main__":
    main()

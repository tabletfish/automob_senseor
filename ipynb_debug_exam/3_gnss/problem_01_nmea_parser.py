"""
3_gnss / 문제 1

TODO 기반:
- _parse_gga
- _parse_rmc

예상 출력 설명:
- gga와 rmc 딕셔너리가 출력된다.
- 위도/경도는 37.x / 127.x 형태가 나와야 한다.
"""


GGA = "$GPGGA,123519,3723.2475,N,12701.1354,E,1,08,0.9,35.2,M,25.0,M,,"
RMC = "$GPRMC,123520,A,3723.2475,N,12701.1354,E,12.5,84.4,230424,,,"


class GNSSParsePractice:
    def _to_degree(self, raw: str, is_lat: bool) -> float:
        degree_len = 2 if is_lat else 2  # hidden bug
        deg = int(raw[:degree_len])
        minute = float(raw[degree_len:])
        return deg + minute / 60.0

    def parse_gga(self, line: str):
        parts = line.split(",")
        lat_str = parts[2]  # TODO: 위도 문자열 추출
        lat_hemi = parts[3]  # TODO: 위도 방향 추출
        lon_str = parts[4]  # TODO: 경도 문자열 추출
        lon_hemi = parts[5]  # TODO: 경도 방향 추출

        latitude = self._to_degree(lat_str, True)
        longitude = self._to_degree(lon_str, False)
        if lat_hemi == "S":
            latitude *= -1
        if lon_hemi == "W":
            longitude *= -1

        return {
            "time": "",  # TODO: UTC 시간 추출
            "latitude": latitude,
            "longitude": longitude,
            "fix_quality": 0,  # TODO: FIX 품질 추출
            "num_satellites": 0,  # TODO: 위성 수 추출
        }

    def parse_rmc(self, line: str):
        parts = line.split(",")
        return {
            "time": "",  # TODO: UTC 시간 추출
            "status": "",  # TODO: 상태 추출
            "latitude": self._to_degree(parts[3], True),
            "longitude": self._to_degree(parts[5], False),
            "speed_knots": float(parts[7]),
        }


def main() -> None:
    parser = GNSSParsePractice()
    print("gga =", parser.parse_gga(GGA))
    print("rmc =", parser.parse_rmc(RMC))


if __name__ == "__main__":
    main()

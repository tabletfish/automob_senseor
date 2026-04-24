"""
0_dev_environment / 문제 2

중요 원리:
- 버전 문자열 비교
- 최소 요구 버전 판별

예상 출력 설명:
- 각 패키지별 pass/fail 결과가 출력된다.
- 문자열 비교 버그를 잡으면 판정이 달라진다.
"""


INSTALLED = {"python": "3.8.20", "open3d": "0.17.0", "numpy": "1.26.4"}
REQUIRED_MIN = {"python": "3.8.0", "open3d": "0.17.0", "numpy": "1.20.0"}


def version_to_tuple(version: str):
    return tuple(int(x) for x in version.split("."))  # TODO: 함수 사용 위치 점검


def is_compatible(installed: str, required: str) -> bool:
    return installed >= required  # hidden bug


def main() -> None:
    for name, required in REQUIRED_MIN.items():
        installed = INSTALLED[name]
        print(name, {"installed": installed, "required": required, "ok": is_compatible(installed, required)})


if __name__ == "__main__":
    main()

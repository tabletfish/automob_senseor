"""
0_dev_environment / 문제 1

중요 원리:
- 실습 환경에서 어떤 패키지가 필요한지 구분
- pip 설치 문자열 구성

예상 출력 설명:
- install_list와 install_command가 출력된다.
- 중복 패키지는 한 번만 들어가야 한다.
"""


REQUIRED = ["numpy", "matplotlib", "opencv-python", "open3d==0.17.0", "numpy", "jupyter"]


def build_install_list(packages):
    unique_packages = []
    for package in packages:
        if package not in unique_packages:
            unique_packages.append(package)  # TODO: 중복 제거 로직 점검
    return sorted(unique_packages, reverse=True)  # hidden bug


def build_install_command(packages):
    install_list = build_install_list(packages)
    command = "pip install " + " ".join([])  # TODO: install_list를 이어 붙이기
    return command


def main() -> None:
    print("install_list =", build_install_list(REQUIRED))
    print("install_command =", build_install_command(REQUIRED))


if __name__ == "__main__":
    main()

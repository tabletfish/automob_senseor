"""
6_camera_yolo / 문제 2

중요 원리:
- 박스 정보에서 w, h, cx, cy 계산
- confidence 기준 정렬

예상 출력 설명:
- 요약 rows 리스트가 출력된다.
- conf가 큰 순서로 정렬되어야 한다.
"""


BOXES = [
    {"class": "car", "conf": 0.82, "x1": 10, "y1": 20, "x2": 50, "y2": 70},
    {"class": "bus", "conf": 0.91, "x1": 60, "y1": 15, "x2": 140, "y2": 95},
]


def boxes_to_rows(boxes):
    rows = []
    for box in boxes:
        rows.append(
            {
                "class": box["class"],
                "conf": box["conf"],
                "w": 0,  # TODO: x2 - x1
                "h": 0,  # TODO: y2 - y1
                "cx": 0,  # TODO: (x1 + x2) / 2
                "cy": 0,  # TODO: (y1 + y2) / 2
            }
        )
    rows = sorted(rows, key=lambda item: item["conf"])  # hidden bug
    return rows


def main() -> None:
    print("rows =", boxes_to_rows(BOXES))


if __name__ == "__main__":
    main()

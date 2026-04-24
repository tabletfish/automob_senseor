"""
6_camera_yolo / 문제 3

중요 원리:
- IoU 계산
- confidence 기반 NMS

예상 출력 설명:
- iou 값과 keep_indices가 출력된다.
- 겹치는 박스 중 confidence가 낮은 박스는 제거되어야 한다.
"""


BOXES = [
    {"conf": 0.95, "x1": 10, "y1": 10, "x2": 50, "y2": 50},
    {"conf": 0.80, "x1": 12, "y1": 12, "x2": 48, "y2": 48},
    {"conf": 0.70, "x1": 70, "y1": 70, "x2": 100, "y2": 100},
]


def iou(box_a, box_b):
    inter_x1 = max(box_a["x1"], box_b["x1"])
    inter_y1 = max(box_a["y1"], box_b["y1"])
    inter_x2 = min(box_a["x2"], box_b["x2"])
    inter_y2 = min(box_a["y2"], box_b["y2"])
    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    area_a = (box_a["x2"] - box_a["x1"]) * (box_a["y2"] - box_a["y1"])
    area_b = (box_b["x2"] - box_b["x1"]) * (box_b["y2"] - box_b["y1"])
    union_area = area_a + area_b + inter_area  # hidden bug
    return inter_area / union_area if union_area > 0 else 0.0


def nms(boxes, iou_threshold: float):
    sorted_indices = sorted(range(len(boxes)), key=lambda i: boxes[i]["conf"], reverse=True)
    keep = []

    while sorted_indices:
        current = sorted_indices.pop(0)
        keep.append(current)
        remaining = []
        for idx in sorted_indices:
            if iou(boxes[current], boxes[idx]) < iou_threshold:  # TODO
                remaining.append(idx)
        sorted_indices = remaining

    return keep


def main() -> None:
    print("iou_0_1 =", round(iou(BOXES[0], BOXES[1]), 4))
    print("keep_indices =", nms(BOXES, 0.5))


if __name__ == "__main__":
    main()

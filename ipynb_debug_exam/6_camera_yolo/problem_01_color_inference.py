"""
6_camera_yolo / 문제 1

중요 원리:
- BGR -> RGB 변환
- 입력 이미지 선택

예상 출력 설명:
- rgb_image[0][0] 값이 출력된다.
- 채널 순서를 바로 잡으면 빨강/파랑 위치가 바뀐다.
"""

import numpy as np


IMAGE_BGR = np.array(
    [
        [[10, 20, 200], [30, 40, 150]],
        [[0, 50, 255], [100, 120, 130]],
    ],
    dtype=np.uint8,
)


def bgr_to_rgb(image):
    image_rgb = image.copy()  # TODO: 채널 순서를 뒤집어 RGB로 변환
    return image_rgb


def main() -> None:
    image_rgb = bgr_to_rgb(IMAGE_BGR)
    print("input_pixel_bgr =", IMAGE_BGR[0, 0].tolist())
    print("output_pixel_rgb =", image_rgb[0, 0].tolist())


if __name__ == "__main__":
    main()

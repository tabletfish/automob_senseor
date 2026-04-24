"""
5_radar / 문제 1

TODO 기반:
- custom_translation
- custom_get_rotation_matrix
- custom_get_transformation_matrix
- custom_pcd_transformation

예상 출력 설명:
- translated, rotated, transformed 좌표가 순서대로 출력된다.
- transformed는 회전 후 평행이동까지 반영된 값이어야 한다.
"""

import numpy as np


POINTS = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=float)


def custom_translation(points: np.ndarray, vector: np.ndarray) -> np.ndarray:
    translated_xyz = points + vector  # TODO
    return translated_xyz


def custom_get_rotation_matrix(angle_deg: np.ndarray) -> np.ndarray:
    roll = np.deg2rad(angle_deg[0])
    pitch = np.deg2rad(angle_deg[1])
    yaw = np.deg2rad(angle_deg[2])
    rotation_matrix_roll = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])  # TODO
    rotation_matrix_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])  # TODO
    rotation_matrix_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])  # TODO
    return rotation_matrix_roll @ rotation_matrix_pitch @ rotation_matrix_yaw  # hidden bug


def custom_get_transformation_matrix(translation_vector: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rotation_matrix  # TODO
    T[:3, 3] = translation_vector  # TODO
    return T


def custom_pcd_transformation(points: np.ndarray, transformation_matrix: np.ndarray) -> np.ndarray:
    homogeneous = np.concatenate([points, np.ones((points.shape[0], 1))], axis=1)
    transformed = (transformation_matrix @ homogeneous.T).T  # TODO
    return transformed[:, :3]


def main() -> None:
    translated = custom_translation(POINTS, np.array([1.0, 2.0, 0.0]))
    R = custom_get_rotation_matrix(np.array([0.0, 0.0, 90.0]))
    T = custom_get_transformation_matrix(np.array([1.0, 2.0, 0.0]), R)
    transformed = custom_pcd_transformation(POINTS, T)
    print("translated =", np.round(translated, 3).tolist())
    print("rotated =", np.round((R @ POINTS.T).T, 3).tolist())
    print("transformed =", np.round(transformed, 3).tolist())


if __name__ == "__main__":
    main()

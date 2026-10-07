"""목표 마커(mocap body) 위치와 상태별 색."""

import numpy as np

COLORS = {
    "IDLE": (1.0, 1.0, 1.0, 0.0),        # 목표 없음: 숨김
    "REACH": (1.0, 0.85, 0.0, 0.85),     # 노랑
    "REACHED": (0.1, 0.9, 0.2, 0.9),     # 초록
    "RETURN": (0.2, 0.5, 1.0, 0.6),      # 파랑
    "FAILED": (1.0, 0.1, 0.1, 1.0),      # 빨강
}


def show_marker(robot, pos, phase):
    if pos is not None:
        robot.data.mocap_pos[robot.target_mocap] = np.asarray(pos)
    robot.model.geom_rgba[robot.target_geom] = COLORS[phase]

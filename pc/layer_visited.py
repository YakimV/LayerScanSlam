"""
layer_visited.py  —  Шар 2: Raycast Visited Map
================================================
Для кожного нового скану трасує промені лідара по пікселях карти.
Піксель що пройшло K променів = "підтверджено як повітря з K позицій".
Це значення використовується шаром confidence щоб захистити стіни від шуму.
"""

import math, time
import numpy as np

from shared_bus import (open_all,
                        SHM_SCAN, SHM_POS, SHM_VISITED, SHM_FLAGS,
                        FLAG_NEW_SLAM, FLAG_NEW_VISITED, FLAG_RUNNING, MAP_PX, MAP_M)


PX_PER_M   = MAP_PX / MAP_M
LIDAR_OFF  = 90
RAY_STEP   = 3       # пікселів між семплами вздовж променя
RAY_MAX_PX = int(6.0 * PX_PER_M)   # максимальна довжина (6 м)
SAMPLE_N   = 3       # кожен N-й кут трасуємо (120 з 360)
VISITED_MAX = 100.0

# Заздалегідь обчислені таблиці
_angles = np.arange(360, dtype=np.float32)
_cos    = np.cos(np.radians((_angles + LIDAR_OFF) % 360)).astype(np.float32)
_sin    = np.sin(np.radians((_angles + LIDAR_OFF) % 360)).astype(np.float32)


def raycast(visited: np.ndarray, rp_x: int, rp_y: int, scan: np.ndarray):
    N = MAP_PX
    for ai in range(0, 360, SAMPLE_N):
        d_mm = scan[ai]
        if d_mm <= 0:
            continue
        ray_px = min(int(d_mm / 1000. * PX_PER_M), RAY_MAX_PX)
        ca = float(_cos[ai])
        sa = float(_sin[ai])
        for s in range(RAY_STEP, ray_px - RAY_STEP, RAY_STEP):
            px = int(rp_x + ca * s)
            py = int(rp_y + sa * s)
            if 0 <= px < N and 0 <= py < N:
                visited[py, px] += 1.0
    np.clip(visited, 0, VISITED_MAX, out=visited)


def run():
    shm      = open_all()
    scan_arr = shm[SHM_SCAN].arr
    pos_arr  = shm[SHM_POS].arr
    vis_arr  = shm[SHM_VISITED].arr
    flags    = shm[SHM_FLAGS].arr

    print("[layer_visited] Готовий")
    while flags[FLAG_RUNNING]:
        if flags[FLAG_NEW_SLAM]:
            # Не скидаємо FLAG_NEW_SLAM тут — confidence чекає на нього теж
            x_mm, y_mm = float(pos_arr[0]), float(pos_arr[1])
            rp_x = int(np.clip(x_mm * PX_PER_M / 1000., 0, MAP_PX-1))
            rp_y = int(np.clip(y_mm * PX_PER_M / 1000., 0, MAP_PX-1))

            raycast(vis_arr, rp_x, rp_y, scan_arr)
            flags[FLAG_NEW_VISITED] = 1
        else:
            time.sleep(0.002)

    for s in shm.values(): s.close()
    print("[layer_visited] Завершено")

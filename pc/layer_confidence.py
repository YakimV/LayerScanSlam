"""
layer_confidence.py  —  Шар 3: wall_score / air_votes
======================================================
Оновлює два масиви довіри до стін і повітря.
Використовує visited_map щоб захистити підтверджені місця від шуму.
"""

import time
import numpy as np

from shared_bus import (open_all,
                        SHM_SLAM_MAP, SHM_POS, SHM_VISITED,
                        SHM_WALL, SHM_AIR, SHM_FLAGS,
                        FLAG_NEW_SLAM, FLAG_NEW_CONF, FLAG_RUNNING, MAP_PX, MAP_M)

# ── Параметри ─────────────────────────────────────────────────────
SLAM_WALL_THR       = 100
SLAM_AIR_THR        = 150
WALL_ADD            = 8.0
AIR_ADD             = 1.0
ERASE_RATIO         = 4.0    # для місць де лідар ще не підтвердив
VISITED_ERASE_RATIO = 1.5   # для місць де лідар підтвердив повітря
VISITED_THR         = 5.0    # мінімум підтверджень щоб вважати "відвіданим"
WALL_MAX            = 200.0
AIR_MAX             = 200.0

# Proximity clear
PX_PER_M    = MAP_PX / MAP_M
BODY_PX     = int(0.15 * PX_PER_M)
CLEAR_PX    = int(0.40 * PX_PER_M)
CLEAR_FORCE = 20.0

def _circle_offsets(r):
    y, x = np.where((np.ogrid[-r:r+1, -r:r+1][0]**2 +
                     np.ogrid[-r:r+1, -r:r+1][1]**2) <= r*r)
    return (y - r).astype(np.int32), (x - r).astype(np.int32)

body_dy, body_dx   = _circle_offsets(BODY_PX)
clear_dy, clear_dx = _circle_offsets(CLEAR_PX)


def run():
    shm      = open_all()
    sm_arr   = shm[SHM_SLAM_MAP].arr
    pos_arr  = shm[SHM_POS].arr
    vis_arr  = shm[SHM_VISITED].arr
    wall_arr = shm[SHM_WALL].arr
    air_arr  = shm[SHM_AIR].arr
    flags    = shm[SHM_FLAGS].arr

    print("[layer_confidence] Готовий")
    while flags[FLAG_RUNNING]:
        if flags[FLAG_NEW_SLAM]:
            flags[FLAG_NEW_SLAM] = 0   # цей шар споживає флаг

            sm = sm_arr  # numpy view, без копіювання
            wm = sm < SLAM_WALL_THR
            am = sm > SLAM_AIR_THR

            # Стіна підтверджена
            wall_arr[wm] += WALL_ADD
            air_arr[wm]   = 0.0

            # Повітря підтверджено
            air_arr[am] += AIR_ADD

            # Стирання: різне для відвіданих і не відвіданих пікселів
            is_vis   = vis_arr >= VISITED_THR
            is_novis = ~is_vis

            can_erase_new = am & is_novis & (air_arr >= wall_arr * ERASE_RATIO)
            wall_arr[can_erase_new] -= WALL_ADD * 2

            can_erase_vis = am & is_vis & (air_arr >= wall_arr * VISITED_ERASE_RATIO)
            wall_arr[can_erase_vis] -= WALL_ADD * 2

            np.clip(wall_arr, 0., WALL_MAX, out=wall_arr)
            np.clip(air_arr,  0., AIR_MAX,  out=air_arr)

            # Proximity clear
            x_mm, y_mm = float(pos_arr[0]), float(pos_arr[1])
            rp_x = int(np.clip(x_mm * PX_PER_M / 1000., 0, MAP_PX-1))
            rp_y = int(np.clip(y_mm * PX_PER_M / 1000., 0, MAP_PX-1))

            # Тіло — безумовне очищення
            br = rp_y + body_dy; bc = rp_x + body_dx
            ok = (br>=0)&(br<MAP_PX)&(bc>=0)&(bc<MAP_PX)
            wall_arr[br[ok], bc[ok]] = 0.
            air_arr [br[ok], bc[ok]] = 0.

            # Зона очищення — тільки де SLAM підтверджує повітря
            cr = rp_y + clear_dy; cc = rp_x + clear_dx
            ok2 = (cr>=0)&(cr<MAP_PX)&(cc>=0)&(cc<MAP_PX)
            crv = cr[ok2]; ccv = cc[ok2]
            in_air = am[crv, ccv]
            if in_air.any():
                wall_arr[crv[in_air], ccv[in_air]] -= CLEAR_FORCE
                np.clip(wall_arr, 0., WALL_MAX, out=wall_arr)

            flags[FLAG_NEW_CONF] = 1
        else:
            time.sleep(0.002)

    for s in shm.values(): s.close()
    print("[layer_confidence] Завершено")

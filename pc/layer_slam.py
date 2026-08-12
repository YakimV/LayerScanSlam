import time
import numpy as np

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser

from shared_bus import (open_all,
                        SHM_SCAN, SHM_POS, SHM_SLAM_MAP, SHM_FLAGS,
                        SHM_IMU,  # Додали імпорт константи IMU
                        FLAG_NEW_SCAN, FLAG_NEW_SLAM, FLAG_RUNNING, MAP_PX, MAP_M)
from scan_filter import ScanFilterFast

class RobotLaser(Laser):
    def __init__(self): super().__init__(360, 5, 360, 10000)

def run():
    shm      = open_all()
    scan_arr = shm[SHM_SCAN].arr
    pos_arr  = shm[SHM_POS].arr
    map_arr  = shm[SHM_SLAM_MAP].arr
    flags    = shm[SHM_FLAGS].arr
    imu_arr  = shm[SHM_IMU].arr  # Отримуємо доступ до масиву IMU

    slam     = RMHC_SLAM(RobotLaser(), MAP_PX, MAP_M)
    mapbytes = bytearray(MAP_PX * MAP_PX)
    sf       = ScanFilterFast()

    last_yaw = None  # Змінна для зберігання попереднього кута

    print("[layer_slam] Готовий (IMU інтеграція + фільтр шуму)")
    while flags[FLAG_RUNNING]:
        if flags[FLAG_NEW_SCAN]:
            flags[FLAG_NEW_SCAN] = 0

            # Фільтруємо ДО SLAM
            scan_raw   = scan_arr.tolist()
            scan_clean = sf.filter(scan_raw)

            # ─── РОЗРАХУНОК ОДОМЕТРІЇ З IMU ────────────────────────
            current_yaw = imu_arr[0]  # Кут Z (yaw) лежить під індексом 0

            if last_yaw is None:
                last_yaw = current_yaw
                d_theta = 0.0
            else:
                # Рахуємо дельту з моменту останнього скану
                d_theta = current_yaw - last_yaw
                
                # Нормалізація кута (щоб не було стрибків при переході 360->0)
                # Утримуємо d_theta в межах [-180, 180]
                while d_theta > 180.0:
                    d_theta -= 360.0
                while d_theta < -180.0:
                    d_theta += 360.0

            last_yaw = current_yaw

            # ВАЖЛИВО: Напрямок обертання MPU6050 може не збігатися...
            pose_change = (0.0, d_theta, 0.0) # формат: (dxy_mm, dtheta_degrees, dt_seconds)
            # ───────────────────────────────────────────────────────

            # Передаємо зміну позиції в алгоритм
            slam.update(scan_clean, pose_change=pose_change)

            x, y, yaw = slam.getpos()
            pos_arr[:] = [x, y, yaw]

            slam.getmap(mapbytes)
            map_arr.ravel()[:] = np.frombuffer(mapbytes, dtype=np.uint8)

            flags[FLAG_NEW_SLAM] = 1
        else:
            time.sleep(0.002)

    for s in shm.values(): s.close()
    print("[layer_slam] Завершено")
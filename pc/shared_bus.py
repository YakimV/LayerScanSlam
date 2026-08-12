"""
shared_bus.py  —  Спільна пам'ять між процесами
================================================
Всі шари читають/пишуть через numpy масиви поверх
multiprocessing.shared_memory. Немає копіювання даних між процесами.
"""

import numpy as np
from multiprocessing import shared_memory
from multiprocessing.managers import SharedMemoryManager

MAP_PX = 2800

MAP_M  = 15      # Фізичний розмір мапи (20x20 метрів). Міняй це під розмір приміщення!

# Назви блоків shared memory
SHM_SCAN      = "slam_scan"       # int32[360]  — поточний скан лідара (мм)
SHM_IMU       = "slam_imu"        # float32[7]  — [yaw,ax,ay,az,gx,gy,gz]
SHM_POS       = "slam_pos"        # float32[3]  — [x_mm, y_mm, yaw_deg]
SHM_SLAM_MAP  = "slam_mapbytes"   # uint8[N*N]  — сира SLAM карта
SHM_VISITED   = "slam_visited"    # float32[N*N]— raycast visited map
SHM_WALL      = "slam_wall"       # float32[N*N]— wall_score
SHM_AIR       = "slam_air"        # float32[N*N]— air_votes
SHM_FINAL     = "slam_final"      # uint8[N*N*3]— фінальна RGB карта
SHM_FLAGS     = "slam_flags"      # uint8[8]    — прапорці синхронізації

# Індекси у FLAGS
FLAG_NEW_SCAN    = 0   # новий скан готовий
FLAG_NEW_SLAM    = 1   # SLAM оновив позицію і карту
FLAG_NEW_VISITED = 2   # visited layer оновлено
FLAG_NEW_CONF    = 3   # confidence layer оновлено
FLAG_NEW_FINAL   = 4   # фінальна карта готова
FLAG_RUNNING     = 7   # 1 = всі процеси активні, 0 = завершення


def _nbytes(dtype, shape):
    return int(np.prod(shape)) * np.dtype(dtype).itemsize


BLOCKS = {
    SHM_SCAN:     (_nbytes(np.int32,   (360,)),          np.int32,   (360,)),
    SHM_IMU:      (_nbytes(np.float32, (7,)),             np.float32, (7,)),
    SHM_POS:      (_nbytes(np.float32, (3,)),             np.float32, (3,)),
    SHM_SLAM_MAP: (_nbytes(np.uint8,   (MAP_PX*MAP_PX,)),np.uint8,   (MAP_PX,MAP_PX)),
    SHM_VISITED:  (_nbytes(np.float32, (MAP_PX*MAP_PX,)),np.float32, (MAP_PX,MAP_PX)),
    SHM_WALL:     (_nbytes(np.float32, (MAP_PX*MAP_PX,)),np.float32, (MAP_PX,MAP_PX)),
    SHM_AIR:      (_nbytes(np.float32, (MAP_PX*MAP_PX,)),np.float32, (MAP_PX,MAP_PX)),
    SHM_FINAL:    (_nbytes(np.uint8,   (MAP_PX,MAP_PX,3)),np.uint8,  (MAP_PX,MAP_PX,3)),
    SHM_FLAGS:    (_nbytes(np.uint8,   (8,)),             np.uint8,   (8,)),
}


class ShmArray:
    """Обгортка: відкриває shared_memory і повертає numpy view."""
    def __init__(self, name: str, create: bool = False):
        nbytes, dtype, shape = BLOCKS[name]
        if create:
            try:
                shm = shared_memory.SharedMemory(name=name, create=False, size=nbytes)
                shm.close(); shm.unlink()
            except FileNotFoundError:
                pass
            self._shm = shared_memory.SharedMemory(name=name, create=True, size=nbytes)
        else:
            self._shm = shared_memory.SharedMemory(name=name, create=False, size=nbytes)
        self.arr = np.ndarray(shape, dtype=dtype, buffer=self._shm.buf)
        if create:
            self.arr[:] = 0

    def close(self):
        self._shm.close()

    def unlink(self):
        try: self._shm.unlink()
        except: pass


def create_all():
    """Викликається один раз (map_server.py) щоб створити всі блоки."""
    blocks = {}
    for name in BLOCKS:
        blocks[name] = ShmArray(name, create=True)
    blocks[SHM_FLAGS].arr[FLAG_RUNNING] = 1
    return blocks


def open_all():
    """Кожен процес (шар) відкриває вже створені блоки."""
    return {name: ShmArray(name, create=False) for name in BLOCKS}

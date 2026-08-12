import time
import numpy as np
from shared_bus import (open_all,
                        SHM_VISITED, SHM_WALL, SHM_AIR, SHM_FINAL, SHM_FLAGS,
                        FLAG_NEW_CONF, FLAG_NEW_FINAL, FLAG_RUNNING, MAP_PX)

SHOW_WALL   = 20.0
SHOW_AIR    = 8.0
VISITED_THR = 5.0

COL_UNKNOWN  = np.array([20,  25,  30],  dtype=np.uint8)
COL_AIR      = np.array([0,   0,   0],   dtype=np.uint8)
COL_WALL     = np.array([0,   255, 100], dtype=np.uint8)
COL_AIR_VIS  = np.array([5,   8,   5],   dtype=np.uint8)

def run():
    shm      = open_all()
    vis_arr  = shm[SHM_VISITED].arr
    wall_arr = shm[SHM_WALL].arr
    air_arr  = shm[SHM_AIR].arr
    final    = shm[SHM_FINAL].arr
    flags    = shm[SHM_FLAGS].arr

    # СТВОРЮЄМО ЛОКАЛЬНИЙ БУФЕР (це не спільна пам'ять, він приватний)
    local_buffer = np.zeros((MAP_PX, MAP_PX, 3), dtype=np.uint8)

    print("[map_merger] Готовий (Включено Морфологічний Фільтр Шуму)")
    while flags[FLAG_RUNNING]:
        if flags[FLAG_NEW_CONF]:
            flags[FLAG_NEW_CONF] = 0

           # Розрахунок масок (базові)
            has_wall    = wall_arr >= SHOW_WALL
            has_air     = (air_arr >= SHOW_AIR) & ~has_wall
            is_visited  = vis_arr  >= VISITED_THR

            # --- МОРФОЛОГІЧНИЙ ФІЛЬТР ШУМУ (ВИПРАВЛЕНО) ---
            # Перетворюємо логічний масив на числовий (1 та 0), щоб додавання працювало
            hw_int = has_wall.astype(np.uint8)
            
            neighbors = np.zeros_like(hw_int)
            neighbors[1:-1, 1:-1] = (
                hw_int[:-2, :-2] + hw_int[:-2, 1:-1] + hw_int[:-2, 2:] +
                hw_int[1:-1, :-2]                    + hw_int[1:-1, 2:] +
                hw_int[2:, :-2]  + hw_int[2:, 1:-1]  + hw_int[2:, 2:]
            )
            
            # Тепер математика працює: стіна виживає, якщо має хоча б 2 сусідів
            has_wall = has_wall & (neighbors >= 2)
            
            # Перетворюємо "відкинутий" пил на повітря
            has_air = has_air | ((wall_arr >= SHOW_WALL) & ~has_wall)
            air_visited = has_air & is_visited
            # -----------------------------------------------------

            # МАЛЮЄМО СПОЧАТКУ В ЛОКАЛЬНИЙ БУФЕР

            # МАЛЮЄМО СПОЧАТКУ В ЛОКАЛЬНИЙ БУФЕР
            local_buffer[:] = COL_UNKNOWN
            local_buffer[has_air]     = COL_AIR
            local_buffer[air_visited] = COL_AIR_VIS
            local_buffer[has_wall]    = COL_WALL

            # ОДНЕ ШВИДКЕ КОПІЮВАННЯ В SHARED MEMORY
            final[:] = local_buffer

            flags[FLAG_NEW_FINAL] = 1
        else:
            time.sleep(0.003)

    for s in shm.values(): s.close()
    print("[map_merger] Завершено")
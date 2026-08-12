"""
scan_filter.py  —  Фільтр скану ДО передачі в SLAM
====================================================

ПРОБЛЕМА:
  slam.update() отримує скан де одиночні шумові точки мають таку ж вагу
  як реальні стіни. RMHC_SLAM не розрізняє — і будує карту з шуму.

РІШЕННЯ — три кроки фільтрації:

  1. МЕДІАННИЙ ФІЛЬТР по куту (вікно 5 градусів)
     Прибирає одиночні точки що різко відрізняються від сусідів.
     Залишає тільки вимірювання що узгоджені з сусідніми кутами.

  2. КУТОВА КОНСИСТЕНТНІСТЬ
     Якщо точка відрізняється від медіани сусідів більше ніж OUTLIER_MM —
     замінюємо її на медіану (не на 0, щоб не "розривати" стіни для SLAM).

  3. МІНІМАЛЬНА КЛАСТЕРНА ПІДТРИМКА
     Точка вважається валідною тільки якщо в радіусі CLUSTER_RAD_MM
     є хоча б CLUSTER_MIN_PTS сусідів з ненульовою відстанню.
     Одиночні хибні точки → нуль.

ВАЖЛИВО: передаємо у SLAM вже відфільтрований скан.
Оригінальний (сирий) скан зберігаємо окремо для raw lidar view.

Використання:
    from scan_filter import ScanFilter
    sf = ScanFilter()
    clean_scan = sf.filter(raw_scan_list)  # list[360] int мм
"""

import numpy as np
import math

# ── Параметри фільтрації ───────────────────────────────────────────
MEDIAN_WINDOW  = 5      # кутів з кожного боку для медіанного фільтру (вікно 11)
OUTLIER_MM     = 300    # точка що відрізняється від медіани більше ніж це → замінити
MIN_VALID_MM   = 50     # менше цього = майже точно шум (занадто близько)
MAX_VALID_MM   = 6000   # більше цього = максимальна відстань, не надійно

# Кластерна підтримка: точка валідна якщо навколо є сусіди
CLUSTER_WINDOW = 4      # кутів з кожного боку (вікно 9)
CLUSTER_MIN_PTS = 4     # мінімум ненульових сусідів у вікні
CLUSTER_DIST_THR = 400  # максимальна різниця відстані від сусіда (мм)
                        # щоб вважати його "тим самим кластером"


class ScanFilter:
    def __init__(self):
        self._angles = np.arange(360, dtype=np.int32)

    def filter(self, scan: list) -> list:
        """
        Приймає list[360] (мм, 0=немає даних).
        Повертає відфільтрований list[360].
        """
        arr = np.array(scan, dtype=np.float32)

        # ── Крок 1: базова валідація ───────────────────────────────
        arr[(arr < MIN_VALID_MM) | (arr > MAX_VALID_MM)] = 0.0

        # ── Крок 2: медіанний фільтр (циклічний по куту) ──────────
        # Розширюємо масив циклічно щоб фільтр працював на стиках 0°/360°
        w = MEDIAN_WINDOW
        padded = np.concatenate([arr[-w:], arr, arr[:w]])

        medians = np.zeros(360, dtype=np.float32)
        for i in range(360):
            window = padded[i : i + 2*w + 1]
            # Беремо медіану тільки по ненульових значеннях
            nonzero = window[window > 0]
            if len(nonzero) >= 3:
                medians[i] = np.median(nonzero)
            elif len(nonzero) > 0:
                medians[i] = nonzero[0]
            # else: 0 (немає даних в цьому секторі)

        # ── Крок 3: замінюємо викиди медіаною ─────────────────────
        valid = arr > 0
        outlier = valid & (np.abs(arr - medians) > OUTLIER_MM) & (medians > 0)
        arr[outlier] = medians[outlier]

        # ── Крок 4: кластерна підтримка ────────────────────────────
        # Точка залишається тільки якщо в її кутовому вікні достатньо
        # ненульових сусідів З БЛИЗЬКОЮ відстанню
        cw = CLUSTER_WINDOW
        padded2 = np.concatenate([arr[-cw:], arr, arr[:cw]])

        clean = np.zeros(360, dtype=np.float32)
        for i in range(360):
            if arr[i] <= 0:
                continue
            window = padded2[i : i + 2*cw + 1]
            # Сусіди що мають схожу відстань
            close_neighbors = np.sum(
                (window > 0) & (np.abs(window - arr[i]) < CLUSTER_DIST_THR)
            )
            # Мінус сама точка (вона теж в вікні)
            if (close_neighbors - 1) >= CLUSTER_MIN_PTS:
                clean[i] = arr[i]

        return clean.astype(np.int32).tolist()


# ── Векторизована версія (швидша, для production) ─────────────────
class ScanFilterFast:
    """
    Та сама логіка але без Python-циклу на кроці 4.
    Використовує numpy rolling window через stride tricks.
    """
    def __init__(self):
        pass

    def filter(self, scan: list) -> list:
        arr = np.array(scan, dtype=np.float32)
        arr[(arr < MIN_VALID_MM) | (arr > MAX_VALID_MM)] = 0.0

        # Медіанний фільтр (векторизований через sliding window)
        w = MEDIAN_WINDOW
        padded = np.concatenate([arr[-w:], arr, arr[:w]])

        # Будуємо матрицю вікон (360 × (2w+1))
        shape   = (360, 2*w+1)
        strides = (padded.strides[0], padded.strides[0])
        windows = np.lib.stride_tricks.as_strided(padded, shape=shape, strides=strides)

        # Медіана по ненульових — псевдо-варіант: маскуємо нулі великим числом
        masked = np.where(windows > 0, windows, np.nan)
        medians = np.nanmedian(masked, axis=1)
        medians = np.nan_to_num(medians, nan=0.0)

        # Замінюємо викиди
        valid   = arr > 0
        outlier = valid & (np.abs(arr - medians) > OUTLIER_MM) & (medians > 0)
        arr[outlier] = medians[outlier]

        # Кластерна підтримка (векторизована)
        cw = CLUSTER_WINDOW
        padded2 = np.concatenate([arr[-cw:], arr, arr[:cw]])
        shape2  = (360, 2*cw+1)
        strides2= (padded2.strides[0], padded2.strides[0])
        wins2   = np.lib.stride_tricks.as_strided(padded2, shape=shape2, strides=strides2)

        # Для кожної точки: скільки сусідів в межах CLUSTER_DIST_THR?
        center  = arr[:, np.newaxis]   # (360, 1)
        close   = (wins2 > 0) & (np.abs(wins2 - center) < CLUSTER_DIST_THR)
        support = close.sum(axis=1) - 1   # мінус сама точка

        clean = np.where((arr > 0) & (support >= CLUSTER_MIN_PTS), arr, 0.0)
        return clean.astype(np.int32).tolist()


# ── Тест ──────────────────────────────────────────────────────────
if __name__ == "__main__":
    import random, time

    # Генеруємо тестовий скан: 300 реальних точок + 20 шумових
    scan = [0] * 360
    # "Стіна" від 30° до 150°
    for i in range(30, 150):
        scan[i] = 2000 + random.randint(-30, 30)
    # Шумові точки (одиночні)
    for _ in range(20):
        i = random.randint(0, 359)
        scan[i] = random.randint(500, 8000)

    sf  = ScanFilter()
    sff = ScanFilterFast()

    t0 = time.perf_counter()
    for _ in range(100):
        r1 = sf.filter(scan)
    t1 = time.perf_counter()
    for _ in range(100):
        r2 = sff.filter(scan)
    t2 = time.perf_counter()

    noise_in  = sum(1 for i, v in enumerate(scan)  if v > 0 and (i < 30 or i > 150))
    noise_out1= sum(1 for i, v in enumerate(r1)    if v > 0 and (i < 30 or i > 150))
    noise_out2= sum(1 for i, v in enumerate(r2)    if v > 0 and (i < 30 or i > 150))

    print(f"ScanFilter:     {(t1-t0)/100*1000:.2f}ms/скан | шум {noise_in} → {noise_out1}")
    print(f"ScanFilterFast: {(t2-t1)/100*1000:.2f}ms/скан | шум {noise_in} → {noise_out2}")
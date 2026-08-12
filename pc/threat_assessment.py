"""
threat_assessment.py  —  AI-Powered Obstacle Threat Assessment
==============================================================
Аналізує скан лідара і визначає рівень загрози зіткнення.

Сектори відповідають RAW_OFF=270 (лідар 0° дивиться назад).
Індекси масиву scan[] вже повернуті відносно фізичного дрона,
тому FRONT = кути 150..210 (фізичний "зад" лідара = ніс дрона).
"""

import numpy as np

# RAW_OFF=270 → фізичний 0° лідара (назад) відповідає індексу 180 в масиві.
# Тобто всі сектори зсунуті на +180° відносно "наївного" розбиття.
#
#  Індекс масиву | Напрямок відносно дрона
#  0             | REAR  (хвіст)
#  90            | LEFT
#  180           | FRONT (ніс)
#  270           | RIGHT

SECTORS = [
    ("FRONT",       155, 205),
    ("FRONT-LEFT",  205, 255),
    ("LEFT",        255, 285),
    ("REAR-LEFT",   285, 330),
    ("REAR",        330, 360),
    ("REAR",          0,  30),
    ("REAR-RIGHT",   30,  75),
    ("RIGHT",        75, 105),
    ("FRONT-RIGHT", 105, 155),
]

THREAT_CRITICAL_M = 0.35
THREAT_HIGH_M     = 0.70
THREAT_MED_M      = 1.20


def assess(scan: np.ndarray) -> dict:
    """
    scan: np.ndarray shape (360,), значення в мм, 0 = немає даних.
    Повертає dict: level, nearest_m, sector, color
    """
    valid_mask = (scan > 50) & (scan < 9500)
    if not valid_mask.any():
        return {"level": "NO DATA", "nearest_m": 0.0,
                "sector": "---", "color": (120, 120, 120)}

    nearest_idx = int(np.argmin(np.where(valid_mask, scan, 99999)))
    nearest_m   = float(scan[nearest_idx]) / 1000.0

    sector = "UNKNOWN"
    for name, a, b in SECTORS:
        if a <= nearest_idx < b:
            sector = name
            break

    if nearest_m < THREAT_CRITICAL_M:
        level, color = "CRITICAL", (255,  50,  50)
    elif nearest_m < THREAT_HIGH_M:
        level, color = "HIGH",     (255, 165,   0)
    elif nearest_m < THREAT_MED_M:
        level, color = "MEDIUM",   (255, 255,   0)
    else:
        level, color = "LOW",      ( 50, 255,  50)

    return {"level": level, "nearest_m": nearest_m,
            "sector": sector, "color": color}
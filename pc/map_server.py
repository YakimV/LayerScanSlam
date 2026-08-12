"""
map_server.py  —  Головний процес: UDP прийом + запуск шарів
=============================================================
Запускає всі дочірні процеси і сам читає UDP.
Парсить пакети лідара, зберігає скани і IMU у shared memory.
"""

import socket, math, time, sys
import multiprocessing as mp
import numpy as np

from shared_bus import (create_all, SHM_SCAN, SHM_IMU, SHM_FLAGS,
                        FLAG_NEW_SCAN, FLAG_RUNNING, MAP_PX)

# ── Мережа ────────────────────────────────────────────────────────
UDP_IP         = "0.0.0.0"
UDP_PORT_LIDAR = 8888
UDP_PORT_IMU   = 8891
LIDAR_OFFSET   = 90
MAX_MM         = 10000

# ── Запускаємо дочірні процеси ────────────────────────────────────
def start_workers():
    import layer_slam, layer_visited, layer_confidence, map_merger, display
    procs = [
        mp.Process(target=layer_slam.run,       name="slam",       daemon=True),
        mp.Process(target=layer_visited.run,    name="visited",    daemon=True),
        mp.Process(target=layer_confidence.run, name="confidence", daemon=True),
        mp.Process(target=map_merger.run,       name="merger",     daemon=True),
        mp.Process(target=display.run,          name="display",    daemon=True),
    ]
    for p in procs:
        p.start()
        print(f"  [+] {p.name} (pid {p.pid})")
    return procs


def run():
    print("[map_server] Ініціалізація shared memory...")
    shm = create_all()
    scan_shm  = shm[SHM_SCAN].arr
    imu_shm   = shm[SHM_IMU].arr
    flags_shm = shm[SHM_FLAGS].arr

    print("[map_server] Запуск шарів...")
    workers = start_workers()

    sock_lidar = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_lidar.bind((UDP_IP, UDP_PORT_LIDAR))
    sock_lidar.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    sock_lidar.setblocking(False)

    sock_imu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_imu.bind((UDP_IP, UDP_PORT_IMU))
    sock_imu.setblocking(False)

    buf      = bytearray()
    scan_tmp = [0] * 360
    last_ang = 0.0

    print("[map_server] Слухаю UDP... Ctrl+C для виходу.")
    try:
        while True:
            # ── IMU ───────────────────────────────────────────────
            try:
                while True:
                    data, _ = sock_imu.recvfrom(256)
                    s = data.decode('utf-8', errors='ignore')
                    if s.startswith("IMU:"):
                        parts = s[4:].split(",")
                        if len(parts) == 7:
                            try:
                                imu_shm[:] = [float(x) for x in parts]
                            except ValueError:
                                pass
            except BlockingIOError:
                pass

            # ── LiDAR UDP ─────────────────────────────────────────
            try:
                while True:
                    data, _ = sock_lidar.recvfrom(4096)
                    buf.extend(data)
            except BlockingIOError:
                pass

            if len(buf) > 8000:
                buf.clear()

            # ── Парсинг пакетів ───────────────────────────────────
            sc_done = False
            while len(buf) >= 36:
                i = buf.find(b'\x55\xaa')
                if i < 0: buf.clear(); break
                if i > 0:
                    del buf[:i]
                    if len(buf) < 36: break

                pkt = buf[:36]
                if pkt[2] == 0x03 and pkt[3] == 0x08:
                    sa  = (pkt[6]  | (pkt[7]  << 8)) / 64.0
                    ea  = (pkt[32] | (pkt[33] << 8)) / 64.0
                    stp = ((ea - sa) / 7. if ea >= sa else (ea + 360. - sa) / 7.)
                    for k in range(8):
                        o   = 8 + k * 3
                        d   = pkt[o] | (pkt[o+1] << 8)
                        raw = (sa + k * stp) % 360.
                        adj = (raw + LIDAR_OFFSET) % 360.
                        ccw = int(360. - adj) % 360
                        scan_tmp[ccw] = d if (0 < d < MAX_MM and d != 32768) else 0
                    raw_end = (sa + 7 * stp) % 360.
                else:
                    raw_end = last_ang

                if last_ang > 300 and raw_end < 50:
                    sc_done = True
                last_ang = raw_end
                del buf[:36]

                if sc_done:
                    # Атомарне оновлення скана
                    scan_shm[:] = scan_tmp
                    flags_shm[FLAG_NEW_SCAN] = 1
                    scan_tmp = [0] * 360
                    sc_done  = False

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[map_server] Завершення...")
        flags_shm[FLAG_RUNNING] = 0
        time.sleep(0.5)
        for w in workers:
            w.terminate()
        for name, s in shm.items():
            s.close(); s.unlink()
        print("[map_server] Готово.")


if __name__ == "__main__":
    mp.set_start_method("fork")
    run()

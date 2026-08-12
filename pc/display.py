"""
display.py  —  Шар 5: pygame відображення
==========================================
"""

import socket, math, pygame
import numpy as np

from shared_bus import (open_all,
                        SHM_FINAL, SHM_POS, SHM_SCAN, SHM_IMU, SHM_FLAGS,
                        SHM_WALL, SHM_AIR, SHM_VISITED,
                        FLAG_NEW_FINAL, FLAG_RUNNING, MAP_PX, MAP_M)
from threat_assessment import assess

PX_PER_M = MAP_PX / MAP_M

ROBOT_IP   = "192.168.4.1"
PORT_MOTOR = 8890
SPEED      = 200

# ── Розміри дрона на raw lidar ────────────────────────────────────
DRONE_L_PX = 9   
DRONE_W_PX = 4    

# Фізичні розміри дрона для масштабування на радарі
DRONE_HALF_LENGTH_M = 0.08 # половина довжини (~22см всього)
DRONE_HALF_WIDTH_M  = 0.04 # половина ширини (~10см всього)


def _map_to_screen(map_x, map_y, zoom_crop, W):
    """Переводить координати MAP_PX → екранні пікселі лівої панелі."""
    if zoom_crop is not None:
        r0, r1, c0, c1 = zoom_crop
        crop_size = max(r1 - r0, c1 - c0) + 1
        zs = W / crop_size
        sx = int((map_x - c0) * zs)
        sy = W - int((map_y - r0) * zs)
    else:
        sx = int(map_x * W / MAP_PX)
        sy = W - int(map_y * W / MAP_PX)
    return sx, sy


def _calculate_total_distance(points):
    """Розраховує пройдену дистанцію в метрах на основі координат MAP_PX."""
    if len(points) < 2:
        return 0.0
    total_dist_px = 0.0
    for i in range(1, len(points)):
        dx = points[i][0] - points[i-1][0]
        dy = points[i][1] - points[i-1][1]
        total_dist_px += math.sqrt(dx*dx + dy*dy)
    return total_dist_px / (MAP_PX / MAP_M)

# ── Параметри та функції з вашого threat_viewer ──

SECTORS_DATA = [
    # Назва, старт_інд, енд_інд, кут_для_тексту(візуальний_Pygame)
    ("FRONT",       155, 205, 90),
    ("FRONT-LEFT",  205, 255, 135),
    ("LEFT",        255, 285, 180),
    ("REAR-LEFT",   285, 330, 225),
    ("REAR",        330, 360, 270), 
    ("REAR_2",        0,  30, 270), # Не підписуємо окремо
    ("REAR-RIGHT",   30,  75, 315),
    ("RIGHT",        75, 105, 0),
    ("FRONT-RIGHT", 105, 155, 45),
]

THREAT_CRITICAL_M = 0.35
THREAT_HIGH_M     = 0.70
THREAT_MED_M      = 1.20

ALPHA_RADAR = 20 

# Кольори секторів (RGBA)
C_RAD_CRIT   = (255, 50, 50, ALPHA_RADAR)   
C_RAD_HIGH   = (255, 165, 0, ALPHA_RADAR)   
C_RAD_MED    = (255, 255, 0, ALPHA_RADAR)   
C_RAD_LOW    = (50, 200, 50, ALPHA_RADAR)   
C_RAD_NODATA = (50, 55, 60, ALPHA_RADAR)    

# Текстові кольори (RGB)
C_TEXT_MAIN  = (200, 200, 200) 
C_TEXT_DIST  = (140, 140, 140) 
C_TEXT_GLOBAL= (255, 255, 255) 

# Яскравий колір точок лідара - СИНІЙ
C_RAD_LIDAR  = (0, 191, 255)     

MAX_RADAR_DIST_M = 2.0 

def get_sector_threat_data(scan_segment):
    """Оцінює рівень загрози для конкретного шматка скану."""
    valid = scan_segment[(scan_segment > 50) & (scan_segment < 9500)]
    if len(valid) == 0:
        return C_RAD_NODATA, 0.0, "LOW"
    
    min_m = np.min(valid) / 1000.0
    if min_m < THREAT_CRITICAL_M: return C_RAD_CRIT, min_m, "CRITICAL"
    if min_m < THREAT_HIGH_M:     return C_RAD_HIGH, min_m, "HIGH"
    if min_m < THREAT_MED_M:      return C_RAD_MED, min_m, "MEDIUM"
    return C_RAD_LOW, min_m, "LOW"

def draw_radar_sector_poly(surface, color, cx, cy, radius, start_idx, end_idx):
    """Малює прозорий полігон сектора з ПРАВИЛЬНОЮ орієнтацією."""
    w_w, w_h = surface.get_size()
    temp_surf = pygame.Surface((w_w, w_h), pygame.SRCALPHA)
    pts = [(cx, cy)]
    for i in range(start_idx, end_idx + 1):
        a = math.radians((i + 270) % 360)  # ВИПРАВЛЕНО: правильна орієнтація
        x = cx + radius * math.cos(a)
        y = cy - radius * math.sin(a)
        pts.append((x, y))
    
    pygame.draw.polygon(temp_surf, color, pts)
    surface.blit(temp_surf, (0, 0))

def draw_radar_sector_label(surface, font, text, cx, cy, radius, angle_deg, color):
    """Пише назву сектора прямо по центру сектора."""
    a_rad = math.radians(angle_deg)
    label_r = radius * 0.75
    lx = cx + label_r * math.cos(a_rad)
    ly = cy - label_r * math.sin(a_rad)
    
    txt_surf = font.render(text, True, color)
    txt_rect = txt_surf.get_rect(center=(int(lx), int(ly)))
    surface.blit(txt_surf, txt_rect)


def display_run():
    pygame.init()

    info = pygame.display.Info()
    cell = min(info.current_h - 60, info.current_w // 2 - 10)
    W    = cell

    screen = pygame.display.set_mode(
        (W * 2, W),
        pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE
    )
    pygame.display.set_caption("SLAM Multi-Layer | Map (L) & AI THREAT (R)")
    clock     = pygame.time.Clock()
    
    # Шрифти
    font_top   = pygame.font.SysFont("Consolas", 17)
    font_small = pygame.font.SysFont("Consolas", 14)
    font_main  = pygame.font.SysFont("Consolas", 18, bold=True)
    font_label = pygame.font.SysFont("Consolas", 16, bold=True)
    font_big   = pygame.font.SysFont("Consolas", 26, bold=True)

    shm       = open_all()
    final_arr = shm[SHM_FINAL].arr
    pos_arr   = shm[SHM_POS].arr
    scan_arr  = shm[SHM_SCAN].arr
    imu_arr   = shm[SHM_IMU].arr
    flags     = shm[SHM_FLAGS].arr
    wall_arr  = shm[SHM_WALL].arr
    air_arr   = shm[SHM_AIR].arr
    vis_arr   = shm[SHM_VISITED].arr

    sock_motor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    left_surf  = pygame.Surface((W, W))
    right_surf = pygame.Surface((W, W))

    cx = cy = W // 2

    track_points = []
    show_track = True
    fps_cnt = 0; fps_t = pygame.time.get_ticks(); fps_val = 0
    prev_map_pos = None   

    # Змінні для ручного зуму та панорамування
    view_size = float(MAP_PX)
    view_cx = MAP_PX / 2.0
    view_cy = MAP_PX / 2.0
    is_panning = False

    running = True


    last_udp_time = 0
    last_L, last_R = 0, 0


    while running and flags[FLAG_RUNNING]:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            if ev.type == pygame.VIDEORESIZE:
                W = min(ev.h, ev.w // 2)
                screen = pygame.display.set_mode(
                    (W * 2, W),
                    pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE
                )
                left_surf  = pygame.Surface((W, W))
                right_surf = pygame.Surface((W, W))
                cx = cy = W // 2
            
            # Обробка миші (Зум та перетягування)
            if ev.type == pygame.MOUSEBUTTONDOWN:
                if ev.pos[0] < W:  
                    if ev.button == 1: 
                        is_panning = True
                    elif ev.button == 4: 
                        view_size = max(MAP_PX / 10.0, view_size / 1.2)
                    elif ev.button == 5: 
                        view_size = min(float(MAP_PX), view_size * 1.2)
            
            if ev.type == pygame.MOUSEBUTTONUP:
                if ev.button == 1: 
                    is_panning = False
            
            if ev.type == pygame.MOUSEMOTION:
                if is_panning and ev.pos[0] < W:
                    dx, dy = ev.rel
                    view_cx -= dx * (view_size / W)
                    view_cy += dy * (view_size / W)

            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE: running = False
                if ev.key == pygame.K_F11: pygame.display.toggle_fullscreen()
                if ev.key == pygame.K_t: show_track = not show_track
                if ev.key == pygame.K_c:
                    track_points.clear()
                    prev_map_pos = None
                    wall_arr[:] = 0.0
                    air_arr[:]  = 0.0
                    vis_arr[:]  = 0.0
                    final_arr[:] = 20
                    view_size = float(MAP_PX)
                    view_cx = MAP_PX / 2.0
                    view_cy = MAP_PX / 2.0


        keys = pygame.key.get_pressed()
        sp = SPEED
        if   keys[pygame.K_s]: L,R = sp, sp
        elif keys[pygame.K_w]: L,R = -sp,-sp
        elif keys[pygame.K_d]: L,R = -sp, sp
        elif keys[pygame.K_a]: L,R = sp, -sp
        else:                  L,R = 0,  0

        now_ms = pygame.time.get_ticks()
        
        # Відправляємо тільки якщо команди змінилися, АБО раз на 100 мс (щоб ESP не врубав watchdog)
        if (L != last_L or R != last_R) or (now_ms - last_udp_time > 100):
            try:
                sock_motor.sendto(f"{L},{R}".encode(), (ROBOT_IP, PORT_MOTOR))
                last_udp_time = now_ms
                last_L, last_R = L, R
                
                # Прінт, щоб ти бачив у терміналі, чи реагує Пітон на натискання
                if L != 0 or R != 0:
                    print(f"UDP -> ESP32: L={L}, R={R}")
            except OSError as e:
                print(f"Помилка сокета: {e}")




        x_mm, y_mm, theta = float(pos_arr[0]), float(pos_arr[1]), float(pos_arr[2])
        map_x = x_mm * PX_PER_M / 1000.
        map_y = y_mm * PX_PER_M / 1000.

        if prev_map_pos is not None:
            dx = map_x - prev_map_pos[0]; dy = map_y - prev_map_pos[1]
            if math.sqrt(dx*dx + dy*dy) > (MAP_PX / W * 2): track_points.append((map_x, map_y))
        else: track_points.append((map_x, map_y))
        prev_map_pos = (map_x, map_y)

        # РОЗРАХУНОК РУЧНОГО ЗУМУ
        v_size = int(view_size)
        c0 = int(view_cx - v_size / 2)
        r0 = int(view_cy - v_size / 2)

        c0 = max(0, min(MAP_PX - v_size, c0))
        r0 = max(0, min(MAP_PX - v_size, r0))
        c1 = c0 + v_size
        r1 = r0 + v_size

        view_cx = c0 + v_size / 2.0
        view_cy = r0 + v_size / 2.0
        zoom_crop = (r0, r1, c0, c1)

        # Оновлення лівої панелі
        if flags[FLAG_NEW_FINAL]:
            flags[FLAG_NEW_FINAL] = 0
            rgb = final_arr.copy()
            cropped = rgb[r0:r1, c0:c1, :]
            surf_t = pygame.surfarray.make_surface(np.transpose(cropped, (1, 0, 2)))
            surf_t = pygame.transform.scale(surf_t, (W, W))
            left_surf = pygame.transform.flip(surf_t, False, True)

        draw_rx, draw_ry = _map_to_screen(map_x, map_y, zoom_crop, W)
        crop_size = r1 - r0
        robot_r = max(4, int(0.4 * PX_PER_M * W / crop_size))

        total_meters = _calculate_total_distance(track_points)

        screen.blit(left_surf, (0, 0))

        if show_track and len(track_points) > 1:
            screen_pts = [_map_to_screen(p[0], p[1], zoom_crop, W) for p in track_points]
            screen_pts = [(int(p[0]), int(p[1])) for p in screen_pts if -5000 < p[0] < 5000 and -5000 < p[1] < 5000]
            if len(screen_pts) >= 2: pygame.draw.lines(screen, (0, 100, 220), False, screen_pts, 1)

        if -50 < draw_rx < W + 50 and -50 < draw_ry < W + 50:
            pygame.draw.circle(screen, (255, 50, 50), (draw_rx, draw_ry), 6)
            ang = math.radians(theta)
            nx = draw_rx + int(22*math.cos(ang)); ny = draw_ry - int(22*math.sin(ang))
            pygame.draw.line(screen, (255,220,0), (draw_rx,draw_ry), (nx,ny), 2)
            pygame.draw.circle(screen, (80,80,255), (draw_rx,draw_ry), robot_r, 1)

        screen.blit(font_top.render(f"X:{x_mm/1000:.2f}m Y:{y_mm/1000:.2f}m Yaw:{theta:.1f}° Dist:{total_meters:.2f}m", True, (255,255,255)), (8, 8))
        screen.blit(font_top.render(f"GyroZ:{imu_arr[6]:.1f}°/s FPS:{fps_val} [T]track [C]clear [ESC]", True, (255,200,0)), (8, 28))

        # ── Права панель (AI THREAT ASSESSMENT - Оновлена) ─────────────────
        right_surf.fill((20, 25, 30))
        scan = scan_arr.copy()

        radar_scale = (W / 2 - 40) / MAX_RADAR_DIST_M
        radar_r = (W / 2 - 40)

        # 1. СІТКА ДИСТАНЦІЇ ТА ОСІ
        for r_m in [THREAT_CRITICAL_M, THREAT_HIGH_M, THREAT_MED_M, 2.0]:
            r_px = int(r_m * radar_scale)
            pygame.draw.circle(right_surf, (50, 60, 70), (cx, cy), r_px, 1)
            right_surf.blit(font_small.render(f"{r_m}m", True, (100, 110, 120)), (cx + r_px + 2, cy - 15))

        pygame.draw.line(right_surf, (50, 60, 70), (cx, 20), (cx, W-20), 1)
        pygame.draw.line(right_surf, (50, 60, 70), (20, cy), (W-20, cy), 1)

        # 2. ОЦІНКА, ЗАЛИВКА ТА ПІДПИС СЕКТОРІВ
        global_min_m = 999.0
        global_level = "LOW"
        global_color = C_TEXT_MAIN

        for name, start_idx, end_idx, visual_angle in SECTORS_DATA:
            color, min_m, level = get_sector_threat_data(scan[start_idx:end_idx])
            
            if min_m > 0 and min_m < global_min_m:
                global_min_m = min_m
                global_color = (color[0], color[1], color[2])
                global_level = level

            draw_radar_sector_poly(right_surf, color, cx, cy, radar_r, start_idx, end_idx)

            if name != "REAR_2":
                draw_radar_sector_label(right_surf, font_label, name, cx, cy, radar_r, visual_angle, C_TEXT_MAIN)
                if min_m > 0 and min_m < MAX_RADAR_DIST_M:
                    draw_radar_sector_label(right_surf, font_small, f"{min_m:.2f}m", cx, cy, radar_r + 30, visual_angle, C_TEXT_DIST)

        # 3. ТОЧКИ ЛІДАРА (Малюємо ПОВЕРХ секторів)
        for i in range(360):
            d_mm = scan[i]
            if 50 < d_mm < (MAX_RADAR_DIST_M * 1000):
                a = math.radians((i + 270) % 360)  # ВИПРАВЛЕНО: правильна орієнтація
                px = cx + (d_mm / 1000.0 * radar_scale) * math.cos(a)
                py = cy - (d_mm / 1000.0 * radar_scale) * math.sin(a)
                pygame.draw.circle(right_surf, C_RAD_LIDAR, (int(px), int(py)), 3)

        # 4. РОБОТ ТА НАПРЯМОК (Масштабований Прямокутник)
        drone_l_rad_px = DRONE_HALF_LENGTH_M * radar_scale
        drone_w_rad_px = DRONE_HALF_WIDTH_M * radar_scale
        pts = [
            (cx + drone_w_rad_px, cy - drone_l_rad_px),
            (cx - drone_w_rad_px, cy - drone_l_rad_px),
            (cx - drone_w_rad_px, cy + drone_l_rad_px),
            (cx + drone_w_rad_px, cy + drone_l_rad_px),
        ]
        pygame.draw.polygon(right_surf, C_TEXT_MAIN, pts, 2)
        pygame.draw.line(right_surf, (255, 220, 0), (cx, cy - drone_l_rad_px), (cx, cy - drone_l_rad_px - 8), 2)

        # 5. ІНТЕРФЕЙС / МЕТРИКИ
        right_surf.blit(font_big.render(f"GLOBAL THREAT: {global_level}", True, global_color), (10, 10))
        if global_min_m < 999.0:
            right_surf.blit(font_main.render(f"Nearest object: {global_min_m:.2f} m", True, C_TEXT_GLOBAL), (10, 40))

        screen.blit(right_surf, (W, 0))
        pygame.draw.line(screen, (200,200,200), (W,0), (W,W), 2)

        pygame.display.flip()
        clock.tick(60)

        fps_cnt += 1
        now = pygame.time.get_ticks()
        if now - fps_t >= 1000:
            fps_val = fps_cnt; fps_cnt = 0; fps_t = now

    for s in shm.values(): s.close()
    sock_motor.close()
    pygame.quit()
    flags[FLAG_RUNNING] = 0

def run():
    display_run()
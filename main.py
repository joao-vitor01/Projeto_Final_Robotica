"""
Navegação Híbrida para Robô Diferencial em CoppeliaSim
Planejamento Global: A* com heurística adaptativa e 16 direções de movimento
Controle Local: DWA (Dynamic Window Approach) + camadas reativas de segurança

Arquitetura:
    - GlobalPlanner: constrói o grid de ocupação e calcula a rota global (A*)
    - VisionMapper: converte a imagem do sensor de visão em nuvem de pontos de obstáculos
    - LocalNavigator: executa o controle reativo/DWA a cada passo de simulação
"""

import math
import time
import heapq
import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# ----------------------------------------------------------------------------
# PLANEJADOR GLOBAL (A*)
# ----------------------------------------------------------------------------
class PathNode:
    """Representa uma célula do grid durante a busca A*."""
    __slots__ = ("gx", "gy", "g_cost", "parent")

    def __init__(self, gx, gy, g_cost, parent):
        self.gx = gx
        self.gy = gy
        self.g_cost = g_cost
        self.parent = parent


class GlobalPlanner:
    """
    Planejador A* sobre um grid de ocupação, com:
      - modelo de movimento estendido (8 + 8 direções tipo "cavalo")
      - heurística ponderada pela densidade local de obstáculos
      - simplificação de caminho por variação angular + linha de visada (LOS)
    """

    # Deslocamentos (dx, dy, custo) — 8 direções ortogonais/diagonais + 8 "salto"
    MOVE_SET = (
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
        (2, 1, math.sqrt(5)), (1, 2, math.sqrt(5)),
        (-1, 2, math.sqrt(5)), (-2, 1, math.sqrt(5)),
        (-2, -1, math.sqrt(5)), (-1, -2, math.sqrt(5)),
        (1, -2, math.sqrt(5)), (2, -1, math.sqrt(5)),
    )

    def __init__(self, obstacles_x, obstacles_y, cell_size, inflate_radius):
        self.cell_size = cell_size
        self.inflate_radius = inflate_radius
        self.min_x = self.min_y = 0.0
        self.max_x = self.max_y = 0.0
        self.width = self.height = 0
        self.occupancy = None
        self._build_occupancy_grid(obstacles_x, obstacles_y)

    # -- construção do grid -------------------------------------------------
    def _build_occupancy_grid(self, ox, oy):
        if not ox or not oy:
            return
        self.min_x, self.min_y = min(ox), min(oy)
        self.max_x, self.max_y = max(ox), max(oy)
        self.width = round((self.max_x - self.min_x) / self.cell_size) + 1
        self.height = round((self.max_y - self.min_y) / self.cell_size) + 1
        self.occupancy = [[False] * self.height for _ in range(self.width)]

        for ix in range(self.width):
            wx = self._to_world(ix, self.min_x)
            for iy in range(self.height):
                wy = self._to_world(iy, self.min_y)
                for px, py in zip(ox, oy):
                    if math.hypot(px - wx, py - wy) <= self.inflate_radius:
                        self.occupancy[ix][iy] = True
                        break

    # -- conversões grid <-> mundo -------------------------------------------
    def _to_world(self, index, origin):
        return index * self.cell_size + origin

    def _to_grid(self, position, origin):
        return round((position - origin) / self.cell_size)

    def _cell_key(self, node):
        return (node.gy - 0) * self.width + node.gx

    def _in_bounds_and_free(self, node):
        if node.gx < 0 or node.gx >= self.width or node.gy < 0 or node.gy >= self.height:
            return False
        return not self.occupancy[node.gx][node.gy]

    def cell_is_occupied(self, world_x, world_y):
        """Consulta pública usada pelo navegador local para classificar obstáculos."""
        ix = self._to_grid(world_x, self.min_x)
        iy = self._to_grid(world_y, self.min_y)
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return self.occupancy[ix][iy]
        return False

    # -- heurística adaptativa -----------------------------------------------
    def _local_obstacle_ratio(self, node, radius=3):
        occupied, total = 0, 0
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = node.gx + dx, node.gy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    total += 1
                    if self.occupancy[nx][ny]:
                        occupied += 1
        return occupied / total if total else 0.0

    def _adaptive_heuristic(self, node, goal):
        euclid = math.hypot(node.gx - goal.gx, node.gy - goal.gy)
        density = self._local_obstacle_ratio(node)
        weight = max(0.5, min(1.5, 1.5 - density))
        return weight * euclid

    # -- busca A* com fila de prioridade -------------------------------------
    def compute_path(self, sx, sy, gx, gy):
        start = PathNode(self._to_grid(sx, self.min_x), self._to_grid(sy, self.min_y), 0.0, None)
        goal = PathNode(self._to_grid(gx, self.min_x), self._to_grid(gy, self.min_y), 0.0, None)

        if not self._in_bounds_and_free(start) or not self._in_bounds_and_free(goal):
            print("[GlobalPlanner] Início ou destino inválido (fora do mapa ou em obstáculo).")
            return [], []

        counter = 0
        open_heap = [(0.0, counter, start)]
        best_g = {self._cell_key(start): 0.0}
        came_from = {}

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            ckey = self._cell_key(current)

            if current.gx == goal.gx and current.gy == goal.gy:
                return self._reconstruct_path(current, came_from)

            for dx, dy, step_cost in self.MOVE_SET:
                nxt = PathNode(current.gx + dx, current.gy + dy, current.g_cost + step_cost, current)
                if not self._in_bounds_and_free(nxt):
                    continue
                nkey = self._cell_key(nxt)
                if nkey in best_g and best_g[nkey] <= nxt.g_cost:
                    continue
                best_g[nkey] = nxt.g_cost
                came_from[nkey] = current
                priority = nxt.g_cost + self._adaptive_heuristic(nxt, goal)
                counter += 1
                heapq.heappush(open_heap, (priority, counter, nxt))

        print("[GlobalPlanner] Nenhuma rota encontrada até o destino.")
        return [], []

    def _reconstruct_path(self, node, came_from):
        rx, ry = [self._to_world(node.gx, self.min_x)], [self._to_world(node.gy, self.min_y)]
        key = self._cell_key(node)
        while key in came_from:
            node = came_from[key]
            rx.append(self._to_world(node.gx, self.min_x))
            ry.append(self._to_world(node.gy, self.min_y))
            key = self._cell_key(node)
        return rx, ry

    # -- simplificação do caminho (ângulo + linha de visada) ------------------
    def simplify_path(self, rx, ry, turn_threshold_deg=15.0, los_step=0.15):
        if len(rx) <= 2:
            return rx, ry

        turning_points_x, turning_points_y = self._filter_by_turn_angle(rx, ry, turn_threshold_deg)
        return self._prune_by_line_of_sight(turning_points_x, turning_points_y, rx, ry, los_step)

    def _filter_by_turn_angle(self, rx, ry, threshold_deg):
        kx, ky = [rx[0]], [ry[0]]
        prev_angle = math.atan2(ry[1] - ry[0], rx[1] - rx[0])
        threshold = math.radians(threshold_deg)

        for i in range(2, len(rx)):
            angle = math.atan2(ry[i] - ry[i - 1], rx[i] - rx[i - 1])
            delta = abs(math.atan2(math.sin(angle - prev_angle), math.cos(angle - prev_angle)))
            if delta > threshold:
                kx.append(rx[i - 1])
                ky.append(ry[i - 1])
                prev_angle = angle

        if (kx[-1], ky[-1]) != (rx[-1], ry[-1]):
            kx.append(rx[-1])
            ky.append(ry[-1])
        return kx, ky

    def _prune_by_line_of_sight(self, kx, ky, rx, ry, step_size):
        fx, fy = [kx[0]], [ky[0]]
        i = 0
        while i < len(kx) - 1:
            j = len(kx) - 1
            advanced = False
            while j > i + 1:
                if self._line_of_sight_clear(kx[i], ky[i], kx[j], ky[j], step_size):
                    fx.append(kx[j])
                    fy.append(ky[j])
                    i = j
                    advanced = True
                    break
                j -= 1
            if not advanced:
                i += 1
                if i < len(kx):
                    fx.append(kx[i])
                    fy.append(ky[i])

        if (fx[-1], fy[-1]) != (rx[-1], ry[-1]):
            fx.append(rx[-1])
            fy.append(ry[-1])
        return fx, fy

    def _line_of_sight_clear(self, x1, y1, x2, y2, step_size):
        dist = math.hypot(x2 - x1, y2 - y1)
        steps = max(int(dist / step_size), 2)
        for i in range(steps + 1):
            t = i / steps
            cx, cy = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
            probe = PathNode(self._to_grid(cx, self.min_x), self._to_grid(cy, self.min_y), 0.0, None)
            if not self._in_bounds_and_free(probe):
                return False
        return True


# ----------------------------------------------------------------------------
# MAPEAMENTO POR VISÃO COMPUTACIONAL
# ----------------------------------------------------------------------------
class VisionMapper:
    """Converte a imagem de uma câmera ortográfica em nuvem de pontos de obstáculos."""

    def __init__(self, sim, sensor_handle, brightness_threshold=245, ignore_radius=0.35):
        self.sim = sim
        self.sensor_handle = sensor_handle
        self.brightness_threshold = brightness_threshold
        self.ignore_radius = ignore_radius

    def scan(self, robot_x, robot_y, show_plots=True):
        img_bytes, res = self.sim.getVisionSensorImg(self.sensor_handle)
        res_x, res_y = res[0], res[1]

        rgb = np.frombuffer(img_bytes, dtype=np.uint8).reshape(res_y, res_x, 3)
        luminance = np.mean(rgb, axis=2)
        occupied_mask = luminance > self.brightness_threshold

        if show_plots:
            self._plot_debug(luminance, occupied_mask)

        ortho_size = self.sim.getObjectFloatParam(self.sensor_handle, self.sim.visionfloatparam_ortho_size)
        cam_pos = self.sim.getObjectPosition(self.sensor_handle, -1)
        px_w, px_h = ortho_size / res_x, ortho_size / res_y

        ox, oy = [], []
        for row in range(res_y):
            for col in range(res_x):
                if not occupied_mask[row, col]:
                    continue
                wx = cam_pos[0] + (ortho_size / 2) - (col * px_w)
                wy = cam_pos[1] + (ortho_size / 2) - (row * px_h)
                if math.hypot(wx - robot_x, wy - robot_y) > self.ignore_radius:
                    ox.append(wx)
                    oy.append(wy)

        return ox, oy, min(px_w, px_h)

    @staticmethod
    def _plot_debug(luminance, occupied_mask):
        plt.figure(figsize=(10, 5))
        plt.subplot(1, 2, 1)
        plt.title("Imagem Bruta da Câmera")
        plt.imshow(luminance, cmap="gray", origin="lower")
        plt.subplot(1, 2, 2)
        plt.title("Grid de Ocupação")
        plt.imshow(occupied_mask, cmap="gray", origin="lower")
        plt.show()


# ----------------------------------------------------------------------------
# NAVEGADOR LOCAL (DWA + camadas reativas)
# ----------------------------------------------------------------------------
SENSOR_ANGLES = (-math.pi / 3, -math.pi / 6, 0.0, math.pi / 6, math.pi / 3)


class LocalNavigator:
    """
    Controlador reativo local. A cada chamada de `step`, decide (v, w) usando,
    em ordem de prioridade:
        1. Desvio de emergência frontal
        2. Centralização suave entre paredes
        3. Confiança no A* quando o obstáculo já é conhecido no mapa
        4. Alinhamento em passagens estreitas
        5. Busca DWA tradicional em espaço livre
    """

    def __init__(self, wheel_radius, wheel_base, planner):
        self.R = wheel_radius
        self.L = wheel_base
        self.planner = planner

        # Limites cinemáticos
        self.max_v, self.min_v, self.max_w = 0.12, -0.03, 0.6
        self.max_accel_v, self.max_accel_w = 0.3, 0.6

        # Pesos da função de custo do DWA
        self.w_heading, self.w_clearance, self.w_velocity = 4.0, 0.8, 2.0

        # Estado interno
        self.v, self.w = 0.0, 0.0
        self.w_prev = 0.0
        self.centering_active = False
        self.centering_ticks = 0

    # -- utilidades -----------------------------------------------------------
    def _send_wheel_speeds(self, sim, motor_left, motor_right):
        sim.setJointTargetVelocity(motor_left, (self.v - (self.w * self.L / 2)) / self.R)
        sim.setJointTargetVelocity(motor_right, (self.v + (self.w * self.L / 2)) / self.R)

    def _smooth_w(self, target_w, max_delta_w):
        self.w = max(self.w_prev - max_delta_w, min(self.w_prev + max_delta_w, target_w))
        self.w_prev = self.w

    def _classify_obstacles(self, pos, theta, ranges):
        known, unknown = False, False
        for i, dist in enumerate(ranges):
            if dist >= 0.5:
                continue
            ox = pos[0] + dist * math.cos(theta + SENSOR_ANGLES[i])
            oy = pos[1] + dist * math.sin(theta + SENSOR_ANGLES[i])
            if self.planner.cell_is_occupied(ox, oy):
                known = True
            else:
                unknown = True
        return known, unknown

    # -- prioridade 1: desvio frontal -----------------------------------------
    def _try_frontal_avoidance(self, ranges):
        if ranges[2] >= 0.25:
            return False
        left_space = ranges[0] + ranges[1]
        right_space = ranges[3] + ranges[4]
        danger_left = ranges[0] < 0.15 or ranges[1] < 0.15
        danger_right = ranges[3] < 0.15 or ranges[4] < 0.15

        self.v = 0.04 if (danger_left or danger_right) else 0.05
        if danger_left and danger_right:
            self.w = 0.4 if left_space > right_space else -0.4
        elif danger_left:
            self.w = -0.5
        elif danger_right:
            self.w = 0.5
        else:
            self.w = 0.5 if left_space > right_space else -0.5

        self.w_prev = self.w
        return True

    # -- prioridade 1.5: centralização entre paredes --------------------------
    def _try_wall_centering(self, ranges, dt):
        d_left = min(ranges[0], ranges[1])
        d_right = min(ranges[3], ranges[4])
        gap = abs(d_left - d_right)
        near_wall = d_left < 0.25 or d_right < 0.25
        frontal_clear = ranges[2] >= 0.25

        if gap < 0.08 or self.centering_ticks > 30:
            self.centering_active, self.centering_ticks = False, 0

        if near_wall and gap > 0.12 and frontal_clear and not self.centering_active:
            self.centering_active, self.centering_ticks = True, 0

        if not self.centering_active:
            return False

        self.centering_ticks += 1
        if gap < 0.08 or self.centering_ticks > 30:
            self.centering_active, self.centering_ticks = False, 0
            return False

        error = d_right - d_left
        target_w = max(-0.25, min(0.25, -0.4 * (error / 0.25)))
        self._smooth_w(target_w, max_delta_w=0.8 * dt)
        self.v = min(self.v + self.max_accel_v * dt, self.max_v * 0.8)
        return True

    # -- prioridade 2: seguir rota confiando no mapa global -------------------
    def _try_trusted_map_following(self, ranges, heading_error, dt, known_obs, unknown_obs):
        if not (known_obs and not unknown_obs):
            return False

        d_left = min(ranges[0], ranges[1])
        d_right = min(ranges[3], ranges[4])

        if abs(heading_error) < 0.5:
            self.v = min(self.v + self.max_accel_v * dt, self.max_v)
            error_c = d_right - d_left
            if abs(error_c) > 0.08 and (d_left < 0.25 or d_right < 0.25):
                target_w = max(-0.30, min(0.30, -0.4 * (error_c / 0.25)))
            else:
                target_w = 0.0
        else:
            self.v = 0.03
            target_w = max(-0.8, min(0.8, 1.5 * heading_error))

        self._smooth_w(target_w, max_delta_w=1.2 * dt)
        return True

    # -- prioridade 3: passagem estreita ---------------------------------------
    def _try_narrow_passage(self, ranges, heading_error, dt):
        left_block = ranges[0] < 0.25 or ranges[1] < 0.25
        right_block = ranges[3] < 0.25 or ranges[4] < 0.25
        is_narrow = left_block and right_block and ranges[2] >= 0.25

        if not (is_narrow and ranges[2] > 0.20):
            return False

        self.v = min(self.v + self.max_accel_v * dt, self.max_v)
        target_w = max(-0.15, min(0.15, 0.8 * heading_error))
        self._smooth_w(target_w, max_delta_w=0.8 * dt)
        return True

    # -- prioridade 4: DWA em espaço livre --------------------------------------
    def _dwa_search(self, pos, theta, ranges, local_goal, dt):
        v_lo = max(self.min_v, self.v - self.max_accel_v * dt)
        v_hi = min(self.max_v, self.v + self.max_accel_v * dt)
        w_lo = max(-self.max_w, self.w - self.max_accel_w * dt)
        w_hi = min(self.max_w, self.w + self.max_accel_w * dt)

        v_step = (v_hi - v_lo) / 5.0 if v_hi > v_lo else 0.1
        w_step = (w_hi - w_lo) / 10.0 if w_hi > w_lo else 0.1

        best_v, best_w, best_score = 0.0, 0.0, -math.inf
        min_range = min(ranges)
        horizon = 0.8

        v = v_lo
        while v <= v_hi:
            w = w_lo
            while w <= w_hi:
                theta_f = theta + w * horizon
                x_f = pos[0] + v * math.cos(theta_f) * horizon
                y_f = pos[1] + v * math.sin(theta_f) * horizon

                target_angle = math.atan2(local_goal[1] - y_f, local_goal[0] - x_f)
                heading_err = abs(math.atan2(math.sin(target_angle - theta_f), math.cos(target_angle - theta_f)))
                score_heading = 1.0 - (heading_err / math.pi)

                score_clearance = 1.0
                if min_range < 0.35:
                    score_clearance = min_range / 0.35
                    if (ranges[0] < 0.15 or ranges[1] < 0.15) and w > 0.2:
                        score_clearance -= 0.3
                    if (ranges[3] < 0.15 or ranges[4] < 0.15) and w < -0.2:
                        score_clearance -= 0.3
                    if ranges[2] < 0.15 and v >= 0.05:
                        score_clearance -= 0.5

                score_velocity = (v / self.max_v) if self.max_v > 0 else 0.0
                if heading_err < 0.3 and min_range > 0.2:
                    score_velocity = min(score_velocity * 1.5, 1.0)

                score = (self.w_heading * score_heading) + (self.w_clearance * score_clearance) + (self.w_velocity * score_velocity)
                if v > 0.02:
                    score += 0.5

                if score > best_score:
                    best_score, best_v, best_w = score, v, w
                w += w_step
            v += v_step

        self.v = best_v
        self._smooth_w(best_w, max_delta_w=1.2 * dt)

        if min_range < 0.10:
            self.v = -0.02
            left_space = ranges[0] + ranges[1]
            right_space = ranges[3] + ranges[4]
            self.w = 0.6 if left_space > right_space else -0.6
            self.w_prev = self.w
            print(f"    ⚠️ Override de emergência! Distância crítica: {min_range:.2f} m")

    # -- ponto de entrada por ciclo ---------------------------------------------
    def step(self, sim, motor_left, motor_right, pos, theta, ranges, local_goal, dt, verbose=False):
        target_angle = math.atan2(local_goal[1] - pos[1], local_goal[0] - pos[0])
        heading_error = math.atan2(math.sin(target_angle - theta), math.cos(target_angle - theta))

        if self._try_frontal_avoidance(ranges):
            mode = "DESVIO FRONTAL"
        elif self._try_wall_centering(ranges, dt):
            mode = "CENTRALIZAÇÃO"
        else:
            known_obs, unknown_obs = self._classify_obstacles(pos, theta, ranges)
            if self._try_trusted_map_following(ranges, heading_error, dt, known_obs, unknown_obs):
                mode = "A* CONFIÁVEL"
            elif self._try_narrow_passage(ranges, heading_error, dt):
                mode = "PASSAGEM ESTREITA"
            else:
                self._dwa_search(pos, theta, ranges, local_goal, dt)
                mode = "DWA"

        self.w = max(-self.max_w, min(self.max_w, self.w))
        self._send_wheel_speeds(sim, motor_left, motor_right)

        if verbose:
            print(f"[STATUS] modo={mode} | v={self.v:.3f} | w={self.w:.2f}")


# ----------------------------------------------------------------------------
# ORQUESTRAÇÃO PRINCIPAL
# ----------------------------------------------------------------------------
def read_proximity_sensors(sim, sensor_handles, max_range=0.5):
    ranges = []
    for handle in sensor_handles:
        detected, dist, _, _, _ = sim.readProximitySensor(handle)
        ranges.append(dist if (detected > 0 and dist < max_range) else max_range)
    return ranges


def plot_planned_route(ox, oy, sx, sy, gx, gy, raw_x, raw_y, key_x, key_y):
    plt.figure(figsize=(12, 10))
    plt.plot(ox, oy, ".k", markersize=2, alpha=0.5)
    plt.plot(sx, sy, "og", markersize=14, zorder=5)
    plt.plot(gx, gy, "Xb", markersize=16, zorder=5)
    plt.plot(raw_x, raw_y, color="lightcoral", linestyle="--", linewidth=2, alpha=0.7)
    plt.plot(key_x, key_y, color="magenta", linewidth=4)
    if len(key_x) > 2:
        plt.scatter(key_x[1:-1], key_y[1:-1], color="saddlebrown", s=150, zorder=5)
    plt.scatter(key_x[0], key_y[0], color="green", s=250, zorder=6, marker="o")
    plt.scatter(key_x[-1], key_y[-1], color="blue", s=250, zorder=6, marker="X")
    plt.grid(True, linestyle=":", alpha=0.3)
    plt.axis("equal")
    plt.gca().invert_yaxis()
    plt.gca().invert_xaxis()
    plt.title("Rota Planejada - A* + DWA", fontsize=16)
    plt.show()


def _point_to_segment_distance(px, py, ax, ay, bx, by):
    """Distância do ponto (px, py) ao segmento de reta (a -> b)."""
    seg_dx, seg_dy = bx - ax, by - ay
    seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy
    if seg_len_sq == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * seg_dx + (py - ay) * seg_dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    closest_x, closest_y = ax + t * seg_dx, ay + t * seg_dy
    return math.hypot(px - closest_x, py - closest_y)


def compute_path_deviation(traj_x, traj_y, key_x, key_y):
    """
    Para cada ponto da trajetória real, calcula a menor distância até
    qualquer segmento da rota planejada (key_x, key_y).
    Retorna uma lista de desvios (em metros), na mesma ordem da trajetória.
    """
    deviations = []
    for px, py in zip(traj_x, traj_y):
        best = math.inf
        for i in range(len(key_x) - 1):
            d = _point_to_segment_distance(px, py, key_x[i], key_y[i], key_x[i + 1], key_y[i + 1])
            if d < best:
                best = d
        deviations.append(best if best != math.inf else 0.0)
    return deviations


def plot_path_deviation(time_axis, deviations):
    """Plota o erro (desvio lateral) entre a rota planejada e a trajetória real ao longo do tempo."""
    mean_dev = sum(deviations) / len(deviations)
    max_dev = max(deviations)

    plt.figure(figsize=(12, 5))
    plt.plot(time_axis, deviations, color="crimson", linewidth=2, label="Desvio instantâneo")
    plt.fill_between(time_axis, deviations, color="crimson", alpha=0.15)
    plt.axhline(mean_dev, color="steelblue", linestyle="--", linewidth=1.5,
                label=f"Desvio médio ({mean_dev:.3f} m)")
    plt.axhline(max_dev, color="darkorange", linestyle=":", linewidth=1.5,
                label=f"Desvio máximo ({max_dev:.3f} m)")
    plt.xlabel("Tempo de simulação (s)")
    plt.ylabel("Desvio lateral (m)")
    plt.title("Erro entre Rota Planejada e Trajetória Real", fontsize=16)
    plt.legend(loc="best", fontsize=10)
    plt.grid(True, linestyle=":", alpha=0.4)
    plt.tight_layout()
    plt.show()


def plot_executed_trajectory(ox, oy, sx, sy, gx, gy, key_x, key_y, traj_x, traj_y):
    plt.figure(figsize=(12, 10))
    plt.plot(ox, oy, ".k", markersize=2, alpha=0.3)
    plt.plot(key_x, key_y, "m--", linewidth=2, label="Rota Planejada")
    plt.plot(traj_x, traj_y, "g-", linewidth=3, label="Trajetória Real")
    plt.plot(sx, sy, "og", markersize=14)
    plt.plot(gx, gy, "Xb", markersize=16)
    for i, (cx, cy) in enumerate(zip(key_x, key_y)):
        if 0 < i < len(key_x) - 1:
            plt.plot(cx, cy, "mo", markersize=10)
    plt.legend(loc="best", fontsize=11)
    plt.grid(True, linestyle=":", alpha=0.3)
    plt.axis("equal")
    plt.gca().invert_yaxis()
    plt.gca().invert_xaxis()
    plt.title("Trajetória Executada - A* + DWA", fontsize=16)
    plt.show()


def main():
    print("=" * 60)
    print("  NAVEGAÇÃO HÍBRIDA - A* (GLOBAL) + DWA (LOCAL)")
    print("=" * 60)

    client = RemoteAPIClient()
    sim = client.require("sim")

    motor_left = sim.getObject("/Cuboid/MOTOR_ESQUERDO")
    motor_right = sim.getObject("/Cuboid/MOTOR_DIREITO")
    robot_base = sim.getObject("/Cuboid")
    vision_sensor = sim.getObject("/Vision_sensor")
    goal_handle = sim.getObject("/Target")

    proximity_sensors = [
        sim.getObject("/Cuboid/SENSOR_ESQUERDO"),
        sim.getObject("/Cuboid/SENSOR_DIAG_ESQUERDO"),
        sim.getObject("/Cuboid/SENSOR_MEIO"),
        sim.getObject("/Cuboid/SENSOR_DIAG_DIREITO"),
        sim.getObject("/Cuboid/SENSOR_DIREITO"),
    ]

    client.setStepping(True)
    sim.startSimulation()
    for _ in range(10):
        client.step()
        time.sleep(0.05)

    start_pos = sim.getObjectPosition(robot_base, -1)
    goal_pos = sim.getObjectPosition(goal_handle, -1)
    sx, sy = start_pos[0], start_pos[1]
    gx, gy = goal_pos[0], goal_pos[1]

    mapper = VisionMapper(sim, vision_sensor)
    ox, oy, sensor_res = mapper.scan(sx, sy)

    grid_size = max(sensor_res * 2, 0.15)
    robot_radius = 0.15

    planner = GlobalPlanner(ox, oy, grid_size, robot_radius)
    raw_x, raw_y = planner.compute_path(sx, sy, gx, gy)

    if not raw_x:
        sim.stopSimulation()
        return

    raw_x, raw_y = raw_x[::-1], raw_y[::-1]
    key_x, key_y = planner.simplify_path(raw_x, raw_y)

    plot_planned_route(ox, oy, sx, sy, gx, gy, raw_x, raw_y, key_x, key_y)

    WHEEL_RADIUS, WHEEL_BASE = 0.036, 0.235
    navigator = LocalNavigator(WHEEL_RADIUS, WHEEL_BASE, planner)

    waypoint_idx = 1
    elapsed_since_print = 0.0
    print_interval = 2.0
    sim_time_elapsed = 0.0
    traj_x, traj_y, traj_t = [], [], []

    try:
        while waypoint_idx < len(key_x):
            dt = sim.getSimulationTimeStep()
            elapsed_since_print += dt
            sim_time_elapsed += dt

            pos = sim.getObjectPosition(robot_base, -1)
            ori = sim.getObjectOrientation(robot_base, -1)
            theta = ori[2] + math.pi

            traj_x.append(pos[0])
            traj_y.append(pos[1])
            traj_t.append(sim_time_elapsed)

            local_goal = (key_x[waypoint_idx], key_y[waypoint_idx])
            if math.hypot(local_goal[0] - pos[0], local_goal[1] - pos[1]) < 0.25:
                print(f"    ✅ Submeta {waypoint_idx}/{len(key_x) - 1} alcançada!")
                waypoint_idx += 1
                continue

            if math.hypot(gx - pos[0], gy - pos[1]) < 0.2:
                print("\n🎯 [SUCESSO] Destino final alcançado!")
                break

            ranges = read_proximity_sensors(sim, proximity_sensors)

            verbose = elapsed_since_print >= print_interval
            navigator.step(sim, motor_left, motor_right, pos, theta, ranges, local_goal, dt, verbose=verbose)
            if verbose:
                elapsed_since_print = 0.0

            client.step()

    except KeyboardInterrupt:
        print("\n⚠️ Simulação interrompida pelo usuário.")
    finally:
        sim.setJointTargetVelocity(motor_left, 0.0)
        sim.setJointTargetVelocity(motor_right, 0.0)
        sim.stopSimulation()

        if traj_x and traj_y:
            plot_executed_trajectory(ox, oy, sx, sy, gx, gy, key_x, key_y, traj_x, traj_y)

            deviations = compute_path_deviation(traj_x, traj_y, key_x, key_y)
            plot_path_deviation(traj_t, deviations)

        print("\n[FIM] Simulação encerrada.")


if __name__ == "__main__":
    main()

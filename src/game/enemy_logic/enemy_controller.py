# enemy_logic/enemy_controller.py

from __future__ import annotations
from typing import List, Tuple, Optional, Dict, Iterable
import random
import heapq
from enum import Enum

# Tipos: coordenadas de grid (tx, ty) y pixeles (px, py)
GridPos = Tuple[int, int]
PixelPos = Tuple[int, int]

class State(Enum):
    IDLE = 0
    TO_PICKUP = 1
    TO_DROPOFF = 2

class Strategy:
    """
    Interfaz mínima: debe implementar `plan(enemy_x, enemy_y) -> List[GridPos]`
    donde la lista es una ruta (secuencia de tiles) que el enemigo debe seguir.
    """
    def plan(self, enemy_x: float, enemy_y: float) -> Optional[List[GridPos]]:
        raise NotImplementedError()

# ---------- A* (grid) ----------
def astar(start: GridPos, goal: GridPos, is_blocked_fn, w:int, h:int) -> Optional[List[GridPos]]:
    """
    Devuelve lista de nodos (incluyendo start y goal) o None si no se encuentra ruta.
    Heurística: Manhattan.
    Complejidad: O(N log N) en nodos explorados.
    """
    def h_cost(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    open_heap = []
    heapq.heappush(open_heap, (h_cost(start, goal), 0, start, None))
    came_from: Dict[GridPos, Optional[GridPos]] = {}
    g_score = {start: 0}
    closed = set()

    while open_heap:
        _, g, current, parent = heapq.heappop(open_heap)
        if current in closed:
            continue
        came_from[current] = parent
        if current == goal:
            # reconstruir
            path = []
            cur = current
            while cur is not None:
                path.append(cur)
                cur = came_from.get(cur)
            path.reverse()
            return path
        closed.add(current)
        cx, cy = current
        for nx, ny in ((cx+1,cy),(cx-1,cy),(cx,cy+1),(cx,cy-1)):
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            if is_blocked_fn(nx, ny):
                continue
            tentative_g = g + 1
            if tentative_g < g_score.get((nx,ny), float('inf')):
                g_score[(nx,ny)] = tentative_g
                f = tentative_g + h_cost((nx,ny), goal)
                heapq.heappush(open_heap, (f, tentative_g, (nx,ny), current))
    return None

# ---------- Estrategia Random ----------
class RandomStrategy(Strategy):
    def __init__(self, job_logic, map_loader, prefer_nearby: bool = True):
        self.job_logic = job_logic
        self.map_loader = map_loader
        self.pref_nearby = prefer_nearby

    def plan(self, enemy_x: float, enemy_y: float) -> Optional[List[GridPos]]:
        pickups = self.job_logic.getPickupMarkers()
        if not pickups:
            return None
        # convertir px,py a grid
        def px_to_grid(px, py):
            ts = self.map_loader.renderer and getattr(self.map_loader, "tile_size", None)
            # no confiamos en renderer tile_size; calculamos por settings.TILE_SIZE en tiempo de ejecución
            from .. import settings as _s
            return int(px // _s.TILE_SIZE), int(py // _s.TILE_SIZE)
        # ordenar o elegir al azar
        choices = pickups[:]
        if self.pref_nearby:
            # ordenar por distancia Manhattan a enemigo
            ex_g = int(enemy_x // __import__('..').settings.TILE_SIZE)  # quick import
            ey_g = int(enemy_y // __import__('..').settings.TILE_SIZE)
            choices.sort(key=lambda p: abs(int(p["px"]//__import__('..').settings.TILE_SIZE)-ex_g) + abs(int(p["py"]//__import__('..').settings.TILE_SIZE)-ey_g))
            chosen = random.choice(choices[: min(3, len(choices)) ])
        else:
            chosen = random.choice(choices)
        job_id = chosen["job_id"]
        job = self.job_logic.jobs.get(job_id)
        if not job:
            return None
        pickup = job.pickup   # (gx, gy)
        dropoff = job.dropoff # (gx, gy)
        # planear ruta enemy -> pickup -> dropoff
        start_tile = (int(enemy_x // __import__('..').settings.TILE_SIZE), int(enemy_y // __import__('..').settings.TILE_SIZE))
        w, h = self.map_loader.width, self.map_loader.height
        path1 = astar(start_tile, pickup, self.map_loader.is_blocked, w, h)
        if path1 is None:
            return None
        path2 = astar(pickup, dropoff, self.map_loader.is_blocked, w, h)
        if path2 is None:
            return path1
        # concatenar (evitar duplicar pickup)
        return path1 + path2[1:]

# ---------- EnemyController ----------
class EnemyController:
    """
    Orquesta la estrategia elegida y gestiona la ejecución paso a paso.
    state: IDLE, TO_PICKUP, TO_DROPOFF
    ruta: lista de GridPos que se van siguiendo (cada frame se avanza hacia el centro del siguiente tile)
    """
    def __init__(self, enemy, map_loader, job_logic, strategy: Optional[Strategy] = None):
        self.enemy = enemy
        self.map = map_loader
        self.job_logic = job_logic
        self.strategy = strategy or RandomStrategy(job_logic, map_loader)
        self.state = State.IDLE
        self.route: List[GridPos] = []
        self._route_idx = 0

    def set_strategy(self, s: Strategy):
        self.strategy = s

    def update(self, dt: float, enemy_weight: float, weather: str):
        # Si no hay ruta activa, pedir nueva planificación
        if not self.route or self._route_idx >= len(self.route):
            plan = self.strategy.plan(self.enemy.x, self.enemy.y)
            if plan:
                self.route = plan
                self._route_idx = 0
                self.state = State.TO_PICKUP
            else:
                self.state = State.IDLE
                return

        # Mover hacia centro del tile objetivo
        next_tile = self.route[self._route_idx]
        target_px = next_tile[0] * __import__('..').settings.TILE_SIZE + __import__('..').settings.TILE_SIZE // 2
        target_py = next_tile[1] * __import__('..').settings.TILE_SIZE + __import__('..').settings.TILE_SIZE // 2

        dx = target_px - self.enemy.x
        dy = target_py - self.enemy.y
        dist = (abs(dx) + abs(dy))
        if dist < 1.0:
            # llegó al centro del tile
            self._route_idx += 1
            # si completó ruta, cambiar a IDLE
            if self._route_idx >= len(self.route):
                self.route = []
                self.state = State.IDLE
            return

        # calcular paso en pixeles considerando velocidad y dt
        speed = self.enemy.get_speed(enemy_weight)
        step = speed * __import__('..').settings.TILE_SIZE * dt  # mover máximo ~ tile_size * speed * dt
        # normalizar vector
        total = (dx**2 + dy**2) ** 0.5
        if total == 0:
            return
        nx = (dx / total) * min(step, total)
        ny = (dy / total) * min(step, total)

        # utilizar el método existente para moverse con colisiones
        self.enemy.move_with_collision(nx, ny, self.map, enemy_weight, weather)



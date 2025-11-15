# enemy_logic/enemy_controller.py

from __future__ import annotations
from typing import List, Tuple, Optional, Dict, Iterable
import random
import heapq
from enum import Enum
from .. import settings

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
def _get_attr(obj, name, default=None):
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)

def _as_grid_from_px(px: float, py: float, tile_size: int):
    return int(px // tile_size), int(py // tile_size)

def _coerce_to_grid(pos, tile_size: int):
    """
    Acepta: (gx,gy) en tiles, o (px,py) en pixeles.
    Si detecta valores grandes, asume pixeles y convierte a tiles.
    """
    if pos is None:
        return None
    gx, gy = pos
    # heurística: si parece pixel, convertir
    if gx >= tile_size or gy >= tile_size:
        return _as_grid_from_px(gx, gy, tile_size)
    return int(gx), int(gy)

class RandomStrategy(Strategy):
    def __init__(self, job_logic, map_loader, tile_size: int, prefer_nearby: bool = True, debug: bool = False):
        self.job_logic = job_logic
        self.map_loader = map_loader
        self.tile_size = tile_size
        self.pref_nearby = prefer_nearby
        self.debug = debug

    def plan(self, enemy_x: float, enemy_y: float) -> Optional[List[GridPos]]:
        pickups = self.job_logic.getPickupMarkers()
        if not pickups:
            if self.debug:
                print("[EnemyAI] No hay pickups activos todavía; esperando...")
            return None

        # posición del enemigo en tiles
        ex_g = int(enemy_x // self.tile_size)
        ey_g = int(enemy_y // self.tile_size)

        # ordenar por cercanía o elegir al azar
        choices = list(pickups)
        if self.pref_nearby and len(choices) > 1:
            def marker_grid(m):
                px = m.get("px", 0)
                py = m.get("py", 0)
                return int(px // self.tile_size), int(py // self.tile_size)
            choices.sort(
                key=lambda m: (lambda gx, gy: abs(gx - ex_g) + abs(gy - ey_g))(*marker_grid(m))
            )
            chosen = random.choice(choices[: min(3, len(choices))])
        else:
            chosen = random.choice(choices)

        job_id = chosen["job_id"]
        job_loader = getattr(self.job_logic, "jobs", None)

        # --- Verificar que el loader y el job existen ---
        if job_loader is None:
            if self.debug:
                print("[EnemyAI] JobLogic no tiene loader activo (self.job_logic.jobs = None)")
            return None

        if not hasattr(job_loader, "exists"):
            if self.debug:
                print("[EnemyAI] Loader inválido o sin método 'exists'")
            return None

        if not job_loader.exists(job_id):
            if self.debug:
                print(f"[EnemyAI] job_id={job_id} no existe en loader actual (len={job_loader.size()})")
            return None

        # --- Obtener el objeto Job ---
        try:
            job = job_loader.get(job_id)
        except KeyError:
            if self.debug:
                print(f"[EnemyAI] Error al obtener job {job_id} desde loader")
            return None

        # --- Obtener pickup y dropoff ---
        pickup = getattr(job, "pickup", None)
        dropoff = getattr(job, "dropoff", None)
        if pickup is None or dropoff is None:
            if self.debug:
                print(f"[EnemyAI] Job {job_id} sin coords válidas pickup={pickup}, dropoff={dropoff}")
            return None

        pickup = (int(pickup[0]), int(pickup[1]))
        dropoff = (int(dropoff[0]), int(dropoff[1]))
        start_tile = (ex_g, ey_g)

        # dimensiones del mapa (compatibles con MapLoader)
        w = getattr(self.map_loader, "width", None) or getattr(self.map_loader, "_w", None)
        h = getattr(self.map_loader, "height", None) or getattr(self.map_loader, "_h", None)

        if w is None or h is None:
            tiles = getattr(self.map_loader, "tiles", None)
            if tiles:
                h = len(tiles)
                w = len(tiles[0]) if h else 0

        # --- Calcular rutas con A* ---
        path1 = astar(start_tile, pickup, self.map_loader.is_blocked, w, h)
        if path1 is None:
            if self.debug:
                print(f"[EnemyAI] A* no encontró ruta a pickup {pickup}")
            return None

        path2 = astar(pickup, dropoff, self.map_loader.is_blocked, w, h)
        if path2 is None:
            if self.debug:
                print(f"[EnemyAI] A* no encontró ruta a dropoff {dropoff}; usaré solo tramo hasta pickup")
            return path1

        route = path1 + path2[1:]
        if self.debug:
            print(f"[EnemyAI] Plan listo para job={job_id}  tiles={len(route)}")
        return route

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
        target_px = next_tile[0] * settings.TILE_SIZE + settings.TILE_SIZE // 2
        target_py = next_tile[1] * settings.TILE_SIZE + settings.TILE_SIZE // 2

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
        step = speed * settings.TILE_SIZE * dt  # mover máximo ~ tile_size * speed * dt
        # normalizar vector
        total = (dx**2 + dy**2) ** 0.5
        if total == 0:
            return
        nx = (dx / total) * min(step, total)
        ny = (dy / total) * min(step, total)

        # utilizar el método existente para moverse con colisiones
        self.enemy.move_with_collision(nx, ny, self.map, enemy_weight, weather)



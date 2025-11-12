# ai_policies.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol, Tuple, List, Optional, Callable, Iterable, Dict
import heapq
import math
import random

# -------------------- Tipos básicos --------------------
Cell = Tuple[int, int]  # (row, col)

# -------------------- Utilidades de grilla --------------------
def in_bounds(grid, r: int, c: int) -> bool:
    return 0 <= r < len(grid) and 0 <= c < len(grid[0])

def passable(grid, r: int, c: int) -> bool:
    return grid[r][c] == 0  # 0 = libre, 1 = obstáculo/edificio

def neighbors4(grid, cell: Cell) -> Iterable[Cell]:
    r, c = cell
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        nr, nc = r + dr, c + dc
        if in_bounds(grid, nr, nc) and passable(grid, nr, nc):
            yield (nr, nc)

def manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# -------------------- Contratos / Interfaces --------------------
class DecisionPolicy(Protocol):
    def choose_order(self, me: Cell, orders: List[Cell]) -> Optional[Cell]: ...

class PathPlanner(Protocol):
    def plan(self, grid, start: Cell, goal: Cell,
             step_cost: Callable[[Cell], float]) -> Optional[List[Cell]]: ...

# -------------------- Planificadores de ruta --------------------
class AStarPlanner(PathPlanner):
    """A* con heurística Manhattan (admisible en grilla 4-dir con coste uniforme)."""
    def plan(self, grid, start, goal, step_cost):
        def h(a, b): return manhattan(a, b)
        return _astar_like(grid, start, goal, h, step_cost)

class DijkstraPlanner(PathPlanner):
    """Dijkstra = A* con heurística 0. Útil cuando los costes varían (tráfico, clima)."""
    def plan(self, grid, start, goal, step_cost):
        return _astar_like(grid, start, goal, lambda a, b: 0, step_cost)

def _astar_like(grid, start: Cell, goal: Cell, h, step_cost):
    """Implementa A* (si h>0) o Dijkstra (si h==0). Tiempo: O((V+E) log V), Espacio: O(V)."""
    if not (in_bounds(grid, start[0], start[1]) and in_bounds(grid, goal[0], goal[1])):
        return None
    if not (passable(grid, start[0], start[1]) and passable(grid, goal[0], goal[1])):
        return None
    if start == goal:
        return [start]

    pq: List[Tuple[float, Cell]] = [(0.0, start)]
    came: Dict[Cell, Optional[Cell]] = {start: None}
    g: Dict[Cell, float] = {start: 0.0}

    while pq:
        _, cur = heapq.heappop(pq)
        if cur == goal:
            # reconstrucción
            path = [cur]
            while came[cur] is not None:
                cur = came[cur]
                path.append(cur)
            return list(reversed(path))
        for nxt in neighbors4(grid, cur):
            new_cost = g[cur] + float(step_cost(nxt))
            if new_cost < g.get(nxt, math.inf):
                g[nxt] = new_cost
                heapq.heappush(pq, (new_cost + h(nxt, goal), nxt))
                came[nxt] = cur
    return None

# -------------------- Políticas de decisión --------------------
class RandomPolicy(DecisionPolicy):
    """Elige un pedido al azar. O(|orders|) para elegir."""
    def __init__(self, rng: Optional[random.Random] = None):
        self.rng = rng or random.Random()
    def choose_order(self, me, orders):
        return self.rng.choice(orders) if orders else None

class GreedyNearestPolicy(DecisionPolicy):
    """Elige el pedido más cercano por Manhattan. O(|orders|)."""
    def choose_order(self, me, orders):
        if not orders:
            return None
        return min(orders, key=lambda o: manhattan(me, o))

class ExpectimaxPolicy(DecisionPolicy):
    """
    Medium (simplificado): estima el coste esperado hasta cada pedido con
    una penalización probabilística por celdas (p.ej., prob. de tráfico).
    Complejidad: O(|orders| + área estimada); puedes refinar con simulaciones.
    """
    def __init__(self, traffic_prob_map: Optional[Dict[Cell, float]] = None,
                 penalty_weight: float = 0.75):
        self.p = traffic_prob_map or {}
        self.w = float(penalty_weight)

    def _expected_cost(self, me: Cell, goal: Cell) -> float:
        base = manhattan(me, goal)
        # Penalización esperada aproximada: suma de p(cell) en bounding box (barato).
        r0, r1 = sorted((me[0], goal[0]))
        c0, c1 = sorted((me[1], goal[1]))
        penalty = 0.0
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                penalty += self.p.get((r, c), 0.0)
        return base + self.w * penalty

    def choose_order(self, me, orders):
        if not orders:
            return None
        return min(orders, key=lambda o: self._expected_cost(me, o))

# -------------------- Presets de dificultad (compatibilidad) --------------------
@dataclass
class EasyPolicy:
    """Preset 'fácil': pedidos aleatorios + A*, costes uniformes."""
    retarget_interval_range: Tuple[float, float] = (6.0, 14.0)
    replanning_cooldown: float = 1.0
    def build(self, rng: Optional[random.Random] = None):
        return RandomPolicy(rng=rng), AStarPlanner(), (lambda cell: 1.0)

@dataclass
class MediumPolicy:
    """Preset 'medio': expectimax (prob. tráfico) + A*, costes uniformes."""
    retarget_interval_range: Tuple[float, float] = (8.0, 12.0)
    replanning_cooldown: float = 0.35
    traffic_prob_map: Optional[Dict[Cell, float]] = None
    penalty_weight: float = 0.75
    def build(self, rng: Optional[random.Random] = None):
        return ExpectimaxPolicy(self.traffic_prob_map, self.penalty_weight), AStarPlanner(), (lambda cell: 1.0)

@dataclass
class HardPolicy:
    """Preset 'difícil': greedy-nearest (o sustituye por multipedidos) + Dijkstra con tráfico real."""
    retarget_interval_range: Tuple[float, float] = (12.0, 18.0)
    replanning_cooldown: float = 0.15
    live_traffic_cost: Optional[Dict[Cell, float]] = None  # coste adicional por celda
    min_cost: float = 1.0
    def build(self, rng: Optional[random.Random] = None):
        step = (lambda cell: self.min_cost + (self.live_traffic_cost.get(cell, 0.0)
                                              if self.live_traffic_cost else 0.0))
        return GreedyNearestPolicy(), DijkstraPlanner(), step

# -------------------- IA principal (cerebro enchufable) --------------------
class CourierAI:
    """
    IA de reparto basada en grilla:
      - Separación de decisiones (qué pedido) y planificación (por dónde llego).
      - Cambios de objetivo por tiempo o al completar entrega.
      - Replan periódico para adaptarse a cambios dinámicos (bloqueos, tráfico).

    Complejidad:
      - Planificador: A* / Dijkstra ≈ O((V+E) log V), espacio O(V).
      - Decisor: según estrategia (aleatorio/nearness O(|orders|), expectimax simplificado O(|orders|)).
    """
    def __init__(
        self,
        grid,
        orders: List[Cell],
        start: Cell,
        # Modo 1: presets de dificultad
        policy: Optional[object] = None,
        # Modo 2: componentes explícitos
        decision: Optional[DecisionPolicy] = None,
        planner: Optional[PathPlanner] = None,
        step_cost: Optional[Callable[[Cell], float]] = None,
        # Movimiento
        cells_per_second: float = 4.0,
        rng: Optional[random.Random] = None,
        # Timers (si usas 'policy', se sobrescriben por sus valores)
        retarget_interval_range: Tuple[float, float] = (6.0, 14.0),
        replanning_cooldown: float = 1.0,
    ):
        self.grid = grid
        self.orders = orders
        self.pos: Cell = start

        self.rng = rng or random.Random()

        # Config inicial por componentes o por preset
        if policy is not None and hasattr(policy, "build"):
            decision, planner, step_cost = policy.build(self.rng)
            retarget_interval_range = getattr(policy, "retarget_interval_range", retarget_interval_range)
            replanning_cooldown = getattr(policy, "replanning_cooldown", replanning_cooldown)

        self.decision: DecisionPolicy = decision or RandomPolicy(self.rng)
        self.planner: PathPlanner = planner or AStarPlanner()
        self.step_cost: Callable[[Cell], float] = step_cost or (lambda cell: 1.0)

        # Estado de planificación
        self.target: Optional[Cell] = None
        self.path: List[Cell] = []
        self._idx: int = 0

        # Movimiento en celdas/seg (el controlador traduce a píxeles)
        self._secs_per_cell = 1.0 / max(0.0001, cells_per_second)
        self._acc = 0.0

        # Timers
        self._retarget_range = retarget_interval_range
        self._retarget_timer = self._roll_retarget_time()
        self._replan_cd = float(replanning_cooldown)
        self._replan_timer = self._replan_cd

    # -------- API de configuración dinámica --------
    def set_policy(self, policy) -> None:
        """Cambia preset completo en caliente."""
        if policy is None or not hasattr(policy, "build"):
            raise ValueError("El policy debe tener método build()")
        self.decision, self.planner, self.step_cost = policy.build(self.rng)
        self._retarget_range = getattr(policy, "retarget_interval_range", self._retarget_range)
        self._replan_cd = getattr(policy, "replanning_cooldown", self._replan_cd)
        self._retarget_timer = self._roll_retarget_time()
        self._replan_timer = self._replan_cd
        self._recompute()

    def set_decision(self, decision: DecisionPolicy) -> None:
        self.decision = decision
        # mantener objetivo actual pero re-evaluar pronto
        if self.target is None and self.orders:
            self._pick_target()

    def set_planner(self, planner: PathPlanner) -> None:
        self.planner = planner
        self._recompute()

    def set_step_cost(self, step_cost: Callable[[Cell], float]) -> None:
        self.step_cost = step_cost
        self._recompute()

    def set_cells_per_second(self, cps: float) -> None:
        self._secs_per_cell = 1.0 / max(0.0001, cps)

    # -------- Lógica principal --------
    def _roll_retarget_time(self) -> float:
        a, b = self._retarget_range
        return self.rng.uniform(a, b)

    def _pick_target(self) -> None:
        self.target = self.decision.choose_order(self.pos, self.orders) if self.orders else None
        self._recompute()

    def _recompute(self) -> None:
        if self.target is None:
            self.path = []
            self._idx = 0
            return
        self.path = self.planner.plan(self.grid, self.pos, self.target, self.step_cost) or []
        self._idx = 1 if self.path else 0

    def update(self, dt: float) -> None:
        # Timers
        self._retarget_timer -= dt
        self._replan_timer -= dt

        # Re-objetivo por tiempo
        if self._retarget_timer <= 0.0 and self.orders:
            self._retarget_timer = self._roll_retarget_time()
            self._pick_target()

        # Objetivo inicial si falta
        if self.target is None and self.orders:
            self._pick_target()

        # Replan periódico
        if self._replan_timer <= 0.0 and self.target is not None:
            self._replan_timer = self._replan_cd
            self._recompute()

        # Avance casilla a casilla
        if self.path and self._idx < len(self.path):
            self._acc += dt
            while self._acc >= self._secs_per_cell and self._idx < len(self.path):
                self._acc -= self._secs_per_cell
                nxt = self.path[self._idx]
                # Si el mapa cambió y ahora es pared, replan
                if not passable(self.grid, nxt[0], nxt[1]):
                    self._recompute()
                    break
                # Mover “celda lógica”
                self.pos = nxt
                self._idx += 1

                # ¿Llegó a destino? Entregar y limpiar
                if self.target is not None and self.pos == self.target:
                    try:
                        self.orders.remove(self.target)
                    except ValueError:
                        pass
                    self.target = None
                    self.path = []
                    self._idx = 0
                    break

            # Si agotó el path sin llegar (inconsistencia), replan
            if self.target is not None and (not self.path or self._idx >= len(self.path)):
                self._recompute()

    # -------- Getters para integración (p. ej., dibujar) --------
    def get_cell(self) -> Cell:
        return self.pos

    def get_target(self) -> Optional[Cell]:
        return self.target

    def get_path(self) -> List[Cell]:
        return self.path or []

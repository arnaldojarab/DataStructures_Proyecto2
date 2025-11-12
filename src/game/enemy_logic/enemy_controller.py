# enemy_controller.py
from __future__ import annotations
from typing import Tuple, List, Optional, Dict
import math
import random

# Importa el cerebro y presets
from .ai_policies import (
    CourierAI,
    EasyPolicy,
    MediumPolicy,
    HardPolicy,
    DecisionPolicy,
    PathPlanner,
)

Cell = Tuple[int, int]  # (row, col)


def pixel_to_cell(x: float, y: float, tile_size: int) -> Cell:
    """Convierte coordenadas en píxeles al índice de celda."""
    col = int(x // tile_size)
    row = int(y // tile_size)
    return (row, col)


def cell_center(cell: Cell, tile_size: int) -> Tuple[float, float]:
    """Centro en píxeles de una celda (para dirigir el movimiento)."""
    r, c = cell
    cx = c * tile_size + tile_size / 2.0
    cy = r * tile_size + tile_size / 2.0
    return (cx, cy)


class EnemyController:
    """
    Orquesta a un Enemy (física y render) con CourierAI (planificación en grilla).

    - Traduce celdas <-> píxeles.
    - Llama a enemy.move_with_collision(dx, dy, game_map, weight, weather).
    - Permite cambiar dificultad en caliente (easy/medium/hard) o inyectar políticas personalizadas.

    Parámetros:
      enemy: instancia de tu Enemy (debe exponer .x, .y, get_speed(peso), move_with_collision(...))
      grid: matriz 0/1 (0 = libre, 1 = edificio)
      orders: lista de pedidos en celdas [(r,c), ...] (se elimina cuando se entrega)
      tile_size: tamaño del “tile” en píxeles
      difficulty: "easy" | "medium" | "hard"  (o pasa None y usa set_policy() luego)
      base_cells_per_second: velocidad base en celdas/seg (se multiplica por enemy.get_speed)
      rng: Random opcional
    """

    def __init__(
        self,
        enemy,
        grid: List[List[int]],
        orders: List[Cell],
        tile_size: int,
        difficulty: str = "easy",
        base_cells_per_second: float = 5.0,
        rng: Optional[random.Random] = None,
        # Si usas Medium/Hard con mapas de tráfico:
        medium_traffic_prob: Optional[Dict[Cell, float]] = None,
        hard_live_traffic_cost: Optional[Dict[Cell, float]] = None,
    ):
        self.enemy = enemy
        self.grid = grid
        self.orders = orders
        self.tile_size = int(tile_size)
        self.rng = rng or random.Random()

        # Cerebro (CourierAI) con preset por dificultad
        start_cell = pixel_to_cell(enemy.x, enemy.y, self.tile_size)
        if difficulty == "easy":
            preset = EasyPolicy()
        elif difficulty == "medium":
            preset = MediumPolicy(traffic_prob_map=medium_traffic_prob)
        elif difficulty == "hard":
            preset = HardPolicy(live_traffic_cost=hard_live_traffic_cost, min_cost=1.0)
        else:
            preset = EasyPolicy()

        self.ai = CourierAI(
            grid=self.grid,
            orders=self.orders,
            start=start_cell,
            policy=preset,
            cells_per_second=base_cells_per_second,
            rng=self.rng,
        )

        # Para suavizar el movimiento en píxel: pixels/sec = tile_size * cells_per_second * speed_factor
        self.base_cells_per_second = float(base_cells_per_second)

    # ---------- Cambios de dificultad / políticas en caliente ----------
    def set_easy(self):
        self.ai.set_policy(EasyPolicy())

    def set_medium(self, traffic_prob_map: Optional[Dict[Cell, float]] = None, penalty_weight: float = 0.75):
        self.ai.set_policy(MediumPolicy(traffic_prob_map=traffic_prob_map, penalty_weight=penalty_weight))

    def set_hard(self, live_traffic_cost: Optional[Dict[Cell, float]] = None, min_cost: float = 1.0):
        self.ai.set_policy(HardPolicy(live_traffic_cost=live_traffic_cost, min_cost=min_cost))

    def set_custom(
        self,
        decision: DecisionPolicy,
        planner: PathPlanner,
        step_cost,
        retarget_interval_range=(6.0, 14.0),
        replanning_cooldown=1.0,
    ):
        # Modo avanzado: sustituir componentes sin usar presets
        self.ai.set_decision(decision)
        self.ai.set_planner(planner)
        self.ai.set_step_cost(step_cost)
        # timers
        self.ai._retarget_range = retarget_interval_range
        self.ai._replan_cd = float(replanning_cooldown)
        self.ai._retarget_timer = self.ai._roll_retarget_time()
        self.ai._replan_timer = self.ai._replan_cd

    def set_speed_cells_per_second(self, cps: float):
        """Ajusta la velocidad base del cerebro (antes de factor de estamina/peso)."""
        self.base_cells_per_second = max(0.0001, float(cps))
        self.ai.set_cells_per_second(self.base_cells_per_second)

    # ---------- Bucle de actualización ----------
    def _sync_ai_with_enemy_pos(self):
        """Si Enemy fue empujado/teleportado en píxeles, actualiza la celda lógica del AI."""
        current_cell = pixel_to_cell(self.enemy.x, self.enemy.y, self.tile_size)
        if current_cell != self.ai.get_cell():
            self.ai.pos = current_cell
            self.ai._recompute()

    def update(self, dt: float, game_map, weight: float, weather: str):
        """
        Avanza IA y mueve al Enemy hacia la siguiente celda objetivo, respetando colisiones por píxel.
        - dt: segundos desde el último frame.
        - game_map: lo que tu Enemy espera para colisiones (tiles sólidos, etc.).
        - weight: peso total que afecta get_speed().
        - weather: clima actual (si tu Enemy lo usa).
        """
        # 1) Sincroniza posición
        self._sync_ai_with_enemy_pos()

        # 2) Corre el “cerebro”
        self.ai.update(dt)

        # 3) Obtén el siguiente waypoint en celdas y mueve en píxeles
        path = self.ai.get_path()
        if not path:
            return

        me_cell = pixel_to_cell(self.enemy.x, self.enemy.y, self.tile_size)
        # Si el path arranca en mi celda, toma el siguiente; si no, el primero
        idx = 1 if len(path) >= 2 and path[0] == me_cell else 0
        next_cell = path[idx]
        wx, wy = cell_center(next_cell, self.tile_size)

        # Dirección hacia el centro del tile objetivo
        dx_pix = wx - self.enemy.x
        dy_pix = wy - self.enemy.y
        dist = math.hypot(dx_pix, dy_pix)
        if dist <= 1e-3:
            return

        # Velocidad en píxeles/seg: celdas/seg * tile_size, modulada por Enemy.get_speed(peso)
        speed_factor = float(self.enemy.get_speed(weight))  # usa la versión corregida de get_speed
        pixels_per_sec = self.base_cells_per_second * self.tile_size * speed_factor
        if pixels_per_sec <= 0.0:
            return

        # Paso limitado por dt
        max_step = pixels_per_sec * dt
        step = min(max_step, dist)
        dx = (dx_pix / dist) * step
        dy = (dy_pix / dist) * step

        # 4) Mover con colisión (tu Enemy ya resuelve paredes/estamina/orientación)
        self.enemy.move_with_collision(dx, dy, game_map, weight, weather)

    # ---------- Accesores útiles (debug/dibujo) ----------
    def get_logic_cell(self) -> Cell:
        return self.ai.get_cell()

    def get_target_cell(self) -> Optional[Cell]:
        return self.ai.get_target()

    def get_current_path(self) -> List[Cell]:
        return self.ai.get_path()


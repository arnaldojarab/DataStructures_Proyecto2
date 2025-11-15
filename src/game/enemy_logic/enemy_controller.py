import math
from .. import settings

class EnemyController:
    def __init__(self, enemy, map_loader, job_logic):
        self.enemy = enemy
        self.map = map_loader
        self.job_logic = job_logic
        self.route = (0,0)

    def update(self, dt: float, difficulty: str, enemy_weight: float, weather: str):
        if difficulty == "Easy":
            self.easy_algorithm(dt, enemy_weight, weather)
        elif difficulty == "Medium":
            self.medium_algorithm(dt, enemy_weight, weather)
        elif difficulty == "Hard":
            self.hard_algorithm(dt, enemy_weight, weather)

    def easy_algorithm(self, dt: float, enemy_weight: float, weather: str):
        pass  # Implementa la lógica fácil aquí
    
    def medium_algorithm(self, dt: float, enemy_weight: float, weather: str):
        # 1) Obtener marcadores actuales
        enemy_dropoffs = self.job_logic.getEnemyDropoffMarkers()
        pickups = self.job_logic.getPickupMarkers()

        target_marker = None

        # 2) Seleccionar target: primero dropoffs enemigos, luego pickups
        #    Criterio: menor tiempo restante y, como desempate, menor distancia al enemigo.
        if enemy_dropoffs:
            """
            target_marker = min(
                enemy_dropoffs,
                key=lambda m: (
                    m["time_remaining"],
                    (m["px"] - self.enemy.x) ** 2 + (m["py"] - self.enemy.y) ** 2,
                ),
            )
            """
            # Toma el primer dropoff enemigo
            target_marker = enemy_dropoffs[0]
        elif pickups:
            """
            target_marker = min(
                pickups,
                key=lambda m: (
                    m["time_remaining"],
                    (m["px"] - self.enemy.x) ** 2 + (m["py"] - self.enemy.y) ** 2,
                ),
            )
            """
            # Si no hay dropoffs enemigos, toma el primer pickup
            target_marker = pickups[0]
        else:
            # No hay objetivos disponibles, no se mueve en este tic
            return

        # 3) Los marcadores ya usan coordenadas en píxeles (px, py), no hay que multiplicar por TILE_SIZE
        target_x = target_marker["px"]
        target_y = target_marker["py"]

        # 4) Obtener siguiente posición usando Greedy Best-First (profundidad 2)
        next_x, next_y = self.Greedy_BestFirst(self.enemy.x, self.enemy.y, target_x, target_y)

        # 5) Convertir esa posición absoluta en una dirección normalizada
        dir_x, dir_y = self.position_to_direction(next_x, next_y)

        # 6) Mover al enemigo usando la misma lógica de velocidad que el jugador
        self.moveEnemy(dir_x, dir_y, dt, enemy_weight, weather)
    
    def hard_algorithm(self, dt: float, enemy_weight: float, weather: str):
        pass  # Implementa la lógica difícil aquí

    def Greedy_BestFirst(self, enemy_x: float, enemy_y: float,
                     target_x: float, target_y: float):
      ts = settings.TILE_SIZE

      # Convertir posición actual y objetivo a coordenadas de celda
      ex = int(enemy_x // ts)
      ey = int(enemy_y // ts)
      tx = int(target_x // ts)
      ty = int(target_y // ts)

      # Heurística en coordenadas de celda
      def heuristic(gx: int, gy: int) -> float:
          dx = gx - tx
          dy = gy - ty
          return dx * dx + dy * dy

      # 8 direcciones en la grilla
      directions = [
          (1, 0),  (-1, 0),
          (0, 1),  (0, -1),
          (1, -1), (1, 1),
          (-1, -1),(-1, 1),
      ]

      best_global_score = float("inf")
      best_global_cell = (ex, ey)  # por defecto, quedarse donde está

      # Pequeña “memoria” de dirección para evitar rebotar
      prev_dx, prev_dy = getattr(self, "route", (0, 0))

      # ---------- Nivel 1 ----------
      for dx1, dy1 in directions:
          gx1 = ex + dx1
          gy1 = ey + dy1

          if self.map.is_blocked(gx1, gy1):
              continue

          best_score_for_action = None

          # ---------- Nivel 2 ----------
          for dx2, dy2 in directions:
              gx2 = gx1 + dx2
              gy2 = gy1 + dy2

              if self.map.is_blocked(gx2, gy2):
                  continue

              # ---------- Nivel 3 ----------
              best_depth3_score = None
              for dx3, dy3 in directions:
                  gx3 = gx2 + dx3
                  gy3 = gy2 + dy3

                  if self.map.is_blocked(gx3, gy3):
                      continue

                  h3 = heuristic(gx3, gy3)
                  if best_depth3_score is None or h3 < best_depth3_score:
                      best_depth3_score = h3

              # Si hubo nivel 3 válido usamos ese score, si no usamos el de nivel 2
              if best_depth3_score is not None:
                  branch_score = best_depth3_score
              else:
                  branch_score = heuristic(gx2, gy2)

              if best_score_for_action is None or branch_score < best_score_for_action:
                  best_score_for_action = branch_score

          # Si no hubo nivel 2 válido, usamos directamente la celda de nivel 1
          if best_score_for_action is None:
              best_score_for_action = heuristic(gx1, gy1)

          # Añadir penalización ligera si la acción invierte la ruta anterior
          penalty = 0.0
          if prev_dx != 0 or prev_dy != 0:
              if dx1 == -prev_dx and dy1 == -prev_dy:
                  # evitar “dar marcha atrás” salvo que sea mucho mejor
                  penalty = 0.5
              elif (dx1, dy1) != (prev_dx, prev_dy):
                  # pequeño costo por cambiar de dirección
                  penalty = 0.1

          adjusted_score = best_score_for_action + penalty

          if adjusted_score < best_global_score:
              best_global_score = adjusted_score
              best_global_cell = (gx1, gy1)

      # Guardar la dirección elegida (en celdas) para la próxima llamada
      step_dx = best_global_cell[0] - ex
      step_dy = best_global_cell[1] - ey
      self.route = (step_dx, step_dy)

      # Convertir la celda elegida a centro en píxeles
      best_px = best_global_cell[0] * ts + ts // 2
      best_py = best_global_cell[1] * ts + ts // 2

      return best_px, best_py


    def position_to_direction(self, next_x: float, next_y: float):
        dx = next_x - self.enemy.x
        dy = next_y - self.enemy.y

        length = math.hypot(dx, dy)
        if length == 0:
            return 0.0, 0.0

        return dx / length, dy / length
    
    def moveEnemy(self, dir_x: float, dir_y: float,
              dt: float, enemy_weight: float, weather: str):
      # Si no hay dirección, no se mueve
      if dir_x == 0 and dir_y == 0:
          return

      # MISMA base que el player
      base_px_per_sec = settings.TILE_SIZE * 8  # igual que en el jugador

      # Multiplicador de velocidad del enemigo (estamina + peso)
      speed_mult = self.enemy.get_speed(enemy_weight)

      # Distancia a recorrer en este frame
      distance = base_px_per_sec * dt * speed_mult

      # dir_x y dir_y ya vienen normalizados desde position_to_direction
      dx = dir_x * distance
      dy = dir_y * distance

      # El clima sigue afectando dentro de move_with_collision/map.surface_weight
      self.enemy.move_with_collision(dx, dy, self.map, enemy_weight, weather)

import math
import heapq
from .. import settings

class EnemyController:
    def __init__(self, enemy, map_loader, job_logic):
        self.enemy = enemy
        self.map = map_loader
        self.job_logic = job_logic
        self.route = (0,0)

    def update(self, dt: float, difficulty: str, enemy_weight: float, weather: str, current_multiplier : float):
        if difficulty == "Easy":
            self.easy_algorithm(dt, enemy_weight, weather, current_multiplier)
        elif difficulty == "Medium":
            self.medium_algorithm(dt, enemy_weight, weather, current_multiplier)
        elif difficulty == "Hard":
            self.hard_algorithm(dt, enemy_weight, weather, current_multiplier)

    def easy_algorithm(self, dt: float, enemy_weight: float, weather: str, current_multiplier):
        pass  # Implementa la lógica fácil aquí
    
    def medium_algorithm(self, dt: float, enemy_weight: float, weather: str, current_multiplier: float):
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

        speed_mult = self.getEnemyCurrentSpeed(enemy_weight, current_multiplier)

        dir_x *= speed_mult
        dir_y *= speed_mult



        # 6) Mover al enemigo usando la misma lógica de velocidad que el jugador
        self.moveEnemy(dir_x, dir_y, dt, enemy_weight, weather)
    
    def hard_algorithm(self, dt: float, enemy_weight: float, weather: str, current_multiplier):

        # 1) Obtener marcadores actuales (dropoffs enemigos primero)
        enemy_dropoffs = self.job_logic.getEnemyDropoffMarkers()
        pickups = self.job_logic.getPickupMarkers()

        if enemy_dropoffs:
            # Mejor dropoff: tiempo + distancia
            target_marker = min(
                enemy_dropoffs,
                key=lambda m: (
                    m["time_remaining"],
                    (m["px"] - self.enemy.x) ** 2 + (m["py"] - self.enemy.y) ** 2,
                ),
            )
        elif pickups:
            target_marker = min(
                pickups,
                key=lambda m: (
                    m["time_remaining"],
                    (m["px"] - self.enemy.x) ** 2 + (m["py"] - self.enemy.y) ** 2,
                ),
            )
        else:
            return  # No hay nada que hacer

        # 2) El target está en píxeles → convertir a tiles
        ts = settings.TILE_SIZE
        tx = int(target_marker["px"] // ts)
        ty = int(target_marker["py"] // ts)

        # Asegurar que no sea un edificio
        valid_tile = self.map.get_valid_position(tx, ty)
        if valid_tile is None:
            return  # algo raro pasó, no hay calle cerca

        tx, ty = valid_tile

        # 3) Calcular A*
        path = self.astar_tile_to_tile(tx, ty)

        if not path or len(path) < 2:
            return  # no hay ruta

        # El siguiente paso es path[1]
        nx, ny = path[1]

        # 4) Convertir a píxeles centro
        next_x = nx * ts + ts // 2
        next_y = ny * ts + ts // 2

        # 5) Convertir a dirección
        dir_x, dir_y = self.position_to_direction(next_x, next_y)

        speed_mult = self.getEnemyCurrentSpeed(enemy_weight, current_multiplier)

        dir_x *= speed_mult
        dir_y *= speed_mult

        # 6) Mover enemigo con tu sistema completo
        self.moveEnemy(dir_x, dir_y, dt, enemy_weight, weather)



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

    def astar_tile_to_tile(self, goal_tx, goal_ty):
        ts = settings.TILE_SIZE

        # convertir posición del enemigo a tile
        start_tx = int(self.enemy.x // ts)
        start_ty = int(self.enemy.y // ts)

        # si está dentro de edificio, corregir
        valid = self.map.get_valid_position(start_tx, start_ty)
        if valid is None:
            return None
        start_tx, start_ty = valid

        # A*
        open_heap = []
        heapq.heappush(open_heap, (0, (start_tx, start_ty)))  # (f_score, (tx, ty))

        came_from = {}
        g_score = { (start_tx, start_ty): 0 }

        # heurística: distancia Euclidiana
        def h(tx, ty):
            dx = tx - goal_tx
            dy = ty - goal_ty
            return (dx*dx + dy*dy) ** 0.5

        f_score = { (start_tx, start_ty): h(start_tx, start_ty) }

        # 8 direcciones
        directions = [
            (1, 0),  (-1, 0),
            (0, 1),  (0, -1),
            (1, -1), (1, 1),
            (-1, -1),(-1, 1),
        ]

        visited = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)
            cx, cy = current

            if current in visited:
                continue
            visited.add(current)

            # Llegó al destino
            if current == (goal_tx, goal_ty):
                return self.reconstruct_path(came_from, current)

            # Expandir vecinos
            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy

                # bloqueado por edificio
                if self.map.is_blocked(nx, ny):
                    continue

                # Costo de superficie
                px = nx * settings.TILE_SIZE
                py = ny * settings.TILE_SIZE

                sw = self.map.surface_weight(px, py)

                # Penalización adicional si es parque
                if self.map.is_park(nx, ny):
                    sw *= 0.3

                # Evitar 0
                sw = max(sw, 0.1)

                # Convertir surface_weight a costo → más lento = mayor costo
                max_sw = 10.0
                terrain_cost = max_sw / sw

                # Diagonal más larga
                move_cost = 1.41 if dx != 0 and dy != 0 else 1.0

                tentative_g = g_score[(cx, cy)] + terrain_cost * move_cost

                # Mejor camino encontrado hacia (nx, ny)
                if tentative_g < g_score.get((nx, ny), 1e9):
                    came_from[(nx, ny)] = (cx, cy)
                    g_score[(nx, ny)] = tentative_g
                    f = tentative_g + h(nx, ny)
                    f_score[(nx, ny)] = f
                    heapq.heappush(open_heap, (f, (nx, ny)))

        return None  # si no hay camino
    

    def getEnemyCurrentSpeed(self, enemy_weight: float, weather_multiplier: float) -> float:

        enemy_speed = self.enemy.get_speed(enemy_weight)
        surface_weight = self.map.surface_weight(self.enemy.x, self.enemy.y)
        rep_speed = self.job_logic.getEnemyRepSpeed() 

        return weather_multiplier * enemy_speed * surface_weight* rep_speed

        
    
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
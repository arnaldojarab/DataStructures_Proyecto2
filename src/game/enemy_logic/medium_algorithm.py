from .. import settings
from .enemy import moveEnemy, position_to_direction
from collections import deque
import heapq


class MediumAlgorithm:
    def __init__(self, enemy, map):
        self.enemy = enemy
        self.map = map
        self.change_dir_timer = 0
        self.next_x, self.next_y = 0, 0

        # Últimas 3 celdas reales donde ha estado el enemigo (en coordenadas de grilla)
        self.last_cells = deque(maxlen=3)

    def update(self, dt: float, dropoffs, pickups, enemy_weight: float, weather: str, speed_mult: float):
        target_marker = None
        # 2) Seleccionar target: primero dropoffs enemigos, luego pickups
        if dropoffs:
            # Toma el primer dropoff enemigo
            target_marker = dropoffs[0]
        elif pickups:
            # Si no hay dropoffs enemigos, toma el primer pickup
            target_marker = pickups[0]
        else:
            return

        # 3) Los marcadores ya usan coordenadas en píxeles (px, py)
        target_x = target_marker["px"]
        target_y = target_marker["py"]

        ts = settings.TILE_SIZE

        # Registrar la celda actual del enemy en el historial (posición REAL)
        ex = int(self.enemy.x // ts)
        ey = int(self.enemy.y // ts)
        self.last_cells.append((ex, ey))

        # 4) Obtener siguiente posición usando Greedy Best-First (profundidad 2)
        self.change_dir_timer -= 1
        if self.change_dir_timer <= 0:
            self.change_dir_timer = 5
            self.next_x, self.next_y = self.Greedy_BestFirst_v5(self.enemy.x, self.enemy.y, target_x, target_y)

        # 5) Convertir esa posición absoluta en una dirección normalizada
        dir_x, dir_y = position_to_direction(self.next_x, self.next_y, self.enemy.x, self.enemy.y)

        dir_x *= speed_mult
        dir_y *= speed_mult

        moveEnemy(self.enemy, self.map, dir_x, dir_y, dt, enemy_weight, weather)

    def Greedy_BestFirst_v5(self, enemy_x, enemy_y, target_x, target_y):
        """
        Devuelve la posición en píxeles de la siguiente celda a la que el enemigo
        debería moverse, usando Greedy Best-First Search sobre la grilla
        con mirada de 2 pasos.

        Intenta evitar las últimas celdas visitadas (self.last_cells),
        pero si no hay alternativa razonable, puede usarlas.
        """
        ts = settings.TILE_SIZE

        # Pasar de píxeles a coordenadas de celda
        ex = int(enemy_x // ts)
        ey = int(enemy_y // ts)
        tx = int(target_x // ts)
        ty = int(target_y // ts)

        # Movimientos posibles: 4 direcciones (arriba, abajo, izquierda, derecha)
        directions = [
            (1, 0),   # derecha
            (-1, 0),  # izquierda
            (0, 1),   # abajo
            (0, -1),  # arriba
        ]

        # Heurística: distancia cuadrática en la grilla (no hace falta sqrt)
        def dist2(cx, cy, tx, ty):
            dx = cx - tx
            dy = cy - ty
            return dx * dx + dy * dy

        # Mejor paso evitando historial
        best_first_step_avoid = None
        best_score_avoid = None

        # Mejor paso general (por si todas las opciones buenas están en historial)
        best_first_step_any = None
        best_score_any = None

        # Recorremos todos los posibles primeros pasos
        for dx1, dy1 in directions:
            nx1 = ex + dx1
            ny1 = ey + dy1

            # Si la primera celda está bloqueada u fuera del mapa, se ignora
            if self.map.is_blocked(nx1, ny1):
                continue

            # Distancia mínima alcanzable en hasta 2 pasos, empezando por esta celda
            best_local = dist2(nx1, ny1, tx, ty)

            # Miramos un segundo paso desde esa celda
            for dx2, dy2 in directions:
                nx2 = nx1 + dx2
                ny2 = ny1 + dy2

                # Evitar considerar volver exactamente a la celda inicial
                if nx2 == ex and ny2 == ey:
                    continue

                if self.map.is_blocked(nx2, ny2):
                    continue

                d2 = dist2(nx2, ny2, tx, ty)
                if d2 < best_local:
                    best_local = d2

            # Actualizar mejor candidato "cualquiera"
            if best_score_any is None or best_local < best_score_any:
                best_score_any = best_local
                best_first_step_any = (nx1, ny1)

            # Actualizar mejor candidato evitando las últimas celdas
            if (nx1, ny1) not in self.last_cells:
                if best_score_avoid is None or best_local < best_score_avoid:
                    best_score_avoid = best_local
                    best_first_step_avoid = (nx1, ny1)

        # Elegir primero el mejor que evita historial; si no hay, usar el mejor general
        chosen_step = best_first_step_avoid if best_first_step_avoid is not None else best_first_step_any

        # Si no hay ningún vecino válido, nos quedamos quietos
        if chosen_step is None:
            return enemy_x, enemy_y

        step_x, step_y = chosen_step

        # OJO: no añadimos aquí a last_cells; solo guardamos posiciones reales en update()

        # Convertir la celda elegida a centro en píxeles
        best_px = step_x * ts + ts // 2
        best_py = step_y * ts + ts // 2

        return best_px, best_py



    def Greedy_BestFirst(self, enemy_x, enemy_y, target_x, target_y):
      """
      Devuelve la posición en píxeles de la siguiente celda a la que el enemigo
      debería moverse, usando Greedy Best-First Search sobre la grilla.
      """
      
      ts = settings.TILE_SIZE

      # Pasar de píxeles a coordenadas de celda
      ex = int(enemy_x // ts)
      ey = int(enemy_y // ts)
      tx = int(target_x // ts)
      ty = int(target_y // ts)

      # Si ya está en la celda del objetivo, devolvemos el propio objetivo
      if ex == tx and ey == ty:
          return target_x, target_y

      # Heurística: distancia al cuadrado en la grilla
      def heuristic(gx, gy):
          dx = gx - tx
          dy = gy - ty
          return dx * dx + dy * dy

      # 8 direcciones (4 cardinales + 4 diagonales)
      directions = [
          (1, 0),  (-1, 0),
          (0, 1),  (0, -1),
          (1, 1),  (1, -1),
          (-1, 1), (-1, -1),
      ]

      start = (ex, ey)
      start_h = heuristic(ex, ey)

      # Cola de prioridad: (heurística, (x, y))
      open_heap = []
      heapq.heappush(open_heap, (start_h, start))

      # Padres para reconstruir el primer paso
      parents = {start: None}

      # Para no caer en ciclos
      visited = set()

      # Mejor celda vista hasta ahora según heurística
      best_cell = start
      best_h = start_h

      # Límite de expansiones para que no se dispare el coste
      max_expansions = 256
      expansions = 0

      while open_heap and expansions < max_expansions:
          current_h, (cx, cy) = heapq.heappop(open_heap)

          if (cx, cy) in visited:
              continue

          visited.add((cx, cy))
          expansions += 1

          # Actualizar mejor celda
          if current_h < best_h:
              best_h = current_h
              best_cell = (cx, cy)

          # Si llegamos al objetivo, terminamos
          if cx == tx and cy == ty:
              best_cell = (cx, cy)
              break

          # Expandir vecinos
          for dx, dy in directions:
              nx = cx + dx
              ny = cy + dy
              neighbor = (nx, ny)

              # is_blocked también se encarga de los límites del mapa
              if self.map.is_blocked(nx, ny):
                  continue
              if neighbor in visited:
                  continue

              if neighbor not in parents:
                  parents[neighbor] = (cx, cy)

              h = heuristic(nx, ny)
              heapq.heappush(open_heap, (h, neighbor))

      # Reconstruir el primer paso desde start hasta best_cell
      current = best_cell

      # Si no hay nada mejor, nos quedamos en la misma celda
      if current == start:
          px = ex * ts + ts // 2
          py = ey * ts + ts // 2
          return px, py

      # Subimos por la cadena de padres hasta el vecino inmediato de start
      while parents.get(current) is not None and parents.get(current) != start:
          current = parents[current]

      step_x, step_y = current

      # Convertir la celda elegida a centro en píxeles
      best_px = step_x * ts + ts // 2
      best_py = step_y * ts + ts // 2

      return best_px, best_py

    def Greedy_BestFirst_v4(self, enemy_x: float, enemy_y: float,
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
    
    def update_v3(self, dt: float, dropoffs, pickups, enemy_weight: float, weather: str, speed_mult: float):
        # 1) Elegir target: primero dropoffs enemigos, luego pickups
        target_marker = None
        if dropoffs:
            target_marker = dropoffs[0]
        elif pickups:
            target_marker = pickups[0]
        else:
            # No hay nada que hacer
            return

        target_x = target_marker["px"]
        target_y = target_marker["py"]

        # 2) Obtener siguiente posición objetivo (centro de la celda siguiente)
        next_x, next_y = self.Greedy_BestFirst(self.enemy.x, self.enemy.y, target_x, target_y)

        # 3) Convertir esa posición absoluta en una dirección normalizada
        dir_x, dir_y = position_to_direction(next_x, next_y, self.enemy.x, self.enemy.y)

        # Escalar por velocidad
        dir_x *= speed_mult
        dir_y *= speed_mult

        # 4) Mover enemigo usando el sistema de colisiones existente
        moveEnemy(self.enemy, self.map, dir_x, dir_y, dt, enemy_weight, weather)

    def Greedy_BestFirst_v3(self, enemy_x, enemy_y, target_x, target_y):
        # Convertir coordenadas a celdas del grid
        start_cell = (
            int(enemy_x // settings.TILE_SIZE),
            int(enemy_y // settings.TILE_SIZE)
        )
        target_cell = (
            int(target_x // settings.TILE_SIZE),
            int(target_y // settings.TILE_SIZE)
        )

        # Si ya estamos en la celda objetivo, devolvemos la posición actual
        if start_cell == target_cell:
            return target_x, target_y

        # Movimientos posibles (arriba, derecha, abajo, izquierda)
        directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
        
        # Lista de movimientos válidos y sus distancias
        valid_moves = []
        
        # Evaluar vecinos inmediatos
        for dx, dy in directions:
            neighbor_cell = (start_cell[0] + dx, start_cell[1] + dy)
            
            # Verificar si el vecino es válido usando el método del mapa
            if not self.map.is_blocked(neighbor_cell[0], neighbor_cell[1]):
                # Calcular distancia Manhattan al objetivo
                distance = abs(neighbor_cell[0] - target_cell[0]) + abs(neighbor_cell[1] - target_cell[1])
                valid_moves.append((neighbor_cell, distance))

        # Si no hay movimientos válidos, nos quedamos en la misma celda
        if not valid_moves:
            # Intentamos movimientos alternativos si estamos atascados
            return self.handle_stuck_situation(start_cell, target_cell)
        
        # Ordenar movimientos por distancia (más cercanos primero)
        valid_moves.sort(key=lambda x: x[1])
        
        # Elegir el mejor movimiento
        best_cell = valid_moves[0][0]
        
        # Convertir celda elegida a coordenadas absolutas (centro de la celda)
        best_x = best_cell[0] * settings.TILE_SIZE + settings.TILE_SIZE // 2
        best_y = best_cell[1] * settings.TILE_SIZE + settings.TILE_SIZE // 2

        return best_x, best_y

    def handle_stuck_situation(self, current_cell, target_cell):
        """Maneja situaciones donde el enemigo está atascado"""
        # Movimientos en orden de prioridad cuando estamos atascados
        directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
        
        # Primero, intentamos cualquier movimiento que nos aleje del obstáculo
        for dx, dy in directions:
            neighbor_cell = (current_cell[0] + dx, current_cell[1] + dy)
            if not self.map.is_blocked(neighbor_cell[0], neighbor_cell[1]):
                best_x = neighbor_cell[0] * settings.TILE_SIZE + settings.TILE_SIZE // 2
                best_y = neighbor_cell[1] * settings.TILE_SIZE + settings.TILE_SIZE // 2
                return best_x, best_y
        
        # Si no hay ningún movimiento posible, nos quedamos donde estamos
        best_x = current_cell[0] * settings.TILE_SIZE + settings.TILE_SIZE // 2
        best_y = current_cell[1] * settings.TILE_SIZE + settings.TILE_SIZE // 2
        return best_x, best_y



    def update_v2(self, dt: float, dropoffs, pickups, enemy_weight: float, weather: str, speed_mult: float):
        # 1) Elegir target: primero dropoffs enemigos, luego pickups
        target_marker = None
        if dropoffs:
            target_marker = dropoffs[0]
        elif pickups:
            target_marker = pickups[0]
        else:
            # No hay nada que hacer
            return

        target_x = target_marker["px"]
        target_y = target_marker["py"]

        # 2) Obtener siguiente posición objetivo (centro de la celda siguiente)
        next_x, next_y = self.Greedy_BestFirst(self.enemy.x, self.enemy.y, target_x, target_y)

        # 3) Convertir esa posición absoluta en una dirección normalizada
        dir_x, dir_y = position_to_direction(next_x, next_y, self.enemy.x, self.enemy.y)

        # Escalar por velocidad
        dir_x *= speed_mult
        dir_y *= speed_mult

        # 4) Mover enemigo usando el sistema de colisiones existente
        moveEnemy(self.enemy, self.map, dir_x, dir_y, dt, enemy_weight, weather)

    def Greedy_BestFirst_v2(self, enemy_x, enemy_y, target_x, target_y):
        """
        Devuelve la posición en píxeles del centro de la siguiente celda
        a la que el enemigo debería moverse.

        Lógica:
        - BFS (cola normal) desde la celda actual hasta un límite de expansiones.
        - Si se alcanza el objetivo, se reconstruye el camino y se toma el primer paso.
        - Si no se alcanza, se elige la celda visitada más cercana al objetivo
          y se toma el primer paso hacia ella.
        - Si aun así no hay nada mejor, se intenta al menos ir a un vecino libre
          que acerque al objetivo. Solo se queda quieto si está totalmente encerrado.
        """

        ts = settings.TILE_SIZE

        # Pasar de píxeles a coordenadas de celda
        ex = int(enemy_x // ts)
        ey = int(enemy_y // ts)
        tx = int(target_x // ts)
        ty = int(target_y // ts)

        start = (ex, ey)
        goal = (tx, ty)

        # Si ya está en la celda del objetivo, ir directo al objetivo en píxeles
        if start == goal:
            return target_x, target_y

        # Heurística simple: distancia Manhattan en la grilla
        def heuristic(cx, cy):
            return abs(cx - tx) + abs(cy - ty)

        # Vecinos 4-direccionales (más estable y simple)
        directions = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
        ]

        # BFS
        queue = deque()
        queue.append(start)

        parents = {start: None}
        visited = {start}

        max_expansions = 256
        expansions = 0
        found_goal = False

        # Para fallback: mejor celda vista según la heurística
        best_node = start
        best_h = heuristic(ex, ey)

        while queue and expansions < max_expansions:
            cx, cy = queue.popleft()
            current = (cx, cy)
            expansions += 1

            # Actualizar mejor nodo visto
            h_cur = heuristic(cx, cy)
            if h_cur < best_h:
                best_h = h_cur
                best_node = current

            # ¿Llegamos al objetivo?
            if current == goal:
                found_goal = True
                best_node = current
                break

            # Expandir vecinos
            for dx, dy in directions:
                nx = cx + dx
                ny = cy + dy
                neighbor = (nx, ny)

                # Bloqueado o fuera del mapa
                if self.map.is_blocked(nx, ny):
                    continue

                if neighbor in visited:
                    continue

                visited.add(neighbor)
                parents[neighbor] = current
                queue.append(neighbor)

        # Elegir destino para reconstruir el primer paso
        if found_goal:
            dest = goal
        else:
            # Si no se llegó al objetivo, usar la mejor celda vista
            dest = best_node

        # Reconstruir el primer paso desde start hasta dest
        current = dest

        if current == start:
            # No hay camino "mejor" descubierto por BFS.
            # Intentar al menos ir a un vecino libre que acerque al objetivo.
            best_neighbor = None
            best_neighbor_h = float("inf")

            for dx, dy in directions:
                nx = ex + dx
                ny = ey + dy
                if self.map.is_blocked(nx, ny):
                    continue

                h = heuristic(nx, ny)
                if h < best_neighbor_h:
                    best_neighbor_h = h
                    best_neighbor = (nx, ny)

            if best_neighbor is None:
                # Encerrado por obstáculos: quedarse en la misma celda
                step_x, step_y = ex, ey
            else:
                step_x, step_y = best_neighbor
        else:
            # Subir por padres hasta el vecino inmediato de start
            while parents.get(current) is not None and parents[current] != start:
                current = parents[current]

            step_x, step_y = current

        # Convertir la celda elegida a centro en píxeles
        best_px = step_x * ts + ts // 2
        best_py = step_y * ts + ts // 2

        return best_px, best_py


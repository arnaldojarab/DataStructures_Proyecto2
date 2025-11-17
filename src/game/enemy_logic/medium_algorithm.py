from .. import settings
from .enemy import moveEnemy, position_to_direction
from collections import deque

class MediumAlgorithm:
    def __init__(self, enemy, map):
        self.enemy = enemy
        self.map = map
        self.change_dir_timer = 0
        self.next_x, self.next_y = 0, 0

        # Últimas 3 celdas reales donde ha estado el enemigo (en coordenadas de grilla)
        self.last_cells = deque(maxlen=5)

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

        # 4) Obtener siguiente posición usando Greedy Best-First (profundidad 2)
        self.change_dir_timer -= 1
        if self.change_dir_timer <= 0:
            ex = int(self.enemy.x // ts)
            ey = int(self.enemy.y // ts)
            self.last_cells.append((ex, ey))
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


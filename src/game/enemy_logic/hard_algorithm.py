from .. import settings
from .enemy import moveEnemy, position_to_direction
import heapq

class HardAlgorithm:
    def __init__(self, enemy, map):
        self.enemy = enemy
        self.map = map

     

    def update(self, dt: float, dropoffs, pickups, enemy_weight: float, weather: str, speed_mult: float):
        target_marker = None

        if dropoffs:
            # Si hay algún dropoff, fija como objetivo el más cercano
            target_marker = min(
                dropoffs,
                key=lambda m: (
                    m["time_remaining"],
                    (m["px"] - self.enemy.x) ** 2 + (m["py"] - self.enemy.y) ** 2,
                ),
            )
        elif pickups:
            # Si no hay dropoffs, fija como objetivo el pickups más cercano
            target_marker = min(
                pickups,
                key=lambda m: (
                    m["time_remaining"],
                    (m["px"] - self.enemy.x) ** 2 + (m["py"] - self.enemy.y) ** 2,
                ),
            )
        else:
            return  # No hay nada que hacer, se queda quieto

        # 2) La ubicación del pedido lo pasa a tiles
        ts = settings.TILE_SIZE
        tx = int(target_marker["px"] // ts)
        ty = int(target_marker["py"] // ts)

        # Asegurar que no sea un edificio, retorna la calle más cercano al lugar del pedido
        valid_tile = self.map.get_valid_position(tx, ty)
        if valid_tile is None:
            return  # algo raro pasó, no hay calle cerca

        tx, ty = valid_tile

        # 3) Calcular A*
        path = self.astar_tile_to_tile(tx, ty)

        if not path or len(path) < 2:
            return  # No hay ruta

        # El siguiente paso es path[1]
        nx, ny = path[1]

        # Convertir a píxeles centro
        next_x = nx * ts + ts // 2
        next_y = ny * ts + ts // 2

        # Convertir a dirección
        dir_x, dir_y = position_to_direction(next_x, next_y, self.enemy.x, self.enemy.y)

        dir_x *= speed_mult
        dir_y *= speed_mult

        # Mover enemigo
        moveEnemy(self.enemy, self.map, dir_x, dir_y, dt, enemy_weight, weather)

    def astar_tile_to_tile(self, goal_tx, goal_ty):
        ts = settings.TILE_SIZE

        # Pasa la posición del enemigo a tiles
        start_tx = int(self.enemy.x // ts)
        start_ty = int(self.enemy.y // ts)

        # Si está dentro de edificio, se corrige
        valid = self.map.get_valid_position(start_tx, start_ty)
        if valid is None:
            return None
        start_tx, start_ty = valid

        # A*
        open_heap = []
        heapq.heappush(open_heap, (0, (start_tx, start_ty)))  # (f_score, (tx, ty))

        came_from = {}
        g_score = { (start_tx, start_ty): 0 }

        # heurística
        def h(tx, ty):
            dx = tx - goal_tx
            dy = ty - goal_ty
            return (dx*dx + dy*dy) ** 0.5

        f_score = { (start_tx, start_ty): h(start_tx, start_ty) }

        # 8 direcciones posibles
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

                # Bloqueado por edificio
                if self.map.is_blocked(nx, ny):
                    continue

                # Costo de superficie
                px = nx * settings.TILE_SIZE
                py = ny * settings.TILE_SIZE

                sw = self.map.surface_weight(px, py)
                
                # Penalización adicional si es parque
                if self.map.is_park(nx, ny):
                    sw *= 0.3

                # Evitar 0, para no dividir entre 0
                sw = max(sw, 0.1)

                # Convertir surface_weight a costo 
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
    
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    
import random
import pygame
import math
from .. import settings
from .enemy import moveEnemy
from collections import deque

class EasyAlgorithm:
    def __init__(self, enemy, map):
        self.enemy = enemy
        self.map = map
        self.change_dir_timer = 0
        self.direction = pygame.Vector2(0, 0)
        self.job_timer = 0.0   

        self.dropoff_queue = deque()   
        self.pickup_queue = deque()

        self.current_job = None        
        self.current_job_type = None 



    def _sync_queues(self, dropoffs, pickups):
        
        """
        Sincroniza las colas internas con los markers que existen en el mapa.
        """

        def sync_one(active_list, queue: deque):
            # 1) Mantener en la cola solo los que siguen existiendo
            new_q = deque([m for m in queue if m in active_list])

            # 2) Añadir al final los nuevos que aún no estaban en la cola
            for marker in active_list:
                if marker not in new_q:
                    new_q.append(marker)

            return new_q

        self.dropoff_queue = sync_one(dropoffs, self.dropoff_queue)
        self.pickup_queue = sync_one(pickups, self.pickup_queue)

    def _choose_new_job(self):

        """
        Elige un nuevo trabajo usando las colas
        """
        opciones = []
        if self.dropoff_queue:
            opciones.append("dropoff")
        if self.pickup_queue:
            opciones.append("pickup")

        if not opciones:
            self.current_job = None
            self.current_job_type = None
            self.job_timer = 0.0
            return

        job_type = random.choice(opciones)

        if job_type == "dropoff":
            marker = self.dropoff_queue.popleft()
        else:
            marker = self.pickup_queue.popleft()

        self.current_job = marker
        self.current_job_type = job_type

        # Tiempo máximo que el bot se queda con este objetivo (segundos)
        self.job_timer = random.uniform(5.0, 15.0)

    def update(self, dt: float, dropoffs, pickups,
               enemy_weight: float, weather: str, speed_mult: float):

        self._sync_queues(dropoffs, pickups)

        # 2) Actualizar timers
        self.change_dir_timer -= dt
        self.job_timer -= dt

        # 3) Invalidar el trabajo actual si ya no existe en el mundo
        if self.current_job is not None:
            if self.current_job_type == "dropoff" and self.current_job not in dropoffs:
                self.current_job = None
                self.current_job_type = None
                self.job_timer = 0.0
            elif self.current_job_type == "pickup" and self.current_job not in pickups:
                self.current_job = None
                self.current_job_type = None
                self.job_timer = 0.0

        # 4) ¿Hay que elegir / rerrollear trabajo?
        if self.current_job is None or self.job_timer <= 0:
            # Si el trabajo caducó pero sigue siendo válido, lo reencolamos al final
            if self.current_job is not None and self.job_timer <= 0:
                if self.current_job_type == "dropoff" and self.current_job in dropoffs:
                    self.dropoff_queue.append(self.current_job)
                elif self.current_job_type == "pickup" and self.current_job in pickups:
                    self.pickup_queue.append(self.current_job)

            # Elegimos un nuevo trabajo desde las colas
            self._choose_new_job()

        target_marker = self.current_job

        if target_marker == None:
            return



        # Usamos el timer en segundos, decreciendo con dt
        self.change_dir_timer -= 1
        if self.change_dir_timer <= 0:
            self.direction = self.get_random_valid_direction()
            self.change_dir_timer = random.randint(30, 90)

        # Si no hay dirección válida (rodeado de bloques), no nos movemos
        if self.direction.x == 0 and self.direction.y == 0:
            return

        # Escalar dirección por la velocidad
        dir_x = self.direction.x * speed_mult
        dir_y = self.direction.y * speed_mult

        # Mover usando la misma función de colisión/clima/peso
        moveEnemy(self.enemy, self.map, dir_x, dir_y, dt, enemy_weight, weather)

    # Este metodo solo puede moverse en el eje x o solo en el y
    def get_random_valid_direction(self):
        dirs = [
            pygame.Vector2(1, 0),    # derecha
            pygame.Vector2(-1, 0),   # izquierda
            pygame.Vector2(0, 1),    # abajo
            pygame.Vector2(0, -1),   # arriba
            pygame.Vector2(1, 1),    # diagonal abajo-derecha
            pygame.Vector2(-1, 1),   # diagonal abajo-izquierda
            pygame.Vector2(1, -1),   # diagonal arriba-derecha
            pygame.Vector2(-1, -1),  # diagonal arriba-izquierda
        ]
        random.shuffle(dirs)

        tile_x = int(self.enemy.x // settings.TILE_SIZE)
        tile_y = int(self.enemy.y // settings.TILE_SIZE)

        for d in dirs:
            nx = tile_x + int(d.x)
            ny = tile_y + int(d.y)

            # si en un asquina esta libre, pero al rededor de la diagonal esta bloqueda no pasa
            if abs(d.x) == 1 and abs(d.y) == 1:
                if (self.map.is_blocked(tile_x + int(d.x), tile_y) or
                    self.map.is_blocked(tile_x, tile_y + int(d.y))):
                    continue  # salta esa diagonal

            # si el tile destino no está bloqueado, se usa
            if not self.map.is_blocked(nx, ny):
                return d.normalize()  # es para mantener velocidad constante por si elige una diagonal

        # si todas están bloqueadas, no moverse
        return pygame.Vector2(0, 0)

    
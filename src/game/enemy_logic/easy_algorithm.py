import random
import pygame
import math
from .. import settings
from .enemy import moveEnemy

class EasyAlgorithm:
    def __init__(self, enemy, map):
        self.enemy = enemy
        self.map = map
        self.change_dir_timer = 0
        self.direction = pygame.Vector2(0, 0)

    def update(self, dt: float, dropoffs, pickups, enemy_weight: float, weather: str, speed_mult: float):
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

    
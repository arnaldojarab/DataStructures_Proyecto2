from collections import deque
import pygame
import os
import random
import math
from .enemy import Enemy

from .. import settings


class EnemyControl:
    def __init__(self, enemy=None, game_map=None):
        self.enemy = enemy or Enemy()
        self.game_map = game_map
        self.change_dir_timer = 0
        self.direction = pygame.Vector2(0, 0)
        self.speed = 2  # píxeles por frame aprox., puedes ajustarlo

    def update(self, dt, diffuculty):
        pass
        

    def MoveEnemy(self, diff,weight, weather):
        if diff == "easy":
            self.test(weight, weather)
        elif diff == "medium":
            self.GreedyPath()
        elif diff == "hard":
            self.DijkstraPath()

    def test(self, weight, weather):
        self.change_dir_timer -= 1

        # cambiar de dirección 
        if self.change_dir_timer <= 0:
            self.direction = self.get_random_valid_direction_v2()
            self.change_dir_timer = random.randint(30, 90)  # Pone el timepo en el que va a esa direccion

        # se mueve a la dirección actual
        dx = self.direction.x * self.speed
        dy = self.direction.y * self.speed
        self.enemy.move_with_collision(dx, dy, self.game_map, weight, weather)

    def get_random_valid_direction_v2(self): #Este metodo puede moverse en diagonal
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
                if (self.game_map.is_blocked(tile_x + int(d.x), tile_y) or
                    self.game_map.is_blocked(tile_x, tile_y + int(d.y))):
                    continue  # salta esa diagonal

            # si el tile destino no está bloqueado, se usa
            if not self.game_map.is_blocked(nx, ny):
                return d.normalize()  # es para mantener velocidad constante por si elige una diagonal

        # si todas están bloqueadas, no moverse
        return pygame.Vector2(0, 0)
    
    # Este metodo solo puede moverse en el eje x o solo en el y
    def get_random_valid_direction(self):

        # direcciones posibles: izquierda, derecha, arriba, abajo
        dirs = [
            pygame.Vector2(1, 0),
            pygame.Vector2(-1, 0),
            pygame.Vector2(0, 1),
            pygame.Vector2(0, -1),
        ]
        random.shuffle(dirs)

        tile_x = int(self.enemy.x // settings.TILE_SIZE)
        tile_y = int(self.enemy.y // settings.TILE_SIZE)

        for d in dirs:
            nx = tile_x + int(d.x)
            ny = tile_y + int(d.y)
            if not self.game_map.is_blocked(nx, ny):
                return d

        # si todas están bloqueadas, no moverse
        return pygame.Vector2(0, 0)

    def RandomPath(self):
        pass

    def GreedyPath(self):
        pass

    def DijkstraPath(self):
        pass




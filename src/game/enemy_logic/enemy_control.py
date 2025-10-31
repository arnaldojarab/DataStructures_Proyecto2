from collections import deque
import pygame
import os
import math
from enemy import Enemy

from .. import settings


class EnemyControl:
    def __init__(self):

        self.enemy = Enemy()


    def MoveEnemy(self, diff):

        if diff == "easy":
            self.RandomPath()
        
        elif diff == "medium":
            self.GreedyPath()
        
        elif diff == "hard":
            self.DijkstraPath()

        else:
            pass



    def RandomPath(self):
        pass

    def GreedyPath(self):
        pass

    def DijkstraPath(self):
        pass




from .enemy_controller import EnemyController, RandomStrategy
from typing import Optional
from .. import settings

class EnemyLogic:
    def __init__(self, enemy, map_loader, job_logic, strategy: Optional[RandomStrategy]=None):
        # instancia el controller que orquesta la IA
        self.enemy = enemy
        self.map = map_loader
        self.job_logic = job_logic

        
        self.enemy_controller = EnemyController(
    enemy=self.enemy,
    map_loader=self.map,
    job_logic=self.job_logic,
    strategy=RandomStrategy(self.job_logic, self.map, settings.TILE_SIZE, debug=True),
)

    def MoveEnemy(self, difficulty: str, enemy_weight: float, weather: str, dt: float):
        # difficulty por ahora no se usa; en el futuro podría seleccionar strategy.
        self.enemy_controller.update(dt, enemy_weight, weather)
from .medium_algorithm import MediumAlgorithm
from .hard_algorithm import HardAlgorithm
from .easy_algorithm import EasyAlgorithm

class EnemyController:
    def __init__(self, enemy, map_loader, job_logic):
        self.enemy = enemy
        self.map = map_loader
        self.job_logic = job_logic

        # algorithms
        self.easy_algorithm = EasyAlgorithm(self.enemy, self.map)
        self.medium_algorithm = MediumAlgorithm(self.enemy, self.map)
        self.hard_algorithm = HardAlgorithm(self.enemy, self.map)

    def update(self, dt: float, difficulty: str, enemy_weight: float, weather: str, current_multiplier : float):
        pickups = self.job_logic.getPickupMarkers()
        dropoffs = self.job_logic.getEnemyDropoffMarkers()
        speed_mult = self.getEnemyCurrentSpeed(enemy_weight, current_multiplier)

        if difficulty == "Easy":
            self.easy_algorithm.update(dt, dropoffs, pickups, enemy_weight, weather, speed_mult)
        elif difficulty == "Medium":
            self.medium_algorithm.update(dt, dropoffs, pickups, enemy_weight, weather, speed_mult)
        elif difficulty == "Hard":
            self.hard_algorithm.update(dt, dropoffs, pickups, enemy_weight, weather, speed_mult)

    def getEnemyCurrentSpeed(self, enemy_weight: float, weather_multiplier: float) -> float:
        enemy_speed = self.enemy.get_speed(enemy_weight)
        surface_weight = self.map.surface_weight(self.enemy.x, self.enemy.y)
        rep_speed = self.job_logic.getEnemyRepSpeed() 

        return weather_multiplier * enemy_speed * surface_weight* rep_speed
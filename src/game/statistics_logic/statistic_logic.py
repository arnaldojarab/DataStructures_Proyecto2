from __future__ import annotations
import pygame
from typing import Literal, Optional
import os

from .. import settings
from ..util import format_mmss, CountdownTimer  # usas estos en tu engine actual

AlignX = Literal["left", "center", "right"]
AlignY = Literal["top", "center", "bottom"]

class statisticLogic:
    """
    Administra estadísticas del HUD. 
    """

    def __init__(
        self,
        align_x: AlignX = "center",
        align_y: AlignY = "top",
        margin_x: int = 0,
        margin_y: int = 5,
        font: Optional[pygame.font.Font] = None,
    ) -> None:
        pygame.font.init()
        self._timer = CountdownTimer(settings.TIMER_START_SECONDS)
        self._font = font or pygame.font.Font(settings.UI_FONT_NAME, settings.UI_FONT_SIZE)
        self._font_stats = pygame.font.Font(settings.UI_FONT_NAME, settings.STATS_FONT_SIZE)

        # Posicionamiento del temporizador (p. ej. centrado arriba)
        self._align_x: AlignX = align_x
        self._align_y: AlignY = align_y
        self._margin_x = margin_x
        self._margin_y = margin_y

        self._money: float = 1000.0
        self._reputation: int = 70
        self._meta_ingresos: float = float(settings.META_INGRESOS)

        self._moneyEnemy: float = 0.0
        self._reputationEnemy: int = 70
        self._meta_ingresosEnemy: float = float(settings.META_INGRESOS)

        self.border_image = self._load_Border()

    # ---------------- API pública mínima ----------------
    def reset(self) -> None:
        """Reinicia TODAS las estadísticas manejadas por esta clase."""
        self._reset_timer()
        self._reset_money()
        self._reset_reputation()  

    def update(self, dt: float, money: float = 0.0, reputation: int = 70, moneyEnemy: float = 0.0, reputationEnemy: int = 70) -> None:
        """Actualiza TODAS las estadísticas frame a frame."""
        self._update_timer(dt)
        self._update_money(money)
        self._update_reputation(reputation)
        self._update_moneyEnemy(moneyEnemy)
        self._update_reputationEnemy(reputationEnemy)
        


    def draw(self, surface: pygame.Surface) -> None:
        """Dibuja todas las estadísticas en pantalla."""
        self._draw_Border(surface)
        self._draw_timer(surface)
        self._draw_money(surface)
        self._draw_enemy_money(surface)
        self._draw_reputation(surface)
        self._draw_enemy_reputation(surface)

    # ---------------- Métodos privados (segmentados) ----------------
    def _reset_timer(self) -> None:
        self._timer = CountdownTimer(settings.TIMER_START_SECONDS)

    def _update_timer(self, dt: float) -> None:
        self._timer.tick(dt)
    
    def check_time_finished(self) -> bool:
        return self._timer.finished()

    def _draw_timer(self, surface: pygame.Surface) -> None:
        label = format_mmss(self._timer.time_left)
        text_surf = self._font.render(label, True, settings.TIMER_TEXT)
        rect = self._place_rect(surface, text_surf.get_rect())
        surface.blit(text_surf, rect)
    
    def _reset_money(self) -> None:
        self._money = 0.0
    
    def _reset_moneyEnemy(self) -> None:
        self._moneyEnemy = 0.0
    
    def _update_money(self, amount: float) -> None:
        self._money = amount

    def _update_moneyEnemy(self, amount: float) -> None:
        self._moneyEnemy = amount    
    
    def _draw_money(self, surface: pygame.Surface) -> None:
        
        label = f'Dinero: ${self._money:,.0f} / ${self._meta_ingresos:,.0f}'
        fg = (16, 110, 16)
        pos = (10, 30 + 25) # margen sup-izq
        self._draw_text_with_outline(surface, label, fg, pos)

    def _draw_enemy_money(self, surface: pygame.Surface) -> None:
        label = f'{self._moneyEnemy:,.0f} / {self._meta_ingresosEnemy:,.0f} :Dinero'

        # Mismo color que el jugador
        fg = (16, 110, 16)  
        y = 30 + 25

        self._draw_text_with_outline_right(surface, label, fg, y)

    def _reset_reputation(self) -> None:
        self._reputation = 70
        
    def _reset_reputationEnemy(self) -> None:
        self._reputationEnemy = 70    
    
    def _update_reputation(self, amount: int) -> None:
        self._reputation = amount
    
    def _update_reputationEnemy(self, amount: int) -> None:
        self._reputationEnemy = amount
    
    def _draw_reputation(self, surface: pygame.Surface) -> None:
        label = f'Reputacion: {self._reputation}'
        fg = (255, 255, 255)
        # Calcular Y usando la altura de la fuente (+20px de padding)
        line_h = self._font.get_height() + 55
        pos = (10, line_h )
        text_surf = self._font_stats.render(label, True, fg)
        surface.blit(text_surf, pos)

    
    def _draw_enemy_reputation(self, surface: pygame.Surface) -> None:
        label = f'{self._reputationEnemy} :Reputacion'
        fg = (255, 255, 255)
        line_h = self._font.get_height() + 55
        self._draw_text_right(surface, label, fg, line_h)

    def _load_Border(self):
        assets_dir = os.path.join(os.path.dirname(__file__), "..", "..", "assets", "images")
        original = pygame.image.load(os.path.join(assets_dir, "border.png")).convert_alpha()

        new_width =600
        new_height = 600

        return pygame.transform.smoothscale(original, (new_width, new_height))
        

    def _draw_Border(self, surface):
        surface.blit(self.border_image, (0, 0))

    def _draw_text_right(self, surface: pygame.Surface, text: str, color: tuple, y: int):
        text_surf = self._font_stats.render(text, True, color)
        x = surface.get_width() - text_surf.get_width() - 12
        surface.blit(text_surf, (x, y))

    def _draw_text_with_outline_right(self, surface: pygame.Surface, text: str, color: tuple, y: int):
        text_surf = self._font_stats.render(text, True, color)
        x = surface.get_width() - text_surf.get_width() - 12
        self._draw_text_with_outline(surface, text, color, (x, y))

    # ---------------- Utilidades de posicionamiento ----------------
    def _place_rect(self, surface: pygame.Surface, rect: pygame.Rect) -> pygame.Rect:
        """Calcula un rect según la alineación configurada y márgenes."""
        sw, sh = surface.get_width(), surface.get_height()

        # Eje X
        if self._align_x == "left":
            rect.left = self._margin_x
        elif self._align_x == "center":
            rect.centerx = sw // 2
        else:  # "right"
            rect.right = sw - self._margin_x

        # Eje Y
        if self._align_y == "top":
            rect.top = self._margin_y
        elif self._align_y == "center":
            rect.centery = sh // 2
        else:  # "bottom"
            rect.bottom = sh - self._margin_y

        return rect
    
    def _draw_text_with_outline(self, surface: pygame.Surface, text: str, fg: tuple[int,int,int], pos: tuple[int,int], outline=(0,0,0), thickness: int = 2) -> None:
        """Render simple de contorno: dibuja el texto en negro alrededor, luego el relleno."""
        txt_main = self._font_stats.render(text, True, fg)
        txt_out  = self._font_stats.render(text, True, outline)

        x, y = pos
        # offsets en cruz+diagonales para un contorno visible
        for ox in (-thickness, 0, thickness):
            for oy in (-thickness, 0, thickness):
                if ox == 0 and oy == 0: 
                    continue
                surface.blit(txt_out, (x + ox, y + oy))
        surface.blit(txt_main, (x, y))
    def _draw_text_with_outline_v2(
        self, 
        surface: pygame.Surface, 
        text: str, 
        fg: tuple[int,int,int], 
        pos: tuple[int,int], 
        outline=(0,0,0), 
        thickness: int = 2, 
        line_spacing: int = 5
    ) -> None:
        """Render simple de contorno con soporte para múltiples líneas."""
        x, y = pos
        lines = text.splitlines()  # divide el texto en líneas si hay '\n'

        for i, line in enumerate(lines):
            txt_main = self._font_stats.render(line, True, fg)
            txt_out  = self._font_stats.render(line, True, outline)

            line_y = y + i * (txt_main.get_height() + line_spacing)

            # contorno en cruz + diagonales
            for ox in (-thickness, 0, thickness):
                for oy in (-thickness, 0, thickness):
                    if ox == 0 and oy == 0:
                        continue
                    surface.blit(txt_out, (x + ox, line_y + oy))

            # texto principal
            surface.blit(txt_main, (x, line_y))
    
    def set_time_left(self, seconds: float) -> None:
        """
        Set the remaining time on the countdown (in seconds).
        Clamps to >= 0. If the timer tracks an initial duration, also clamps to that.
        """
        s = float(seconds)
        # Clamp lower bound
        s = 0.0 if s < 0.0 else s
        # Optional upper bound if the class has a known start/total duration
        if hasattr(self, "_start_seconds"):
            s = min(s, float(self._start_seconds))
        elif hasattr(self, "total_seconds"):
            s = min(s, float(self.total_seconds))

        # Assign and update finished state if the class uses it
        setattr(self, "time_left", s)
        if hasattr(self, "_finished"):
            self._finished = (s <= 0.0)

    # ---------------- Extras públicos útiles ----------------
    def finished(self) -> bool:
        return self._timer.finished()

    @property
    def time_left(self) -> float:
        return self._timer.time_left
    
    def save_state(self) -> dict:
        """
        Serializa SOLO el estado mutable necesario para reanudar:
        - tiempo restante del temporizador
        - dinero y reputación actuales
        No guarda align/margins/fonts ni otros valores del ctor.
        """
        return {
            "timer": {
                "time_left": float(self._timer.time_left),
            },
            "stats": {
                "money": float(self._money),
                "reputation": int(self._reputation),
            },
        }

    def load_state(self, state: dict) -> bool:
        """
        Restaura el estado desde un dict. No lanza excepciones; devuelve False si falla.
        """
        try:
            if not isinstance(state, dict):
                return False

            timer = state.get("timer", {})
            stats = state.get("stats", {})

            # 1) Temporizador
            if "time_left" in timer:
                self._timer = CountdownTimer(float(timer["time_left"]))

            # 2) Dinero y reputación
            if "money" in stats:
                self._update_money(float(stats["money"]))
            if "reputation" in stats:
                self._update_reputation(int(stats["reputation"]))

            return True
        except Exception:
            return False

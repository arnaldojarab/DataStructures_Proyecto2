import os
import pygame
from .. import settings
from .button import Button
from typing import Optional, Callable

class MainMenu:
    def __init__(self, screen_size, on_load: Optional[Callable[[str], bool]] = None):
        self.w, self.h = screen_size
        self.font = pygame.font.Font(settings.UI_FONT_NAME, settings.UI_FONT_SIZE)
        self.on_load = on_load
        self.difficulty = "Hard"  # dificultad seleccionada ("Easy", "Medium", "Hard")

        base_dir = os.path.dirname(os.path.abspath(__file__))
        bg_path = os.path.normpath(os.path.join(base_dir, "..", "..", "assets", "images", "menu_bg.png"))
        self.bg_surf = pygame.image.load(bg_path).convert()
        self.bg_surf = pygame.transform.smoothscale(self.bg_surf, (self.w, self.h))

        # Desplazamiento horizontal para icono y botones
        self.offset_x = -150

        # phases: MAIN (botones principales) | LOAD (lista de partidas) | DIFFICULTY (selección de dificultad)
        self.phase = "MAIN"

        # icono
        self.icon_surf = self._load_circular_icon(size=224)
        self.icon_rect = pygame.Rect(0, 0, 0, 0)

        # placeholders
        self.panel_rect = pygame.Rect(0, 0, self.w, self.h)
        self.btn_start: Button | None = None
        self.btn_load: Button | None = None

        # botones de dificultad
        self.btn_easy: Button | None = None
        self.btn_medium: Button | None = None
        self.btn_hard: Button | None = None

        # UI para LOAD
        self.load_title = "Cargar partida"
        self.save_buttons: list[Button] = []
        self.load_feedback: Optional[str] = None  # para mostrar errores (opcional)

        self._layout((self.w, self.h))

    def _load_circular_icon(self, size: int) -> pygame.Surface | None:
        """Load the icon and make it circular with per-pixel alpha."""
        try:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            icon_path = os.path.normpath(os.path.join(base_dir, "..", "..", "assets", "images", "kimby_icon_2.png"))

            img = pygame.image.load(icon_path).convert_alpha()
            min_side = min(img.get_width(), img.get_height())
            img = img.subsurface(pygame.Rect(
                (img.get_width() - min_side)//2,
                (img.get_height() - min_side)//2,
                min_side, min_side
            )).copy()
            img = pygame.transform.smoothscale(img, (size, size))

            mask = pygame.Surface((size, size), pygame.SRCALPHA)
            pygame.draw.circle(mask, (255, 255, 255, 255), (size//2, size//2), size//2)

            circ = pygame.Surface((size, size), pygame.SRCALPHA)
            circ.blit(img, (0, 0))
            circ.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
            return circ
        except Exception:
            return None

    def _layout(self, screen_size):
        """Recalcula panel y posiciones según el tamaño actual."""
        self.w, self.h = screen_size
        self.panel_rect = pygame.Rect(0, 0, self.w, self.h)

        # --- MAIN buttons ---
        btn_w, btn_h = 248, 54
        gap_y = 16                   # separación entre botones
        icon_gap = 48                # separación entre icono y primer botón

        btn_x = self.w // 2 - btn_w // 2 + self.offset_x

        has_icon = self.icon_surf is not None
        icon_h = self.icon_surf.get_height() if has_icon else 0

        # alto de los dos botones principales + su separación
        buttons_block_h = btn_h + gap_y + btn_h

        # alto total del bloque (si hay icono, se suma icono + icon_gap)
        block_h = (icon_h + icon_gap + buttons_block_h) if has_icon else buttons_block_h

        # y inicial para centrar verticalmente todo el bloque
        y0 = max(0, self.h // 2 - block_h // 2)

        # posiciones verticales
        if has_icon:
            ix = self.w // 2 - self.icon_surf.get_width() // 2 + self.offset_x
            iy = y0
            self.icon_rect = pygame.Rect(ix, iy, self.icon_surf.get_width(), self.icon_surf.get_height())
            btn_y = iy + icon_h + icon_gap
        else:
            self.icon_rect = pygame.Rect(0, 0, 0, 0)
            btn_y = y0

        load_y = btn_y + btn_h + gap_y

        # Botón "Iniciar nueva partida"
        self.btn_start = Button(
            rect=pygame.Rect(btn_x, btn_y, btn_w, btn_h),
            text="Iniciar nueva partida",
            font=self.font,
            bg=settings.MENU_BG,
            bg_hover=settings.MENU_BG_HOVER,
            fg=settings.TEXT_LIGHT,
        )

        # Botón "Cargar partida"
        self.btn_load = Button(
            rect=pygame.Rect(btn_x, load_y, btn_w, btn_h),
            text="Cargar partida",
            font=self.font,
            bg=settings.MENU_BG,
            bg_hover=settings.MENU_BG_HOVER,
            fg=settings.TEXT_LIGHT,
        )

        # --- DIFFICULTY buttons ---
        diff_btn_w, diff_btn_h = 248, 54
        diff_gap_y = 16
        diff_x = self.w // 2 - diff_btn_w // 2

        # alto total de los 3 botones de dificultad
        diff_block_h = diff_btn_h * 3 + diff_gap_y * 2
        diff_y0 = self.h // 2 - diff_block_h // 2

        easy_y = diff_y0
        medium_y = easy_y + diff_btn_h + diff_gap_y
        hard_y = medium_y + diff_btn_h + diff_gap_y

        # Botones de dificultad
        self.btn_easy = Button(
            rect=pygame.Rect(diff_x, easy_y, diff_btn_w, diff_btn_h),
            text="Fácil",
            font=self.font,
            bg=settings.BTN_BG_HOVER,
            bg_hover=settings.MENU_BG_HOVER,
            fg=settings.MENU_BG,
        )
        self.btn_medium = Button(
            rect=pygame.Rect(diff_x, medium_y, diff_btn_w, diff_btn_h),
            text="Medio",
            font=self.font,
            bg=settings.BTN_BG_HOVER,
            bg_hover=settings.MENU_BG_HOVER,
            fg=settings.MENU_BG,
        )
        self.btn_hard = Button(
            rect=pygame.Rect(diff_x, hard_y, diff_btn_w, diff_btn_h),
            text="Difícil",
            font=self.font,
            bg=settings.BTN_BG_HOVER,
            bg_hover=settings.MENU_BG_HOVER,
            fg=settings.MENU_BG,
        )

        # Si estamos en fase LOAD, reconstruir la lista de partidas
        if self.phase == "LOAD":
            self._build_save_list()

    def resize(self, screen_size):
        self._layout(screen_size)

    def draw(self, surface: pygame.Surface):
        # Adaptar layout si el tamaño del surface cambió
        current_size = surface.get_size()
        if current_size != (self.w, self.h):
            self._layout(current_size)
            self.bg_surf = pygame.transform.scale(self.bg_surf, current_size)

        # Fondo según la fase
        if self.phase in ("MAIN", "LOAD"):
            # Fondo con imagen para menú principal y carga
            surface.blit(self.bg_surf, (0, 0))
        elif self.phase == "DIFFICULTY":
            # Fondo plano para menú de dificultad
            surface.fill(settings.MENU_BG)

        # Dibujar según fase
        if self.phase == "MAIN":
            if self.icon_surf and self.icon_rect.width > 0:
                surface.blit(self.icon_surf, (self.icon_rect.x, self.icon_rect.y))
            self._draw_main(surface)
        elif self.phase == "LOAD":
            self._draw_load(surface)
        elif self.phase == "DIFFICULTY":
            self._draw_difficulty(surface)

    def _draw_main(self, surface: pygame.Surface):
        self.btn_start.draw(surface)
        self.btn_load.draw(surface)

    def _draw_load(self, surface: pygame.Surface):
        # Título
        title = self.load_title
        title_surf = self.font.render(title, True, settings.TEXT_LIGHT)
        title_x = self.w // 2 - title_surf.get_width() // 2
        title_y = int(self.h * 0.18)

        draw_text_outline(
            surface,
            title,
            self.font,
            (title_x, title_y),
            color_fg=settings.TEXT_LIGHT,
            color_outline=settings.MENU_BG,
            outline_width=2
        )

        # Mensaje de feedback si hay error al cargar
        if self.load_feedback:
            fb = self.font.render(self.load_feedback, True, settings.TEXT_RED)
            fb_x = self.w // 2 - fb.get_width() // 2
            fb_y = self.h // 2 - fb.get_height() // 2
            fb_rect = fb.get_rect(topleft=(fb_x, fb_y)).inflate(16, 10)
            pygame.draw.rect(surface, settings.MENU_BG, fb_rect, border_radius=6)
            surface.blit(fb, (fb_x, fb_y))
        else:
            for b in self.save_buttons:
                b.draw(surface)

        hint = "ESC para volver"
        hint_surf = pygame.font.Font(settings.UI_FONT_NAME, 18).render(hint, True, settings.BUTTON_BG)
        surface.blit(hint_surf, (self.w // 2 - hint_surf.get_width() // 2, int(self.h * 0.88)))

    def _draw_difficulty(self, surface: pygame.Surface):
        """Dibuja el submenú de selección de dificultad."""
        # Título centrado
        title = "Seleccionar dificultad"
        title_surf = self.font.render(title, True, settings.TEXT_LIGHT)
        title_x = self.w // 2 - title_surf.get_width() // 2
        title_y = int(self.h * 0.18)

        draw_text_outline(
            surface,
            title,
            self.font,
            (title_x, title_y),
            color_fg=settings.TEXT_LIGHT,
            color_outline=settings.MENU_BG_HOVER,
            outline_width=2
        )

        # Dibujar botones de dificultad
        if self.btn_easy:
            self.btn_easy.draw(surface)
        if self.btn_medium:
            self.btn_medium.draw(surface)
        if self.btn_hard:
            self.btn_hard.draw(surface)

        # Pista para volver
        hint = "ESC para volver"
        hint_surf = pygame.font.Font(settings.UI_FONT_NAME, 18).render(hint, True, settings.TEXT_LIGHT)
        surface.blit(hint_surf, (self.w // 2 - hint_surf.get_width() // 2, int(self.h * 0.88)))

    def handle_event(self, event) -> str | None:
        # Fase principal
        if self.phase == "MAIN":
            if self.btn_start and self.btn_start.handle_event(event):
                self.phase = "DIFFICULTY"
                self._layout((self.w, self.h))
                return None

            if self.btn_load and self.btn_load.handle_event(event):
                self.phase = "LOAD"
                self._layout((self.w, self.h))
                return None

            return None

        # Fase de selección de dificultad
        if self.phase == "DIFFICULTY":
            # Volver al menú principal con ESC
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.phase = "MAIN"
                return None

            # Selección de dificultad con clic
            if self.btn_easy and self.btn_easy.handle_event(event):
                self.set_difficulty("Easy")
                return "start"

            if self.btn_medium and self.btn_medium.handle_event(event):
                self.set_difficulty("Medium")
                return "start"

            if self.btn_hard and self.btn_hard.handle_event(event):
                self.set_difficulty("Hard")
                return "start"

            return None

        # Fase de carga de partidas (LOAD)
        if self.phase == "LOAD":
            # Volver con ESC
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.phase = "MAIN"
                return None

            # Click sobre un save
            for b in self.save_buttons:
                if b.handle_event(event):
                    filename = b.text  # viene con .sav
                    ok = False
                    if self.on_load:
                        ok = bool(self.on_load(filename))
                    if ok:
                        return "loaded"   # el engine pone GameState.PLAYING
                    else:
                        # muestra feedback, no cambia de pantalla
                        self.load_feedback = "No se pudo cargar la partida."
                        return None
            return None

        return None

    def reset_menu(self):
        """Resetea el menú al estado principal."""
        self.phase = "MAIN"

    def on_load_game(self):
        """Placeholder for Load Game action (no-op for now)."""
        pass

    def _build_save_list(self):
        """Crea botones para cada .sav en /saves, centrados en vertical."""
        self.save_buttons.clear()
        self.load_feedback = None
        base_dir = os.path.dirname(os.path.abspath(__file__))
        saves_dir = os.path.join(base_dir, "..", "..", "..", "saves")

        try:
            entries = []
            if os.path.isdir(saves_dir):
                for name in os.listdir(saves_dir):
                    if name.lower().endswith(".sav"):
                        entries.append(name)
            entries.sort(key=str.lower)
        except Exception:
            entries = []

        btn_w, btn_h = 320, 48
        total_h = len(entries) * btn_h + max(0, (len(entries) - 1)) * 10
        y0 = self.h // 2 - total_h // 2
        x = self.w // 2 - btn_w // 2

        for idx, fname in enumerate(entries):
            y = y0 + idx * (btn_h + 10)
            self.save_buttons.append(
                Button(
                    rect=pygame.Rect(x, y, btn_w, btn_h),
                    text=fname,
                    font=self.font,
                    bg=settings.MENU_BG,
                    bg_hover=settings.MENU_BG_HOVER,
                    fg=settings.TEXT_LIGHT
                )
            )

        # Si no hay saves, deja un feedback mínimo
        if not self.save_buttons:
            self.load_feedback = "No se encontraron partidas guardadas."

    def get_difficulty(self) -> str:
        return self.difficulty

    def set_difficulty(self, difficulty: str):
        self.difficulty = difficulty


# --- helper: outlined text ---
def draw_text_outline(surface, text, font, pos, color_fg, color_outline, outline_width=2):
    """Dibuja texto con contorno 'stroke' usando múltiples blits alrededor."""
    x, y = pos
    base = font.render(text, True, color_fg)
    outline_surf = font.render(text, True, color_outline)
    offsets = []
    r = outline_width
    for dx in (-r, 0, r):
        for dy in (-r, 0, r):
            if dx == 0 and dy == 0:
                continue
            offsets.append((dx, dy))

        # blits del contorno
    for dx, dy in offsets:
        surface.blit(outline_surf, (x + dx, y + dy))
    surface.blit(base, (x, y))

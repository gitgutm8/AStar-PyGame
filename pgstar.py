import sys
import time
from functools import partialmethod
from contextlib import suppress

import pygame as pg

import astar
from astar import Vector


_LCLICK, _RCLICK = 1, 3
_ENTER = 13

FPS = 50
BLOCK_SIZE = 30
SQUARE = (BLOCK_SIZE, BLOCK_SIZE)

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (155, 155, 155)

_EMPTY = 0
_BLOCKED = 1
_START = 2
_GOAL = 4
_MARKED = 8


class GridPath:

    def __init__(self, cols, lines):
        self.cols = cols
        self.lines = lines
        self.graph = [[_EMPTY] * cols for _ in range(lines)]
        self.last_duration = '...'
        # in this case the heuristic function is the same as the weights function (distance)
        self.astar = astar.AStar(self._adj, self._weights, self._weights)

    def solve(self):
        source, goal = self.init_solve()
        start = time.time()
        path = self.astar.get_shortest(Vector(source), Vector(goal))
        self.last_duration = time.time() - start
        for x, y in path[:-1]:
            self[y][x] = _MARKED

    def init_solve(self):
        start = goal = None
        for y, line in enumerate(self):
            for x, tile in enumerate(line):
                if tile == _MARKED:
                    self[y][x] = _EMPTY

                elif tile == _START and start is None:
                    start = x, y
                elif tile == _GOAL and goal is None:
                    goal = x, y

                elif tile not in {_BLOCKED, _EMPTY}:
                    # found a second start/goal
                    raise ValueError('needs exactly one start and goal')

        if start is None or goal is None:
            raise ValueError('needs exactly one start and goal')
        return start, goal

    def change_tile(self, pos, state):
        self.astar.invalidate()
        x, y = pos
        self[y][x] ^= state  # Add/remove state
        self[y][x] &= state  # Remove all other states

    def clear(self):
        self.graph = [[_EMPTY] * self.cols for _ in range(self.lines)]

    def _adj(self, node):
        neighbours = [
            (-1, -1), (-1, 0), (-1, 1),
            ( 0, -1),          ( 0, 1),
            ( 1, -1), ( 1, 0), ( 1, 1)
        ]
        x, y = node
        for dy, dx in neighbours:
            x2, y2 = x+dx, y+dy
            if (x2, y2) in self and self[y2][x2] != _BLOCKED:
                yield Vector((x2, y2))

    @staticmethod
    def _weights(node, adj):
        return (node-adj).length()

    def __contains__(self, pos):
        x, y = pos
        return 0 <= x < self.cols and 0 <= y < self.lines

    def __getitem__(self, key):
        return self.graph[key]

    def __iter__(self):
        return iter(self.graph)


class PathMaker:

    state_to_color = {
        _EMPTY: BLACK,
        _BLOCKED: GREY,
        _START: (0, 255, 0),
        _GOAL: (255, 0, 0),
        _MARKED: (0, 0, 255)
    }

    def __init__(self, cols, lines):
        self.cols, self.lines = cols, lines
        self.selected_state = _BLOCKED

        pg.display.set_caption('A*')
        self.block_size = BLOCK_SIZE
        self.world = pg.display.set_mode(Vector((cols, lines)) * self.block_size)
        self.screen = pg.display.get_surface()
        self.clock = pg.time.Clock()
        self.font = pg.font.Font('freesansbold.ttf', 20)

        self.padding_bottom = 1
        self.padding_left = 1
        grid_lower = lines-self.padding_bottom
        self.grid = GridPath(cols-self.padding_left, grid_lower)

        self.bottom_side_bar = pg.Rect(Vector((self.padding_left, grid_lower)) * self.block_size,
                                       (cols * self.block_size, self.padding_bottom * self.block_size))

        self.left_side_bar = pg.Rect(Vector((0, 0)), (self.padding_left * self.block_size,
                                                      lines * self.block_size))

    def solve(self):
        with suppress(ValueError):
            self.grid.solve()

    def run(self):
        callbacks = {
            pg.MOUSEBUTTONDOWN: self.handle_mouse_input,
            pg.KEYDOWN: self.handle_key_input,
            pg.QUIT: sys.exit
        }
        while True:
            self.clock.tick(FPS)
            for e in pg.event.get():
                if e.type in callbacks:
                    callbacks[e.type](e)
            self.draw()

    def handle_mouse_input(self, event):
        scaled_pos = Vector(event.pos) // self.block_size + Vector((-self.padding_left, 0))
        if event.button == _LCLICK:
            if scaled_pos in self.grid:
                self.grid.change_tile(scaled_pos, self.selected_state)

    def handle_key_input(self, event):
        {
            pg.K_1: self.set_blocked,
            pg.K_2: self.set_start,
            pg.K_3: self.set_goal,
            pg.K_ESCAPE: self.grid.clear,
            _ENTER: self.solve
        }.get(event.key, lambda: 0)()

    def draw(self):
        for y, line in enumerate(self.grid):
            for x, state in enumerate(line, self.padding_left):
                rect = Vector((x, y)) * self.block_size, SQUARE
                color = self.state_to_color[state]
                pg.draw.rect(self.screen, color, rect)
        self.draw_side_bars()
        self.draw_grid_lines()
        pg.display.update()

    def draw_side_bars(self):
        self.screen.fill(BLACK, self.bottom_side_bar)
        pos = 50, (self.lines-0.75) * self.block_size
        self.draw_text(self.font, f'Solved in: {self.grid.last_duration} seconds', pos, WHITE)

    def draw_grid_lines(self):
        for i in range(self.padding_left, self.cols):
            start = Vector((i, 0)) * self.block_size
            end = Vector((i, self.lines-self.padding_bottom)) * self.block_size
            pg.draw.line(self.screen, WHITE, start, end)
        for i in range(self.lines):
            start = Vector((self.padding_left, i)) * self.block_size
            end = Vector((self.cols, i)) * self.block_size
            pg.draw.line(self.screen, WHITE, start, end)

    def draw_text(self, font, text, pos, color=BLACK, *modifiers):
        """
        Writes text to an arbitrary position on the screen.

        :return: {None}
        """
        text = font.render(text, 1, color)
        for modifier in modifiers:
            text, pos = modifier(text, pos)
        self.screen.blit(text, pos)

    def set_state(self, state):
        self.selected_state = state

    set_blocked = partialmethod(set_state, _BLOCKED)
    set_start = partialmethod(set_state, _START)
    set_goal = partialmethod(set_state, _GOAL)


if __name__ == '__main__':
    pg.init()
    PathMaker(20, 20).run()

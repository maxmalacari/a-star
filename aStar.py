# A* search Algorithm - Max Malacari 20/02/2017
# Includes random obstacles in the way
# Diagonal movement included
# Can't move diagonally "between" obstacles

import pygame as pg
import sys # to handle quit event
import random as rand
from math import *

# Some options to set!
wWidth = 700 # dimensions of the drawing window
wHeight = 700
cols = 100 # number of cells in each dimension
rows = 100
start_i = 0 # start and end cell coordinates (will be guaranteed not a wall)
start_j = 0
end_i = cols
end_j = rows
wallFraction = 0.05 # fraction of cells that contain an obstacle (0.3 nominal)
showBoundaries = False # show bounding boxes for cells

# Dark theme color palette
BG_COLOR         = (15, 15, 25)      # near-black blue-tinted background
WALL_COLOR       = (45, 45, 60)      # dark blue-grey walls
START_COLOR      = (0, 240, 120)     # vivid green
END_COLOR        = (240, 60, 120)    # vivid pink-red
CURRENT_COLOR    = (255, 255, 100)   # bright yellow (active cell)
BOUNDARY_COLOR   = (40, 40, 55)      # subtle grid lines
HEAT_LOW         = (30, 80, 220)     # blue  (low f-score = promising)
HEAT_MID         = (220, 180, 20)    # yellow (mid f-score)
HEAT_HIGH        = (220, 40, 40)     # red   (high f-score = expensive)
CLOSED_COLOR     = (70, 40, 90)      # muted purple (evaluated cells)
PATH_START_COLOR = (0, 220, 220)     # cyan (start of path)
PATH_END_COLOR   = (200, 0, 220)     # magenta (end/current of path)
OVERLAY_TEXT     = (200, 200, 220)   # stats text color

w = wWidth // cols
h = wHeight // rows

pg.init()
pg.font.init()
screen = pg.display.set_mode((wWidth, wHeight))
pg.display.set_caption("A* Pathfinding Visualizer")
STATS_FONT = pg.font.SysFont("monospace", 13)
clock = pg.time.Clock()
FPS = 60

# Class to hold the properties of a cell
class Cell():
    def __init__(self, i, j):
        self.i = i
        self.j = j
        self.f = 0
        self.g = 0
        self.h = 0
        self.neighbours = []
        self.previous = 0
        self.isWall = False

        if rand.random() < wallFraction:
            self.isWall = True
        if i == start_i and j == start_j:
            self.isWall = False
        if i == end_i - 1 and j == end_j - 1:
            self.isWall = False

    def show(self, colour):
        pg.draw.rect(screen, colour, (self.i*w, self.j*h, w, h), 0)

    def showCellBoundary(self):
        pg.draw.rect(screen, BOUNDARY_COLOR, (self.i*w, self.j*h, w, h), 1)

    def addNeighbours(self, grid):
        if self.isWall == False: # only add neighbours to non-wall cells
            i = self.i
            j = self.j
            # top/bottom/left/right
            if i < cols-1 and grid[i+1][j].isWall == False:
                self.neighbours.append(grid[i+1][j])
            if j < rows-1 and grid[i][j+1].isWall == False:
                self.neighbours.append(grid[i][j+1])
            if i > 0 and grid[i-1][j].isWall == False:
                self.neighbours.append(grid[i-1][j])
            if j > 0 and grid[i][j-1].isWall == False:
                self.neighbours.append(grid[i][j-1])
            # diagonals, also check blocks adjacent to diagonals are not walls
            if i > 0 and j > 0 and grid[i-1][j-1].isWall == False and grid[i][j-1].isWall == False and grid[i-1][j].isWall == False:
                self.neighbours.append(grid[i-1][j-1])
            if i < cols-1 and j > 0 and grid[i+1][j-1].isWall == False and grid[i][j-1].isWall == False and grid[i+1][j].isWall == False:
                self.neighbours.append(grid[i+1][j-1])
            if i > 0 and j < rows-1 and grid[i-1][j+1].isWall == False and grid[i-1][j].isWall == False and grid[i][j+1].isWall == False:
                self.neighbours.append(grid[i-1][j+1])
            if i < cols-1 and j < rows-1 and grid[i+1][j+1].isWall == False and grid[i+1][j].isWall == False and grid[i][j+1].isWall == False:
                self.neighbours.append(grid[i+1][j+1])

def main():

    grid = []
    setup(grid, cols, rows)

    start = grid[start_i][start_j]
    end = grid[end_i-1][end_j-1]

    openSet = []
    closedSet = []
    wallSet = []
    for i in range(0,cols):
        for j in range(0,rows):
            if grid[i][j].isWall:
                wallSet.append(grid[i][j])

    openSet.append(start)
    finished = False
    iteration = 0
    current = None

    while finished == False:
        iteration += 1

        # Event handling (allows quitting during algorithm)
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                sys.exit()

        path = []

        # Fill background each frame to prevent rendering artifacts
        screen.fill(BG_COLOR)

        # Show wall cells
        for cell in wallSet:
            cell.show(WALL_COLOR)
        # Show cells in the closed set
        for cell in closedSet:
            cell.show(CLOSED_COLOR)

        # Algorithm
        if len(openSet) > 0:
            lowestCostIndex = 0
            current = openSet[lowestCostIndex]

            for i in range(0,len(openSet)):
                if openSet[i].f < openSet[lowestCostIndex].f:
                    lowestCostIndex = i
                    current = openSet[lowestCostIndex]
            if current == end:
                finished = True
                path = calculatePath(current) # get optimal path
                print("Solution found!")

            else:
                closedSet.append(current)
                openSet.remove(current)

                neighbours = current.neighbours
                for i in range(0,len(neighbours)):
                    neighbour = neighbours[i]
                    neighbour.h = heuristic(neighbour, end)
                    if neighbour in closedSet:
                        continue
                    temp_g = current.g + sqrt((neighbour.i - current.i)**2 + (neighbour.j - current.j)**2) # movement cost to get to neighbour
                    if neighbour in openSet: # is neighbour already in the open set?
                        if temp_g < neighbour.g: # did we get there more efficiently?
                            neighbour.g = temp_g
                            neighbour.f = neighbour.h + neighbour.g
                            neighbour.previous = current
                    else:
                        neighbour.g = temp_g
                        neighbour.f = neighbour.h + neighbour.g
                        neighbour.previous = current
                        openSet.append(neighbour)

                    path = calculatePath(current) # get the current path to this cell

        else:
            finished = True # to quit the loop
            print("No solution!")

        # Show open set with heat-map coloring + current cell highlight
        draw_open_set(openSet, current)

        # Show the current path with gradient coloring
        draw_gradient_path(path, w, h)

        # Show bounding boxes
        if showBoundaries == True:
            for i in range(0,cols):
                for j in range(0,rows):
                    grid[i][j].showCellBoundary()

        # Show end points
        start.show(START_COLOR)
        end.show(END_COLOR)

        # Stats overlay (drawn last, on top of everything)
        draw_stats_overlay(screen, iteration, openSet, closedSet, path, finished)

        # Update the display
        pg.display.flip()
        clock.tick(FPS)


    # Stop from instantly closing on finish
    while(True):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                sys.exit()
        clock.tick(30)

# Set up the grid of cell objects
def setup(grid, cols, rows):
    for i in range(0,cols):
        grid.append([])
        for j in range(0,rows):
            grid[i].append(Cell(i,j))

    # add neighbours once grid is initialized
    for i in range(0,cols):
        grid.append([])
        for j in range(0,rows):
            grid[i][j].addNeighbours(grid)

def heuristic(cell1, cell2):
    #h = abs(cell1.i - cell2.i) + abs(cell1.j - cell2.j)
    h = sqrt((cell1.i-cell2.i)**2 + (cell1.j-cell2.j)**2)
    return h

def calculatePath(current): # get current optimal path up to current cell
    path = []
    temp = current
    path.append(temp)
    while temp.previous:
        path.append(temp.previous)
        temp = temp.previous
    return path

def lerp_color(color_a, color_b, t):
    """Linearly interpolate between two RGB colors. t in [0.0, 1.0]."""
    return (
        int(color_a[0] + (color_b[0] - color_a[0]) * t),
        int(color_a[1] + (color_b[1] - color_a[1]) * t),
        int(color_a[2] + (color_b[2] - color_a[2]) * t),
    )

def heat_color(t):
    """
    Map normalized score t in [0.0, 1.0] to a heat color.
    0.0 (low f, promising) -> HEAT_LOW (blue)
    0.5 (mid f)            -> HEAT_MID (yellow)
    1.0 (high f, expensive) -> HEAT_HIGH (red)
    """
    if t <= 0.5:
        return lerp_color(HEAT_LOW, HEAT_MID, t * 2.0)
    else:
        return lerp_color(HEAT_MID, HEAT_HIGH, (t - 0.5) * 2.0)

def draw_open_set(openSet, current):
    """Draw open set cells with heat-map coloring; highlight current cell on top."""
    if not openSet:
        return

    f_values = [cell.f for cell in openSet]
    min_f = min(f_values)
    max_f = max(f_values)
    f_range = max_f - min_f if max_f != min_f else 1.0

    for cell in openSet:
        if cell is current:
            continue  # draw current last so it renders on top
        t = (cell.f - min_f) / f_range
        cell.show(heat_color(t))

    # Draw active cell with bright highlight + inner glow
    if current is not None:
        current.show(CURRENT_COLOR)
        glow_margin = max(1, w // 4)
        pg.draw.rect(
            screen,
            (255, 255, 220),
            (current.i * w + glow_margin,
             current.j * h + glow_margin,
             w - glow_margin * 2,
             h - glow_margin * 2),
            0
        )

def draw_gradient_path(path, w, h):
    """Draw the path as line segments with a cyan-to-magenta color gradient."""
    if len(path) < 2:
        return

    # calculatePath returns current→start; reverse for start→current flow
    display_path = list(reversed(path))
    n_segments = len(display_path) - 1

    for i in range(n_segments):
        t = i / n_segments
        color = lerp_color(PATH_START_COLOR, PATH_END_COLOR, t)
        p1 = (display_path[i].i * w + w // 2, display_path[i].j * h + h // 2)
        p2 = (display_path[i+1].i * w + w // 2, display_path[i+1].j * h + h // 2)
        pg.draw.line(screen, color, p1, p2, 3)

def draw_stats_overlay(screen, iteration, openSet, closedSet, path, finished):
    """Render a semi-transparent stats panel in the top-left corner."""
    lines = [
        f"Iter:   {iteration}",
        f"Open:   {len(openSet)}",
        f"Closed: {len(closedSet)}",
        f"Path:   {len(path)} cells",
        f"Done:   {'Yes' if finished else 'No'}",
    ]

    padding = 6
    line_height = STATS_FONT.get_linesize()
    box_w = 140
    box_h = len(lines) * line_height + padding * 2

    overlay_surf = pg.Surface((box_w, box_h), pg.SRCALPHA)
    overlay_surf.fill((0, 0, 0, 160))
    screen.blit(overlay_surf, (8, 8))

    for idx, line in enumerate(lines):
        text_surf = STATS_FONT.render(line, True, OVERLAY_TEXT)
        screen.blit(text_surf, (8 + padding, 8 + padding + idx * line_height))

main()

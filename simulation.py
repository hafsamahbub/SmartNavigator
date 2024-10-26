# Setting up the grid
import pygame
import sys

# Initialize Pygame
pygame.init()

# Screen settings
screen_width, screen_height = 600, 600
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Robot Pathfinding Simulation")

# Grid settings
cols, rows = 20, 20  # Grid size
cell_size = screen_width // cols

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)   # Robot
RED = (255, 0, 0)     # Goal
GRAY = (200, 200, 200) # Obstacles

# Create a grid
grid = [[0 for _ in range(cols)] for _ in range(rows)]

# Set start and goal positions
start_pos = (0, 0)
goal_pos = (rows - 1, cols - 1)
grid[start_pos[0]][start_pos[1]] = 2  # Start position
grid[goal_pos[0]][goal_pos[1]] = 3    # Goal position

# Set some obstacles
obstacles = [(5, 5), (5, 6), (6, 5), (10, 15), (10, 16), (10, 17)]
for obstacle in obstacles:
    grid[obstacle[0]][obstacle[1]] = 1  # Mark obstacles

def draw_grid():
    for row in range(rows):
        for col in range(cols):
            color = WHITE
            if grid[row][col] == 1:  # Obstacle
                color = GRAY
            elif grid[row][col] == 2:  # Start
                color = BLUE
            elif grid[row][col] == 3:  # Goal
                color = RED
            pygame.draw.rect(screen, color, (col * cell_size, row * cell_size, cell_size, cell_size))
            pygame.draw.rect(screen, BLACK, (col * cell_size, row * cell_size, cell_size, cell_size), 1)

# Main loop
running = True
while running:
    screen.fill(WHITE)
    draw_grid()
    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

pygame.quit()
sys.exit()



# A* Pathfinding Algorithm
import heapq
def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {cell: float("inf") for row in grid for cell in enumerate(row)}
    g_score[start] = 0
    f_score = {cell: float("inf") for row in grid for cell in enumerate(row)}
    f_score[start] = heuristic(start, goal)

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            return reconstruct_path(came_from, current)

        neighbors = get_neighbors(current, rows, cols)
        for neighbor in neighbors:
            if grid[neighbor[0]][neighbor[1]] == 1:  # Skip obstacles
                continue

            tentative_g_score = g_score[current] + 1
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

# Heuristic function for A* (Manhattan distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Get neighbors of a cell in the grid
def get_neighbors(pos, rows, cols):
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Up, down, left, right
        x, y = pos[0] + dx, pos[1] + dy
        if 0 <= x < rows and 0 <= y < cols:
            neighbors.append((x, y))
    return neighbors



# Draw the Path on the Grid
# Reconstruct the path from start to goal
def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


# Main loop
running = True
path = astar(grid, start_pos, goal_pos)  # Calculate the path using A*

while running:
    screen.fill(WHITE)
    draw_grid()
    
    if path:
        draw_path(path)  # Draw the path on the grid

    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False


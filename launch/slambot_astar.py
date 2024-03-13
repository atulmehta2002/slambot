import pygame
import os
import heapq
import yaml
import time
import tempfile
from PIL import Image
import matplotlib.pyplot as plt

# Function to load a PGM file
def load_pgm(pgm_file):
    pgm_image = pygame.image.load(pgm_file)
    return pgm_image

def scale_and_save_png(pgm_file, scale_factor, png_file):
  try:
    raw_pgm = Image.open(pgm_file)                                                                                               # Open the PGM image
    scaled_pgm = raw_pgm.resize((int(raw_pgm.width * scale_factor), int(raw_pgm.height * scale_factor)), Image.NEAREST)          # Scale the image using resize
    
    # png_image = Image.open(scaled_pgm).convert('L')                                                                            # Open the PGM image in grayscale mode (L)
    scaled_pgm = scaled_pgm.convert('RGB')                                                                                       # Convert to RGB for compatibility with PNG format 
    scaled_pgm.save(png_file, format='PNG')                                                                                      # Save the image as a PNG file

    print(f"PGM file saved as: {png_file}")

  except FileNotFoundError:
    print(f"Error: PGM file not found: {pgm_file}")
  except Exception as e:
    print(f"An error occurred: {e}")

def show_plots(pgm_file):                                 #-------------------------------------------Use for debugging only-------------------------------------------#

    image_1 = Image.open(pgm_file).convert('RGB')
    print(f"Displaying the Plot for original PGM file")

    plt.imshow(image_1)
    plt.title(f"PGM Image: {pgm_file}")
    plt.show()

# Function to scale down the pgm_image to a grid
def scale_down_to_grid(pgm_image, grid_size):
    scaled_image = []
    for y in range(0, pgm_image.get_height(), grid_size):
        row = []
        for x in range(0, pgm_image.get_width(), grid_size):
            total_r, total_g, total_b, total_a = 0, 0, 0, 0
            count = 0  # Count the number of pixels in this grid cell
            for dy in range(grid_size):
                for dx in range(grid_size):
                    if (x + dx) < pgm_image.get_width() and (y + dy) < pgm_image.get_height():
                        pixel = pgm_image.get_at((x + dx, y + dy))
                        total_r += pixel.r
                        total_g += pixel.g
                        total_b += pixel.b
                        total_a += pixel.a
                        count += 1
            average_r = total_r // count if count > 0 else 0
            average_g = total_g // count if count > 0 else 0
            average_b = total_b // count if count > 0 else 0
            average_a = total_a // count if count > 0 else 0
            row.append((average_r, average_g, average_b, average_a))
        scaled_image.append(row)
    return scaled_image

# A* algorithm implementation
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(graph, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while frontier:
        current_cost, current_node = heapq.heappop(frontier)

        if current_node == goal:
            break

        for next_node in graph.neighbors(current_node):
            new_cost = cost_so_far[current_node] + graph.cost(current_node, next_node)
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(goal, next_node)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current_node

    path = []
    current_node = goal
    while current_node != start:
        path.append(current_node)
        current_node = came_from[current_node]
    path.append(start)
    path.reverse()
    return path

# Graph representing the grid
class GridGraph:
    def __init__(self, grid):
        self.grid = grid

    def neighbors(self, node):
        x, y = node
        neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        valid_neighbors = []
        for nx, ny in neighbors:
            if 0 <= nx < len(self.grid[0]) and 0 <= ny < len(self.grid):
                valid_neighbors.append((nx, ny))
        return valid_neighbors

    def cost(self, from_node, to_node):
        return abs(self.grid[to_node[1]][to_node[0]][0] - self.grid[from_node[1]][from_node[0]][0])

#-----------------------------------------------------------------------------------------------------------------------------------------------------------------------#
# Initialize Pygame
pygame.init()

# Constants
GRID_SIZE = 1                                      # Adjust this to change the grid size
pgm_file = "/home/atul/ros2_ws/src/slambot/maps/mapped_area.pgm"                       # Change this to your map file
png_file = "/home/atul/slam-bot-streaming/public/images/mapped_area.png"                        # this is scaled image of pgm image
shortest_path_map = "/home/atul/slam-bot-streaming/public/images/shortest_path_map.png"         # Save the shortest path map as a PNG file
scale_factor = 5                            # Scale up to 500%

counter = 0                 # used to save map only once after second click event
save_counter = 1            # used to save number of times the mp is saved

# Load the map
map_image = load_pgm(pgm_file)

# Scale down the pgm_image to a grid
scaled_image = scale_down_to_grid(map_image, GRID_SIZE)

# Set up the display
cell_size = 5 # Adjust this for smaller squares
window_size = (len(scaled_image[0]) * cell_size, len(scaled_image) * cell_size)
screen = pygame.display.set_mode(window_size)
pygame.display.set_caption("Map Display")

# Graph representing the grid
graph = GridGraph(scaled_image)

# Variables for marking points
start_point = None
end_point = None

#-----------------------------------------------------------------------------------------------------------------------------------------------------------------------#

# Main loop
running = True
while running:
    
    with open("coordinates.yaml", "r") as file:                     # read the contents of yaml file and store then in respective variables
        safe_data = yaml.safe_load(file)
        
    start_x, start_y = safe_data["start_coordinate"]
    goal_x, goal_y = safe_data["goal_coordinate"]
    
    start_point = (start_x, start_y)
    end_point = (goal_x, goal_y)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
                
    # Draw the grid
    screen.fill((255, 255, 255))
    for y, row in enumerate(scaled_image):
        for x, color in enumerate(row):
            rect = pygame.Rect(x * cell_size, y * cell_size, cell_size, cell_size)
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, (0, 0, 0), rect, 1)

    # Draw start and end points
    if start_point:
        pygame.draw.circle(screen, (255, 0, 0), (start_point[0] * cell_size + cell_size // 2, start_point[1] * cell_size + cell_size // 2), cell_size // 4)
    if end_point:
        pygame.draw.circle(screen, (0, 255, 0), (end_point[0] * cell_size + cell_size // 2, end_point[1] * cell_size + cell_size // 2), cell_size // 4)

    # Run A* algorithm and draw the path
    if start_point and end_point:
        path = astar(graph, start_point, end_point)
        for i in range(len(path) - 1):
            pygame.draw.line(screen, (0, 0, 255), (path[i][0] * cell_size + cell_size // 2, path[i][1] * cell_size + cell_size // 2),
                             (path[i+1][0] * cell_size + cell_size // 2, path[i+1][1] * cell_size + cell_size // 2), 3)
        
        if end_point and counter >= 50:
            scale_and_save_png(pgm_file, scale_factor, png_file)                                      # save the scaled up map of pgm image to png format
            
            pygame.image.save(screen, shortest_path_map)                                              # save the shortest path map image to png format
            print(f"Length in (x, y): ({len(scaled_image[0])}, {len(scaled_image)})")
            print(f"Map saved as: {shortest_path_map}, {save_counter} times!!")
            print(f"----------------------------X------------------------------")
            counter = 0
            save_counter = save_counter + 1
            
        
        start_point = None
        end_point = None
        counter = counter + 1
        
    pygame.display.flip()

pygame.quit()



## everything ok till here :) ##

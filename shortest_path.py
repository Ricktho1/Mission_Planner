import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from queue import PriorityQueue

grid_size = 101

def generate_weighted_grid(size):
    grid = np.zeros((size, size, size), dtype=int)
    num_weighted_points = int(0.1 * size**3) 
    
    for _ in range(num_weighted_points):
        x, y, z = np.random.randint(0, size, size=3)
        grid[x, y, z] = np.random.randint(1, 11) 
    
    return grid

def astar(grid, start, end):
    def heuristic(a, b):
        return np.linalg.norm(np.array(a) - np.array(b))
    
    neighbors = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}
    
    while not open_set.empty():
        _, current = open_set.get()
        
        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        for dx, dy, dz in neighbors:
            neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
            if 0 <= neighbor[0] < grid_size and 0 <= neighbor[1] < grid_size and 0 <= neighbor[2] < grid_size:
                tentative_g_score = g_score[current] + (grid[neighbor] if grid[neighbor] > 0 else 1)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                    open_set.put((f_score[neighbor], neighbor))
    
    return None 

def schedule_paths(paths, velocity):
    time_map = {}
    scheduled_paths = []
    travel_times = []
    
    for path in paths:
        start_time = 0
        scheduled_path = []
        total_time = 0
        
        for i, point in enumerate(path):
            arrival_time = start_time + i / velocity
            if point in time_map and time_map[point] <= arrival_time:
                start_time = time_map[point] + 1 / velocity  # Delay start
                arrival_time = start_time + i / velocity
            
            time_map[point] = arrival_time
            scheduled_path.append((point, arrival_time))
            total_time = arrival_time  
        
        scheduled_paths.append(scheduled_path)
        travel_times.append(total_time)
    
    return scheduled_paths, travel_times

def plot_paths_with_weights(grid, scheduled_paths):
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111, projection='3d')
    colors = sns.color_palette("hsv", len(scheduled_paths))

    for idx, path in enumerate(scheduled_paths):
        path_points = [p[0] for p in path]
        x, y, z = zip(*path_points)

        # Extract weights for points along the path
        weights = [grid[point] for point in path_points]
        sizes = [w * 50 for w in weights]  # Scale point sizes based on weight

        # Plot path
        ax.plot(x, y, z, marker='o', color=colors[idx], label=f'Path {idx+1}')
        
        # Plot weighted points along the path
        ax.scatter(x, y, z, c=weights, s=sizes, cmap='viridis', alpha=0.6)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.show()
    fig.savefig("weighted_paths_plot.png")

def main():
    velocity = float(input("Enter the velocity of travel (m/s): "))
    num_pairs = int(input("Enter the number of start-end point pairs: "))
    points = []
    
    for i in range(num_pairs):
        start = tuple(map(int, input(f"Enter start point {i+1} (x,y,z): ").split(',')))
        end = tuple(map(int, input(f"Enter end point {i+1} (x,y,z): ").split(',')))
        points.extend([start, end])
    
    grid = generate_weighted_grid(grid_size)
    paths = []
    
    for i in range(0, len(points), 2):
        start, end = points[i], points[i+1]
        path = astar(grid, start, end)
        if path:
            paths.append(path)
        else:
            print(f"No path found for {start} -> {end}")
    
    scheduled_paths, travel_times = schedule_paths(paths, velocity)
    
    for idx, time in enumerate(travel_times):
        print(f"Total travel time for Path {idx+1}: {time:.2f} seconds")
    
    plot_paths_with_weights(grid,scheduled_paths)

if __name__ == "__main__":
    main()

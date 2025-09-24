import sys
import time
import os
from heapq import heappush, heappop
import random
import argparse

def manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

class Grid(object):
    def __init__(self, map_str):
        lines = [line for line in map_str.strip().split('\n') if line.strip()]
        r, c, sx, sy, gx, gy = map(int, lines[0].split())
        self.rows = r
        self.cols = c
        self.start = (sx, sy)
        self.goal = (gx, gy)
        self.costs = []
        for line in lines[1:1+r]:
            self.costs.append(list(map(int, line.split())))
    
    def is_valid(self, x, y):
        return 0 <= x < self.rows and 0 <= y < self.cols
    
    def get_cost(self, x, y):
        if not self.is_valid(x, y):
            return float('inf')
        c = self.costs[x][y]
        return c if c != -1 else float('inf')
    
    def is_free(self, x, y, t=0):
        return self.get_cost(x, y) != float('inf')

class DynamicGrid(Grid):
    def __init__(self, map_str):
        super(DynamicGrid, self).__init__(map_str)
        self.obst_row = 5
    
    def is_free(self, x, y, t):
        if x == self.obst_row and y == (t % self.cols):
            return False
        return super(DynamicGrid, self).is_free(x, y, t)
    
    def get_dynamic_pos(self, t=0):
        return (self.obst_row, t % self.cols)

def uniform_cost_search(grid, use_time=False):
    start_time = time.time()
    start = grid.start if not use_time else (grid.start[0], grid.start[1], 0)
    goal_pos = grid.goal[:2]
    directions = [(-1,0), (1,0), (0,-1), (0,1)]
    pq = []
    heappush(pq, (0, start))
    came_from = {}
    cost_so_far = {}
    cost_so_far[start] = 0
    nodes = 0
    visited = set()
    while pq:
        current_cost, current = heappop(pq)
        if current in visited:
            continue
        visited.add(current)
        nodes += 1
        if current[:2] == goal_pos:
            break
        x, y = current[0], current[1]
        t = current[2] if use_time else 0
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            nt = t + 1 if use_time else t
            if grid.is_free(nx, ny, nt):
                new_cost = current_cost + grid.get_cost(nx, ny)
                neighbor = (nx, ny, nt) if use_time else (nx, ny)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    heappush(pq, (new_cost, neighbor))
                    came_from[neighbor] = current
    exec_time = time.time() - start_time
    if goal_pos not in [k[:2] for k in came_from]:
        return None, float('inf'), nodes, exec_time
    path = []
    current = next((k for k in cost_so_far if k[:2] == goal_pos), None)
    while current and current != start:
        path.append(current[:2])
        current = came_from.get(current)
    if current:
        path.append(grid.start)
        path.reverse()
    total_cost = cost_so_far[next(k for k in cost_so_far if k[:2] == goal_pos)]
    return path, total_cost, nodes, exec_time

def a_star(grid, use_time=False):
    start_time = time.time()
    start = grid.start if not use_time else (grid.start[0], grid.start[1], 0)
    goal_pos = grid.goal[:2]
    directions = [(-1,0), (1,0), (0,-1), (0,1)]
    pq = []
    g_score = {start: 0}
    f_score = {start: manhattan(start[:2], goal_pos)}
    heappush(pq, (f_score[start], start))
    came_from = {}
    nodes = 0
    visited = set()
    while pq:
        _, current = heappop(pq)
        if current in visited:
            continue
        visited.add(current)
        nodes += 1
        if current[:2] == goal_pos:
            break
        x, y = current[0], current[1]
        t = current[2] if use_time else 0
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            nt = t + 1 if use_time else t
            if grid.is_free(nx, ny, nt):
                tentative_g = g_score[current] + grid.get_cost(nx, ny)
                neighbor = (nx, ny, nt) if use_time else (nx, ny)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + manhattan((nx, ny), goal_pos)
                    heappush(pq, (f_score[neighbor], neighbor))
    exec_time = time.time() - start_time
    if goal_pos not in [k[:2] for k in came_from]:
        return None, float('inf'), nodes, exec_time
    path = []
    current = next((k for k in g_score if k[:2] == goal_pos), None)
    while current and current != start:
        path.append(current[:2])
        current = came_from.get(current)
    if current:
        path.append(grid.start)
        path.reverse()
    total_cost = g_score[next(k for k in g_score if k[:2] == goal_pos)]
    return path, total_cost, nodes, exec_time

def local_search(grid, start, current_cost, goal, restarts=10):
    start_time = time.time()
    best_path = None
    best_cost = float('inf')
    total_nodes = 0
    random.seed(42)
    for _ in range(restarts):
        path = [start]
        pos = start
        local_cost = current_cost
        local_nodes = 0
        max_iterations = grid.rows * grid.cols  # Prevent infinite loops
        while pos[:2] != goal and max_iterations > 0:
            neighbors = []
            x, y = pos[0], pos[1]
            directions = [(-1,0), (1,0), (0,-1), (0,1)]
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if grid.is_free(nx, ny):
                    edge_cost = grid.get_cost(nx, ny)
                    h = manhattan((nx, ny), goal)
                    neighbors.append(((nx, ny), local_cost + edge_cost + h))
                    local_nodes += 1
            if not neighbors:
                break
            next_pos = min(neighbors, key=lambda x: x[1])[0]
            path.append(next_pos)
            local_cost += grid.get_cost(next_pos[0], next_pos[1])
            pos = next_pos
            max_iterations -= 1
        total_nodes += local_nodes
        if pos[:2] == goal and local_cost < best_cost:
            best_path = path
            best_cost = local_cost
    exec_time = time.time() - start_time
    return best_path, best_cost, total_nodes, exec_time

def simulate_delivery(grid):
    start_time = time.time()
    path, cost, nodes_plan, _ = a_star(grid, use_time=True)
    if not path:
        return None, float('inf'), nodes_plan, time.time() - start_time
    total_cost = 0
    t = 0
    current = grid.start
    full_path = [current]
    total_nodes = nodes_plan
    print("Simulating delivery...")
    for next_pos in path[1:]:
        if not grid.is_free(next_pos[0], next_pos[1], t):
            print(f"Blocked at t={t}, pos={current}")
            print("Replanning...")
            rem_path, rem_cost, rem_nodes, _ = local_search(grid, current, total_cost, grid.goal)
            if rem_path:
                full_path.extend(rem_path[1:])
                total_cost += rem_cost
                total_nodes += rem_nodes
                print("Replanned successfully")
                break
            else:
                return None, float('inf'), total_nodes, time.time() - start_time
        total_cost += grid.get_cost(next_pos[0], next_pos[1])
        current = next_pos
        full_path.append(next_pos)
        t += 1
    exec_time = time.time() - start_time
    return full_path, total_cost, total_nodes, exec_time

def manual_control(grid, visualize=False, plot=False):
    start_time = time.time()
    current = grid.start
    path = [current]
    total_cost = 0
    t = 0
    nodes = 0
    directions = {
        'w': (-1, 0),  # Up
        's': (1, 0),   # Down
        'a': (0, -1),  # Left
        'd': (0, 1)    # Right
    }
    print(f"Manual Control: Use 'w' (up), 's' (down), 'a' (left), 'd' (right), 'q' (quit). Goal: {grid.goal}")
    
    while current != grid.goal:
        if visualize:
            os.system('cls' if os.name == 'nt' else 'clear')
            visualize_grid(grid, path, t)
            time.sleep(0.5)
        if plot:
            visualize_plot(grid, path, t)
        if isinstance(grid, DynamicGrid):
            print(f"Dynamic obstacle at: {grid.get_dynamic_pos(t)}")
        
        try:
            move = input("Enter move (w/s/a/d/q): ").lower().strip()
        except EOFError:
            print("Input interrupted. Quitting.")
            return None, float('inf'), nodes, time.time() - start_time
        
        if move == 'q':
            print("Quit by user.")
            return None, float('inf'), nodes, time.time() - start_time
        if move not in directions:
            print("Invalid input. Use 'w', 's', 'a', 'd', or 'q'.")
            continue
        
        dx, dy = directions[move]
        nx, ny = current[0] + dx, current[1] + dy
        nodes += 1
        
        if not grid.is_valid(nx, ny):
            print("Move out of bounds. Try again.")
            continue
        if not grid.is_free(nx, ny, t):
            print("Move blocked by obstacle. Try again.")
            continue
        
        total_cost += grid.get_cost(nx, ny)
        current = (nx, ny)
        path.append(current)
        t += 1
    
    exec_time = time.time() - start_time
    if visualize:
        os.system('cls' if os.name == 'nt' else 'clear')
        visualize_grid(grid, path, t)
    if plot:
        visualize_plot(grid, path, t)
    return path, total_cost, nodes, exec_time

def visualize_grid(grid, path=None, t=0, borders=False, show_costs_on_path=False, use_colors=False):
    if isinstance(grid, DynamicGrid):
        dyn_pos = grid.get_dynamic_pos(t)
        dyn_note = f"\nDynamic: D at {dyn_pos} (moves right 1 cell/step, wraps at col {grid.cols})"
    else:
        dyn_note = ""
    
    print(f"Grid ({'Dynamic' if isinstance(grid, DynamicGrid) else 'Static'} {grid.rows}x{grid.cols}, t={t})")
    
    grid_vis = [['' for _ in range(grid.cols)] for _ in range(grid.rows)]
    path_set = set(path) if path else set()
    
    current = path[-1] if path else grid.start
    
    for i in range(grid.rows):
        for j in range(grid.cols):
            pos = (i, j)
            c = grid.costs[i][j]
            if pos == grid.start and pos == current:
                display = 'S'
            elif pos == grid.goal and pos == current:
                display = 'G'
            elif pos == current and pos not in [grid.start, grid.goal]:
                display = '@'
            elif isinstance(grid, DynamicGrid) and pos == dyn_pos:
                display = 'D'
            elif pos in path_set and pos not in [grid.start, grid.goal, current]:
                display = '*' if not show_costs_on_path else str(c)
            else:
                display = '#' if c == -1 else str(c)
            
            grid_vis[i][j] = display
    
    for row in grid_vis:
        print(' '.join(row))
    
    if path:
        print(f"Path: {' -> '.join(str(p) for p in path)} (Cost: {sum(grid.get_cost(p[0], p[1]) for p in path[1:])})")
    
    print("Legend: #=obstacle | S=start | G=goal | *=path | D=dynamic | @=current | Numbers=cost")
    if dyn_note:
        print(dyn_note)
    print("-" * 40)

def visualize_plot(grid, path=None, t=0):
    try:
        import matplotlib.pyplot as plt
        import numpy as np
        costs = np.array(grid.costs)
        costs_masked = np.ma.masked_where(costs == -1, costs)
        fig, ax = plt.subplots(figsize=(8, 8))
        im = ax.imshow(costs_masked, cmap='Blues', vmin=0, vmax=5, origin='upper')
        plt.colorbar(im, ax=ax, shrink=0.8)
        
        obs_y, obs_x = np.where(costs == -1)
        ax.scatter(obs_x, obs_y, color='black', s=80, marker='s', zorder=5)
        
        if path:
            path_x, path_y = np.array([p[0] for p in path]), np.array([p[1] for p in path])
            ax.plot(path_y, path_x, 'r-', linewidth=3, marker='o', markersize=5, zorder=4)
        
        ax.plot(grid.start[1], grid.start[0], 'go', markersize=15, zorder=6, label='Start')
        ax.plot(grid.goal[1], grid.goal[0], 'ro', markersize=15, zorder=6, label='Goal')
        if isinstance(grid, DynamicGrid):
            dyn = grid.get_dynamic_pos(t)
            ax.plot(dyn[1], dyn[0], 'ko', markersize=12, zorder=6, label=f'Dynamic (t={t})')
        
        if path:
            current = path[-1]
            if current not in [grid.start, grid.goal]:
                ax.plot(current[1], current[0], 'y^', markersize=12, zorder=6, label='Current')
        
        ax.set_xticks(np.arange(-0.5, grid.cols, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, grid.rows, 1), minor=True)
        ax.grid(which='minor', alpha=0.3)
        ax.set_xticks(np.arange(0, grid.cols+1, 1))
        ax.set_yticks(np.arange(0, grid.rows+1, 1))
        
        ax.legend(loc='upper right', fontsize=9)
        plt.tight_layout()
        plt.show()
    except ImportError:
        print("Matplotlib not installed. Install with 'pip install matplotlib' or use --visualize for ASCII.")
    except Exception as e:
        print(f"Plot error: {e}. Use --visualize for ASCII.")

maps = {
    'small': """5 5 0 0 4 4
1 1 1 1 1
1 -1 2 1 1
1 1 1 3 -1
1 1 1 1 1
1 1 1 1 1""",
    'medium': """10 10 0 0 9 9
1 1 1 1 1 1 1 1 1 1
1 1 -1 1 2 1 1 1 1 1
1 2 1 1 1 3 1 1 1 1
1 1 1 -1 1 1 1 2 1 1
1 1 2 1 1 1 1 1 1 1
1 1 1 1 1 -1 1 1 1 1
1 3 1 1 2 1 1 1 1 1
1 1 1 1 1 1 3 1 1 1
1 1 2 1 1 1 1 1 -1 1
1 1 1 1 1 1 1 1 1 1""",
    'dynamic': """10 10 0 0 9 9
1 1 1 1 1 1 1 1 1 1
1 1 -1 1 2 1 1 1 1 1
1 2 1 1 1 3 1 1 1 1
1 1 1 -1 1 1 1 2 1 1
1 1 2 1 1 1 1 1 1 1
1 1 1 1 1 -1 1 1 1 1
1 3 1 1 2 1 1 1 1 1
1 1 1 1 1 1 3 1 1 1
1 1 2 1 1 1 1 1 -1 1
1 1 1 1 1 1 1 1 1 1""",
    'large': """20 20 0 0 19 19
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
1 1 -1 1 1 2 1 1 1 1 1 3 1 1 1 -1 1 1 1 1
1 1 1 2 1 1 1 -1 1 1 1 1 2 1 1 1 1 3 1 1
1 1 1 1 3 1 1 1 1 2 1 1 1 -1 1 1 1 1 1 1
1 2 1 1 1 1 1 1 1 1 3 1 1 1 2 1 1 1 -1 1
1 1 1 -1 1 2 1 1 1 1 1 1 1 1 1 3 1 1 1 1
1 1 1 1 1 1 3 1 1 2 1 1 -1 1 1 1 1 1 2 1
1 3 1 1 1 1 1 1 1 1 1 2 1 1 1 1 1 -1 1 1
1 1 2 1 1 -1 1 1 3 1 1 1 1 2 1 1 1 1 1 1
1 1 1 1 1 1 1 2 1 1 1 1 3 1 -1 1 1 2 1 1
1 2 1 1 1 1 1 1 1 3 1 1 1 1 1 2 1 1 1 -1
1 1 -1 1 2 1 1 1 1 1 1 1 1 1 3 1 1 1 1 1
1 1 1 1 1 3 1 1 2 1 1 -1 1 1 1 1 2 1 1 1
1 1 1 2 1 1 1 1 1 1 1 1 1 3 1 1 1 -1 1 1
1 3 1 1 1 1 2 1 1 1 -1 1 1 1 1 1 1 1 2 1
1 1 1 1 1 1 1 3 1 1 1 2 1 1 1 1 -1 1 1 1
1 1 2 1 1 -1 1 1 1 1 1 1 3 1 1 2 1 1 1 1
1 1 1 1 3 1 1 1 2 1 1 1 1 1 1 1 1 3 1 -1
1 2 1 1 1 1 1 1 1 1 3 1 2 1 1 1 1 1 1 1
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"""
}

def main():
    parser = argparse.ArgumentParser(description="Autonomous Delivery Agent")
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--planner', choices=['ucs', 'astar', 'local'], default='ucs')
    group.add_argument('--manual', action='store_true', help="Manually control the agent with w/s/a/d")
    parser.add_argument('--map', choices=maps.keys(), default='small')
    parser.add_argument('--visualize', action='store_true', help="Show ASCII grid viz")
    parser.add_argument('--plot', action='store_true', help="Show matplotlib plot (install matplotlib)")
    args = parser.parse_args()

    map_str = maps[args.map]
    if args.map == 'dynamic':
        grid = DynamicGrid(map_str)
    else:
        grid = Grid(map_str)

    if args.manual:
        path, cost, nodes, exec_time = manual_control(grid, args.visualize, args.plot)
    elif args.planner == 'local' and args.map == 'dynamic':
        path, cost, nodes, exec_time = simulate_delivery(grid)
    elif args.planner == 'local':
        path, cost, nodes, exec_time = local_search(grid, grid.start, 0, grid.goal)
    elif args.planner == 'ucs':
        path, cost, nodes, exec_time = uniform_cost_search(grid, use_time=(args.map == 'dynamic'))
    else:
        path, cost, nodes, exec_time = a_star(grid, use_time=(args.map == 'dynamic'))

    print(f"Mode: {'manual' if args.manual else args.planner}, Map: {args.map}")
    print(f"Path length: {len(path) if path else 0}")
    print(f"Cost: {cost}")
    print(f"Nodes expanded: {nodes}")
    print(f"Time: {exec_time:.4f}s")
    if path:
        print(f"Path: {path}")

    if (args.visualize or args.plot) and not args.manual:
        t = 0
        if args.visualize:
            os.system('cls' if os.name == 'nt' else 'clear')
            visualize_grid(grid, path, t)
        if args.plot:
            visualize_plot(grid, path, t)

if __name__ == "__main__":
    main()
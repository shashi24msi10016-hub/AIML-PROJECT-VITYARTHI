# Pathfinding Simulation

This Python project implements a pathfinding simulation on a grid-based map with static or dynamic obstacles. It supports multiple pathfinding algorithms (Uniform Cost Search, A* Search, Local Search) and a manual control mode for navigating the grid. The simulation can visualize the grid using ASCII art or Matplotlib plots.

## Features
- **Grid Types**:
  - Static grid with fixed obstacles and costs.
  - Dynamic grid with a moving obstacle in a specified row.
- **Pathfinding Algorithms**:
  - Uniform Cost Search (UCS)
  - A* Search with Manhattan distance heuristic
  - Local Search with multiple restarts
- **Dynamic Simulation**: Replans the path using Local Search if a dynamic obstacle blocks the agent.
- **Manual Control**: Navigate the grid using `w` (up), `s` (down), `a` (left), `d` (right), and `q` (quit).
- **Visualization**:
  - ASCII-based grid visualization.
  - Matplotlib-based graphical visualization (optional, requires Matplotlib).

## Requirements
- Python 3.6 or higher
- Optional: `matplotlib` for graphical visualization (`pip install matplotlib`)

## Installation
1. Clone or download the repository.
2. Ensure Python 3 is installed.
3. (Optional) Install Matplotlib for plotting:
   ```bash
   pip install matplotlib
   ```

## Usage
Run the script with command-line arguments to select the map, planner, and visualization options.

### Command-Line Arguments
- `--map`: Choose the grid map (`small`, `medium`, `large`, `dynamic`). Default: `small`
- `--planner`: Select the pathfinding algorithm (`ucs`, `astar`, `local`). Default: `ucs`
- `--manual`: Enable manual control mode (mutually exclusive with `--planner`).
- `--visualize`: Display ASCII grid visualization.
- `--plot`: Display Matplotlib plot (requires Matplotlib installed).

### Examples
1. Run UCS on the small map with ASCII visualization:
   ```bash
   python pathfinding.py --map small --planner ucs --visualize
   ```
2. Run A* on the dynamic map:
   ```bash
   python pathfinding.py --map dynamic --planner astar
   ```
3. Use manual control on the medium map with Matplotlib plot:
   ```bash
   python pathfinding.py --map medium --manual --plot
   ```
4. Simulate delivery with Local Search on the dynamic map:
   ```bash
   python pathfinding.py --map dynamic --planner local
   ```

### Output
The script outputs:
- The mode (planner or manual) and map used.
- Path length, total cost, nodes expanded, and execution time.
- The path taken (list of coordinates).
- Visualization (if enabled) showing the grid, path, start (`S`), goal (`G`), obstacles (`#`), dynamic obstacle (`D`), and current position (`@`).

## Map Format
Maps are defined as strings with:
- First line: `rows cols start_x start_y goal_x goal_y`
- Subsequent lines: Grid rows with space-separated integers:
  - Positive numbers represent cell costs.
  - `-1` represents an obstacle.
- Predefined maps: `small` (5x5), `medium` (10x10), `large` (20x20), `dynamic` (10x10 with a moving obstacle in row 5).

## Notes
- The dynamic map uses a moving obstacle that shifts right one cell per time step, wrapping around at the grid's right edge.
- Manual control requires user input for each move and checks for valid moves and obstacles.
- Matplotlib visualization requires the `matplotlib` library; without it, use `--visualize` for ASCII output.
- The Local Search algorithm uses a greedy approach with restarts and may not always find the optimal path.

## License
This project is licensed under the MIT License.
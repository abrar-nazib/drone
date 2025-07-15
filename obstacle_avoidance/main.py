import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class Node:
    def __init__(self, position):
        self.position = position
        self.parent = None
        self.g_cost = float("inf")
        self.h_cost = float("inf")

    def __eq__(self, other):  # For comparing nodes
        return self.position == other.position

    def calculate_f_cost(self):
        self.f_cost = self.g_cost + self.h_cost


class AStarPlanner:
    def __init__(self, grid_width, grid_height, grid_depth, start, goal, obstacles):
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.grid_depth = grid_depth
        self.start = Node(start)
        self.goal = Node(goal)
        self.obstacles = obstacles
        self.grid = np.zeros((grid_width, grid_height, grid_depth))
        self.path = None

    def create_grid(self):
        for obstacle in self.obstacles:
            x, y, z = obstacle
            self.grid[x][y][z] = 1

    @staticmethod
    def heuristic(current, goal):
        return np.sqrt(
            np.sum((np.array(current.position) - np.array(goal.position)) ** 2)
        )

    def astar(self):
        # Set up the open and closed sets
        open_set = [self.start]
        closed_set = []
        self.start.g_cost = 0
        self.start.h_cost = self.heuristic(self.start, self.goal)
        self.start.calculate_f_cost()

        while open_set:
            # Find the node with the lowest f_cost
            current = min(open_set, key=lambda node: node.f_cost)

            if current == self.goal:
                self.path = []
                while current:
                    self.path.append(current.position)
                    current = current.parent
                self.path.reverse()
                return

            open_set.remove(current)
            closed_set.append(current)

            neighbors = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    for dz in [-1, 0, 1]:
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        new_pos = (
                            current.position[0] + dx,
                            current.position[1] + dy,
                            current.position[2] + dz,
                        )
                        if (
                            new_pos[0] < 0
                            or new_pos[0] >= self.grid_width
                            or new_pos[1] < 0
                            or new_pos[1] >= self.grid_height
                            or new_pos[2] < 0
                            or new_pos[2] >= self.grid_depth
                            or self.grid[new_pos[0]][new_pos[1]][new_pos[2]] == 1
                        ):
                            continue
                        new_node = Node(new_pos)
                        neighbors.append(new_node)

            for neighbor in neighbors:
                if neighbor in closed_set:
                    continue
                new_g_cost = current.g_cost + self.heuristic(current, neighbor)
                if new_g_cost < neighbor.g_cost or neighbor not in open_set:
                    neighbor.g_cost = new_g_cost
                    neighbor.h_cost = self.heuristic(neighbor, self.goal)
                    neighbor.calculate_f_cost()
                    neighbor.parent = current
                    if neighbor not in open_set:
                        open_set.append(neighbor)

        return None

    def plot_grid(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        for x in range(self.grid_width):
            for y in range(self.grid_height):
                for z in range(self.grid_depth):
                    if self.grid[x][y][z] == 1:
                        ax.scatter(x, y, z, color="black", marker="s", s=200)

        ax.scatter(
            self.start.position[0],
            self.start.position[1],
            self.start.position[2],
            color="green",
            marker="s",
        )
        ax.scatter(
            self.goal.position[0],
            self.goal.position[1],
            self.goal.position[2],
            color="red",
            marker="s",
        )

        manager = plt.get_current_fig_manager()
        try:
            manager.full_screen_toggle()
        except AttributeError as e:
            print(e)
            manager.window.maximize()

        if self.path is not None:
            for i in range(1, len(self.path)):
                ax.plot(
                    [self.path[i - 1][0], self.path[i][0]],
                    [self.path[i - 1][1], self.path[i][1]],
                    [self.path[i - 1][2], self.path[i][2]],
                    color="blue",
                )
                plt.pause(0.5)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        plt.show()


# Define the grid dimensions
grid_width = 50
grid_height = 50
grid_depth = 50

# Define the start and goal points
start = (0, 5, 20)
goal = (40, 8, 28)

# Generate 20 obstacles between the start and goal points
obstacles = []
for i in range(20, 25):
    obstacles.append((i, 5, 20))
    obstacles.append((i, 4, 20))
    obstacles.append((i, 6, 20))

    obstacles.append((i, 5, 21))
    obstacles.append((i, 4, 21))
    obstacles.append((i, 6, 21))

    obstacles.append((i, 5, 19))
    obstacles.append((i, 4, 19))
    obstacles.append((i, 6, 19))

obstacles.append((0, 0, 0))
obstacles.append((30, 30, 30))

planner = AStarPlanner(grid_width, grid_height, grid_depth, start, goal, obstacles)
planner.create_grid()
planner.astar()
planner.plot_grid()

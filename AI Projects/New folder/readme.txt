Develop an application in a programming language of your choice to implement the search methods we discussed in the course for the robot navigation problem. This assignment weights 20% on the final grade of the course.

There is a robot that may move in a rectangular grid of size NxN. Some cells in the grid contain obstacles and the others are free for the robot to move in. The robot starts from a given cell in the grid and the goal is to reach a specific cell. The robot may move horizontally or vertically, one cell in each step, obviously not in cells that contain obstacles. The cost of each horizontal or vertical move equals to 1. The application should accept as an option the possibility for the robot to move also diagonally, with cost either 1 or 1.5 (also an option). You have to implement the methods:

- Breadth first
- Uniform cost
- Depth first
- Iterative deepening
- Greedy
- A*
**README.md**

# Traveling Salesman Problem Solver

## Overview

This C++ project provides a solution to the Traveling Salesman Problem (TSP) using genetic algorithms and graph theory. The application aims to optimize routes for a given set of cities, represented as nodes in a graph, considering both distance and capacity constraints.

## Features

- **Genetic Algorithm:** The project employs a genetic algorithm to find an approximate solution to the TSP, evolving routes over multiple generations.
  
- **Graph Theory Operations:** The application includes graph operations such as finding the Minimum Spanning Tree (MST) using Kruskal's algorithm and computing the maximum flow in the graph.

- **Input from File:** Graph data, including distances and capacities, can be read from an external file ("datos.txt") for easy customization.

- **Coordinate Representation:** Cities are represented as coordinates in the input file, enabling real-world positioning for route visualization.

## Installation

1. Clone the repository to your local machine.
   
```bash
git clone https://github.com/your-username/tsp-genetic-algorithm.git
```

2. Compile the C++ code using your preferred compiler.

```bash
g++ main.cpp -o tsp_solver
```

3. Run the executable.

```bash
./tsp_solver
```

## Usage

1. Ensure that the "datos.txt" file contains the correct graph data. Adjust the file if needed.

2. Execute the compiled binary to find optimized routes for the TSP.

3. Review the generated results, including the best route found and its length.

## Sample "datos.txt" File

```plaintext
4
0 16 45 32
16 0 18 21
45 18 0 7
32 21 7 0
0 48 12 18
52 0 42 32
18 46 0 56
24 36 52 0
(200,500)
(300,100)
(450,150)
(520,480)
```

Coordinates in parentheses represent the positions of cities in a two-dimensional space.

## Contributions

Contributions and improvements are welcome. Feel free to fork the repository and submit pull requests.

## License

This project is licensed under the [MIT License](LICENSE).

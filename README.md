![Screenshot 2024-03-12 001248](https://github.com/ChinonsoAgbo/searchAlgorithms/assets/81990068/ab0969b2-b07c-4c15-9ecd-a5b032bdefd5)
Algortithm Image 

In addition to the normal floor, the dungeon can also contain walls, water and gold. These elements should help to understand the possibilities and limitations of different graph-based search algorithms.

After opening the GUI, the floor tiles of the dungeon map can be changed by clicking on them. The current dungeon map should then be converted into a graph by pressing the Enter key and started with - Dijkstra's search methods, - an A* algorithm - or an algorithm that finds the best solution even with negative weights.

The visited path should be visualised (see below). I generated a graph from dungeon tiles and Implemented the method in generateGraph() in the DemoPanel class.This should transfer the current dungeon map into a graph. 

Please note the following when generating the graph: - The top, bottom, left and right neighbour tiles can be accessed from a tile (if they are not on the edge). - Entering a normal tile (normal = true) costs 1 - Entering a water tile (water = true) costs 3 - Entering a gold tile (coin = true) costs -1000 (it's worth it!). - Wall tiles (solid = true) cannot be entered. 

### Search Algorithm

I implemented 

- Dijkstra search algorithm in the `searchDijkstra()` method. and
- Implement an A* algorithm in the `searchAStar()` method

that both collects all gold coins and reaches the goal as quickly as possible. Some more detailed instructions can be found in the comments to the source code. To use different algorithms, set the ALGORITHM member to the desired algorithm (e.g. final int ALGORITHM = DIJKSTRA;). At the end of each run, the cumulative path weight and the required iterations should be displayed in the window or in the console.

import javax.swing.JPanel;
import java.awt.*;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.PriorityQueue;

public class DemoPanel extends JPanel {

    private Graph graph;
    final int maxCol = 15;
    final int maxRow = 10;
    final int nodeSize = 70;
    final int screenWidth = nodeSize * maxCol;
    final int screenHeight = nodeSize * maxRow;
    FloorTile[][] floorTile = new FloorTile[maxCol][maxRow];
    FloorTile startFloorTile, goalFloorTile, currentFloorTile;
    boolean goalReached = false;
    int step = 0;
    int maxSteps = 1000;
    final int DIJKSTRA = 0;
    final int ASTAR = 1;
    final int BELLMANNFORD = 2;
    //    final int ALGORITHM = DIJKSTRA;
//    final int ALGORITHM = ASTAR;

    final int ALGORITHM = BELLMANNFORD;


    public DemoPanel() {
        this.setPreferredSize(new Dimension(screenWidth, screenHeight));
        this.setBackground(Color.black);
        this.setLayout(new GridLayout(maxRow, maxCol));
        this.addKeyListener(new KeyHandler(this));
        this.setFocusable(true);

        int col = 0;
        int row = 0;

        while (col < maxCol && row < maxRow) {
            floorTile[col][row] = new FloorTile(col, row);
            this.add(floorTile[col][row]);
            col++;
            if (col == maxCol) {
                col = 0;
                row++;
            }
        }
        setStartTile(2, 6);
        setGoalTile(11, 3);

        setSolidTile(10, 2);
        setSolidTile(10, 3);
        setSolidTile(10, 4);
        setSolidTile(10, 5);
        setSolidTile(10, 6);
        setSolidTile(10, 7);

        setSolidTile(6, 2);
        setSolidTile(7, 2);
        setSolidTile(8, 2);
        setSolidTile(9, 2);

        setSolidTile(11, 7);
        setSolidTile(12, 7);
        setSolidTile(6, 1);

        setWaterTile(5, 0);
        setWaterTile(6, 0);
        setWaterTile(7, 0);

        setCoinTile(7, 1);
        setCoinTile(8, 1);
        setCoinTile(9, 1);
    }

    /**
     * Sets the start tile at the specified column and row.
     *
     * @param col The column of the start tile.
     * @param row The row of the start tile.
     */
    private void setStartTile(int col, int row) {
        floorTile[col][row].setAsStart();
        startFloorTile = floorTile[col][row];
        currentFloorTile = startFloorTile;
    }

    private void setGoalTile(int col, int row) {
        floorTile[col][row].setAsGoal();
        goalFloorTile = floorTile[col][row];
    }

    private void setSolidTile(int col, int row) {
        floorTile[col][row].setAsSolid();
    }

    private void setWaterTile(int col, int row) {
        floorTile[col][row].setAsWater();
    }

    private void setCoinTile(int col, int row) {
        floorTile[col][row].setAsCoin();
    }

    /**
     * Searches for the optimal path based on the selected algorithm.
     */
    public void search() {
        if (ALGORITHM == DIJKSTRA) {
            searchDijkstra();
        } else if (ALGORITHM == ASTAR) {
            searchAStar();
        } else if (ALGORITHM == BELLMANNFORD) {
            searchBellmannFord();
        }
    }

    /**
     * Generates a graph based on the current floor tiles and adds nodes and edges accordingly.
     */
    private void generateGraph() {
        graph = new Graph();
        for (int col = 0; col < maxCol; col++) {
            for (int row = 0; row < maxRow; row++) {
                FloorTile currentTile = floorTile[col][row];
                String label = col + "/" + row;
                graph.addNode(label, currentTile);
            }
        }
        for (int col = 0; col < maxCol; col++) {
            for (int row = 0; row < maxRow; row++) {
                String label = col + "/" + row;
                if (col + 1 < maxCol) {
                    addInspectedEdge(col + 1, row, label);
                }
                if (col - 1 >= 0) {
                    addInspectedEdge(col - 1, row, label);
                }
                if (row + 1 < maxRow) {
                    addInspectedEdge(col, row + 1, label);
                }
                if (row - 1 >= 0) {
                    addInspectedEdge(col, row - 1, label);
                }
            }
        }
        System.out.println("");
        //TODO: implement a method to write the current dungeon tiles to a graph
    }

    private void addInspectedEdge(int x, int y, String label) {
        FloorTile southTile = floorTile[x][y];
        String destLabel = (x) + "/" + y;
        if (southTile.solid) {
            return;
        }
        if (southTile.normal) {
            graph.addEdge(label, destLabel, 1);
        }
        if (southTile.coin) {
            graph.addEdge(label, destLabel, ALGORITHM == DIJKSTRA ? 1 : -1000);
        }
        if (southTile.water) {
            graph.addEdge(label, destLabel, 3);
        }
    }

    /**
     * Searches for the optimal path using Dijkstra's algorithm.
     */
    //TODO: implement a method find the optimal path" using Dijkstra's algorithm
    //TODO: this method will be executed by pressing the enter key
    //TODO: mark all visited nodes in the dungeon map orange using the setAsVisited() method
    //TODO: and all nodes on the optimal path green using the setAsPath() method
    //TODO: use an array of BSFItems to save the optimal path
    //TODO: print the accumulated path length in the console
    //TODO: once the algorithm reached the goal tile you can stop the loop using the goalReached variable
    //TODO: in case there's no solution you can stop the loop by using the maxSteps variable
    // first the generateGraph method is called
    private void searchDijkstra() {

        generateGraph();
        List<Node> nodes = graph.getNodes();
        // using BreadthFirstSearch Items for Dijkstra's Algorithm as well as above
        BFSItem[] bfsi_table = new BFSItem[nodes.size()];

        int i;
        int initCapacity = 150;
        // For Dijkstra's algorithm, the element with the lowest current weight needs to be selected.
        // A PriorityQueue always sorts itself such that the next element to be polled is the smallest
        // To achieve that, the PriorityQueue needs a Comparator to sort elements accordingly
        Graph.AdjacencyElemLessComparator comp = new Graph.AdjacencyElemLessComparator();
        PriorityQueue<AdjElement> queue = new PriorityQueue<>(initCapacity, comp);

        // initializing all nodes
        for (i = 0; i < nodes.size(); i++) {
            // unprocessed nodes are set to the maximum negative int value and their predecessor is set to null
            bfsi_table[i] = new BFSItem(-Integer.MAX_VALUE, null, nodes.get(i).getKey());
        }
        int start_id = graph.getNodeID(startFloorTile.col + "/" + startFloorTile.row);
        // Starting the search from a given start_id. The bfsi_table contains the result,
        // and queue is the priority queue
        searchMinimalSpanningTree(start_id, bfsi_table, queue);

    }

    /**
     * Performs a breadth-first search to find the minimal spanning tree starting from the given index in the graph.
     *
     * @param index      The index of the starting node in the graph.
     * @param bfsi_table An array of BFSItem objects representing the state of each node during the search.
     * @param queue      PriorityQueue of AdjElement objects to manage the order of exploration.
     */
    private void searchMinimalSpanningTree(int index,
                                           BFSItem[] bfsi_table,
                                           PriorityQueue<AdjElement> queue) {
        List<Node> nodes = graph.getNodes();
        int startIndex = 0;
        int weight = 0;
        AdjElement adjE = null;
        Node node = null;
        ArrayList<AdjElement> adjL = null;
        boolean changed = false;
        AdjElement currentE = null;
        node = nodes.get(index);
        String goalKey = "";
        if (node != null) {
            // add start node to PriorityQueue
            currentE = new AdjElement(node.getID(), -Integer.MAX_VALUE);
            addPQueueElement(queue, currentE);
            bfsi_table[index].pred = null;
        }
        do {  // repeat while there are still elements in the PriorityQueue
            // poll element with the smallest weight and get the respective Node object
            currentE = queue.poll();
            currentFloorTile = (FloorTile) node.getData();
            currentFloorTile.setAsVisited();
            if (currentFloorTile.goal) {
                goalReached = true;
                goalKey = node.getKey();
            }
            startIndex = currentE.getNodeIndex();
            node = nodes.get(startIndex);
            System.out.println(node.getData());
            // flip the sign if the element is polled from the queue to mark it finished
            bfsi_table[startIndex].dist = -bfsi_table[startIndex].dist;
            // the distance for the starting node is set to 0
            if (bfsi_table[startIndex].dist == Integer.MAX_VALUE) bfsi_table[startIndex].dist = 0;
            // get the adjacency list of the start node and iterate over all elements
            adjL = node.getAdjacencies();
            Iterator<AdjElement> iter = adjL.iterator();
            while (iter.hasNext()) {
                adjE = iter.next();
                int subIndex = adjE.getNodeIndex();
                // get current weight of adjacent element
                weight = adjE.getWeight();
                //if the current cumulative distance is lower than 0, the element has not been finished yet
                if (bfsi_table[subIndex].dist < 0) {
                    // in this case the cumulative distance can be set to the predecessor distance plus the current
                    // node's weight
                    int cW = bfsi_table[startIndex].dist + weight;
                    AdjElement e = new AdjElement(subIndex, cW);
                    // adding the current element to the queue
                    // if the element is already in the list the new cumulative distance is only updated if
                    // it is lower than the one already saved in the queue
                    // changed is set to true if an element has been updated
                    changed = addPQueueElement(queue, e);

                    if (changed) {
                        // if an update took place the result bfsi_table is updated accordingly
                        bfsi_table[subIndex].dist = -cW;
                        bfsi_table[subIndex].pred = node.getKey();
                    }
                }
            } //end while(iter.hasNext())
        } //end do
        while (!queue.isEmpty() && !goalReached);

        currentFloorTile = goalFloorTile;
        while (!currentFloorTile.equals(startFloorTile)) {
            currentFloorTile.setAsPath();
            int currentIndex = getIndex(bfsi_table, goalKey);
            if (currentIndex == -1) {
                return;
            }
            BFSItem currentItem = bfsi_table[currentIndex];
            goalKey = currentItem.pred;
            currentFloorTile = (FloorTile) nodes.get(graph.getNodeID(goalKey)).getData();
        }
    }

    /**
     * Adds or updates an AdjElement in the PriorityQueue based on its node index and weight.
     *
     * @param queue The PriorityQueue<AdjElement> to add/update the element in.
     * @param e     The AdjElement to be added or updated in the PriorityQueue.
     * @return True if the element has been added or updated, false otherwise.
     */
    private boolean addPQueueElement(PriorityQueue<AdjElement> queue, AdjElement e) {
        boolean found = false;
        boolean add = true;
        // adding AdjElement e to PriorityQueue<AdjElement> queue
        // if the element already exists the new weight of e needs to be lower than the currently saved one
        // otherwise the element won't be added
        // returns true if an element has been added and false otherwise
        Iterator<AdjElement> iter = queue.iterator();
        while (iter.hasNext() && !found) {
            AdjElement myE = iter.next();
            if (myE.getNodeIndex() == e.getNodeIndex()) {
                found = true;
                if (myE.getWeight() > e.getWeight()) {
                    queue.remove(myE);
                } else {
                    add = false;
                }
            }
        }
        if (add) {
            queue.add(e);
        }
        return (add);
    }

    /**
     * Finds the index of a BFSItem with a given node label in the provided array.
     * @param items The array of BFSItem objects to search.
     * @param label The node label to search for.
     * @return The index of the BFSItem with the specified label if found; otherwise, returns -1.
     */
    private int getIndex(BFSItem[] items, String label) {
        for (int i = 0; i < items.length; i++) {
            if (items[i].node_label.equals(label)) {
                return i;
            }
        }
//        System.out.println("Label not found: " + label);

        return -1;
    }

    /**
     * Executes the A* search algorithm to find the optimal path in the graph.
     * The A* algorithm is initiated from the starting node and iteratively explores nodes
     * based on their heuristic value and the accumulated cost.
     * Once the goal is reached or the maximum steps are reached, the algorithm stops.
     * Visited nodes are marked orange, and nodes on the optimal path are marked green.
     * The optimal path length is printed in the console.
     * Pressing the enter key triggers the execution of this method.
     */
    private void searchAStar() {
        //TODO: implement a method find the optimal path using the A* algorithm
        //TODO: this method will be executed by pressing the enter key
        //TODO: mark all visited nodes in the dungeon map orange using the setAsVisited() method
        //TODO: and all nodes on the optimal path green using the setAsPath() method
        //TODO: use an array of BSFItems to save the optimal path
        //TODO: print the accumulated path length in the console
        //TODO: once the algorithm reached the goal tile you can stop the loop using the goalReached variable
        //TODO: in case there's no solution you can stop the loop by using the maxSteps variable
        // first the generateGraph method is called

        // first, the generateGraph method is called

        generateGraph();

        List<Node> nodes = graph.getNodes();

        // Initialization of A* table and priority queue
        BFSItem[] astar_table = new BFSItem[nodes.size()];

        int i;
        int initCapacity = 150;
        Graph.AdjacencyElemLessComparator comp = new Graph.AdjacencyElemLessComparator();
        PriorityQueue<AdjElement> queue = new PriorityQueue<>(initCapacity, comp);

        for (i = 0; i < nodes.size(); i++) {
            astar_table[i] = new BFSItem(-Integer.MAX_VALUE, null, nodes.get(i).getKey());
        }
        // Starting node ID obtained from the graph based on the starting floor tile
        int start_id = graph.getNodeID(startFloorTile.col + "/" + startFloorTile.row);
        // Execute A* algorithm
        searchAStarAlgorithm(start_id, astar_table, queue);

    }

    /**
     * Executes the A* search algorithm from a given index in the graph to find the optimal path.
     *
     * @param index       The index of the starting node in the graph.
     * @param astar_table An array of BFSItem objects representing the state of each node during the search.
     * @param queue       PriorityQueue of AdjElement objects to manage the order of exploration.
     */
    private void searchAStarAlgorithm(int index, BFSItem[] astar_table, PriorityQueue<AdjElement> queue) {
        List<Node> nodes = graph.getNodes();
        int startIndex = 0;
        int weight = 0;
        AdjElement astarItem = null;
        Node node = null;
        ArrayList<AdjElement> adjL = null;
        AdjElement adjE = null;
        boolean changed = false;
        String goalKey = "";

        // Initialize the start node
        node = nodes.get(index);
        astarItem = new AdjElement(node.getID(), -Integer.MAX_VALUE);
        addAStarQueueElement(queue, astarItem);
        astar_table[index].pred = null;

        do {
            astarItem = queue.poll();
            currentFloorTile = (FloorTile) node.getData();
            currentFloorTile.setAsVisited();

            if (currentFloorTile.goal) {
                goalReached = true;
                goalKey = node.getKey();
            }

            startIndex = astarItem.getNodeIndex();
            node = nodes.get(startIndex);
            System.out.println(node.getData());

            astar_table[startIndex].dist = -astar_table[startIndex].dist;

            if (astar_table[startIndex].dist == Integer.MAX_VALUE) {
                astar_table[startIndex].dist = 0;
            }

            adjL = node.getAdjacencies();
            Iterator<AdjElement> iter = adjL.iterator();

            while (iter.hasNext()) {
                adjE = iter.next();
                int subIndex = adjE.getNodeIndex();
                weight = adjE.getWeight();

                if (astar_table[subIndex].dist < 0) {
                    int cW = astar_table[startIndex].dist + weight;
                    int heuristic = calculateHeuristic(nodes.get(subIndex));

                    AdjElement e = new AdjElement(subIndex, cW + heuristic);
                    changed = addAStarQueueElement(queue, e);

                    if (changed) {
                        astar_table[subIndex].dist = -cW;
                        astar_table[subIndex].pred = node.getKey();
                    }
                }
            }
        } while (!queue.isEmpty() && !goalReached);

        currentFloorTile = goalFloorTile;
        while (!currentFloorTile.equals(startFloorTile)) {
            currentFloorTile.setAsPath();
            int currentIndex = getIndex(astar_table, goalKey);
            if (currentIndex == -1) {
                return;
            }
            BFSItem currentItem = astar_table[currentIndex];
            goalKey = currentItem.pred;
            currentFloorTile = (FloorTile) nodes.get(graph.getNodeID(goalKey)).getData();
        }
    }

    /**
     * Adds or updates an AdjElement in the PriorityQueue based on its node index and weight for A* algorithm.
     *
     * @param queue The PriorityQueue<AdjElement> to add/update the element in.
     * @param e     The AdjElement to be added or updated in the PriorityQueue.
     * @return True if the element has been added or updated, false otherwise.
     */
    private boolean addAStarQueueElement(PriorityQueue<AdjElement> queue, AdjElement e) {
        boolean found = false;
        boolean add = true;

        // Iterate through the PriorityQueue to find the element with the same node index
        Iterator<AdjElement> iter = queue.iterator();
        while (iter.hasNext() && !found) {
            AdjElement myE = iter.next();
            if (myE.getNodeIndex() == e.getNodeIndex()) {
                found = true;

                // If the existing element has a greater weight, remove it
                // Otherwise, do not add the new element

                if (myE.getWeight() > e.getWeight()) {
                    queue.remove(myE);
                } else {
                    add = false;
                }
            }
        }
        // If the element is not found or has a lower weight, add the new element to the PriorityQueue

        if (add) {
            queue.add(e);
        }
        return add;
    }
    /**
     * Calculates the heuristic value (estimated cost) from the given node to the goal node.
     * The heuristic used is the Manhattan distance between the current node and the goal node.
     *
     * @param node The node for which the heuristic value is calculated.
     * @return The calculated heuristic value representing the estimated cost to reach the goal.
     */
    private int calculateHeuristic(Node node) {

        // This is where the estimated cost from the current node to the goal is calculated
        FloorTile currentTile = (FloorTile) node.getData();
        return Math.abs(currentTile.col - goalFloorTile.col) + Math.abs(currentTile.row - goalFloorTile.row);
    }

    /**
     * Executes the Bellman-Ford algorithm to find the optimal path in the graph.
     * The Bellman-Ford algorithm is initiated from the starting node and iteratively relaxes
     * the edges until the shortest paths are found or a negative cycle is detected.
     * Visited nodes are marked orange, and nodes on the optimal path are marked green.
     * The optimal path length is printed in the console.
     * The algorithm stops when the goal is reached or the maximum steps are reached.
     * Pressing the enter key triggers the execution of this method.
     */
    private void searchBellmannFord() {
        // TODO: mark all visited nodes in the dungeon map orange using the setAsVisited() method
        // TODO: and all nodes on the optimal path green using the setAsPath() method
        // TODO: use an array of BFSItems to save the optimal path
        // TODO: print the accumulated path length in the console
        // TODO: once the algorithm reached the goal tile you can stop the loop using the goalReached variable
        // TODO: in case there's no solution you can stop the loop by using the maxSteps variable
        // first the generateGraph method is called
        generateGraph();
        List<Node> nodes = graph.getNodes();

        // using Bellman-Ford Items
        BFSItem[] bellmanFordTable = new BFSItem[nodes.size()];

        int i;
        // initialize all nodes
        for (i = 0; i < nodes.size(); i++) {
            // unprocessed nodes are set to the maximum integer value and their predecessor is set to null
            bellmanFordTable[i] = new BFSItem(Integer.MAX_VALUE, null, nodes.get(i).getKey());
        }

        int start_id = graph.getNodeID(startFloorTile.col + "/" + startFloorTile.row);
        // Starting the search from a given start_id. The bellmanFordTable contains the result
        searchBellmanFordAlgorithm(start_id, bellmanFordTable);


    }


    /**
     * Performs the Bellman-Ford algorithm to find the shortest paths from a given starting node to all other nodes in the graph.
     *
     * @param index            The index of the starting node in the graph.
     * @param bellmanFordTable An array of BFSItem representing the table used in the Bellman-Ford algorithm to store distances and predecessors.
     */

    private void searchBellmanFordAlgorithm(int index, BFSItem[] bellmanFordTable) {
        // Retrieve the list of nodes from the graph
        List<Node> nodes = graph.getNodes();
        int startIndex;
        int weight;
        Node node;
        ArrayList<AdjElement> adjL;
        AdjElement adjE;
        String goalKey = "";

        // Initialize distances and predecessors
        for (int i = 0; i < nodes.size(); i++) {
            bellmanFordTable[i].dist = Integer.MAX_VALUE;
            bellmanFordTable[i].pred = null;
        }

        // Set the distance of the starting node to 0
        bellmanFordTable[index].dist = 0;

        // Relaxation step for |V| - 1 iterations
        for (int i = 0; i < nodes.size() - 1; i++) {
            for (int j = 0; j < nodes.size(); j++) {
                node = nodes.get(j);
                startIndex = graph.getNodeID(node.getKey());
                adjL = node.getAdjacencies();
                Iterator<AdjElement> iter = adjL.iterator();

                while (iter.hasNext()) {
                    adjE = iter.next();
                    int subIndex = adjE.getNodeIndex();
                    weight = adjE.getWeight();

//                    if (bellmanFordTable[startIndex].dist != Integer.MAX_VALUE &&
//                            bellmanFordTable[startIndex].dist - weight < bellmanFordTable[subIndex].dist) {
//                        bellmanFordTable[subIndex].dist = bellmanFordTable[startIndex].dist - weight;
//                        bellmanFordTable[subIndex].pred = node.getKey();
//                        goalKey = node.getKey();
//                        // Mark the node as visited (setAsVisited) in your FloorTile class
//                        FloorTile visitedTile = (FloorTile) nodes.get(graph.getNodeID(goalKey)).getData();
//                        visitedTile.setAsVisited();

                    if (bellmanFordTable[startIndex].dist != Integer.MAX_VALUE &&
                            bellmanFordTable[startIndex].dist + weight < bellmanFordTable[subIndex].dist) {
                        bellmanFordTable[subIndex].dist = bellmanFordTable[startIndex].dist + weight;
                        bellmanFordTable[subIndex].pred = node.getKey();
                        goalKey = node.getKey();

                        // Mark the node as visited (setAsVisited) in your FloorTile class
                        FloorTile visitedTile = (FloorTile) nodes.get(graph.getNodeID(goalKey)).getData();
                        visitedTile.setAsVisited();
                        System.out.println(node.getData());

                    }
                }
            }
        }

        // Check for negative cycles
        for (Node value : nodes) {
            node = value;
            startIndex = graph.getNodeID(node.getKey());
            adjL = node.getAdjacencies();

            for (AdjElement adjElement : adjL) {
                adjE = adjElement;
                int subIndex = adjE.getNodeIndex();
                weight = adjE.getWeight();

                if (bellmanFordTable[startIndex].dist != Integer.MAX_VALUE &&
                        bellmanFordTable[startIndex].dist + weight < bellmanFordTable[subIndex].dist) {
                    System.out.println("Negative cycle detected involving nodes: " + node.getKey() + " and " + nodes.get(subIndex).getKey());
                    return; // Exit the method if a negative cycle is detected
                }
            }
        }

         // Backtrack through the optimal path
        currentFloorTile = goalFloorTile;
        while (currentFloorTile != null && !currentFloorTile.equals(startFloorTile)) {
            currentFloorTile.setAsPath();
            int currentIndex = getIndex(bellmanFordTable, goalKey);
            if (currentIndex == -1) {
                System.out.println("Error: Unable to find current index in bellmanFordTable.");
                return;
            }
            BFSItem currentItem = bellmanFordTable[currentIndex];

            if (currentItem == null) {
                System.out.println("Error: Null current item during backtracking.");
                return;
            }

            goalKey = currentItem.pred;

            // Mark the node as part of the optimal path
            if (goalKey != null) {
                int nodeID = graph.getNodeID(goalKey);
                if (nodeID != -1) {
                    currentFloorTile = (FloorTile) nodes.get(nodeID).getData();
                    currentFloorTile.setAsPath();
                } else {
                    System.out.println("Error: Unable to find nodeID for goalKey: " + goalKey);
                    return;
                }
            } else {
                System.out.println("Error: Null goalKey during backtracking.");
                return;
            }
        }
    }
}


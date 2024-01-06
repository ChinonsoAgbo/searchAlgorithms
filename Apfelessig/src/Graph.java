import java.util.*;
public class Graph {
    private ArrayList<Node> nodes = new ArrayList<>();
    private HashMap<String, Node> labels = new HashMap<>();

    public int addNode(String label, Object data) {
        int idx = 0;
        if (!labels.containsKey(label)) {
            idx = nodes.size();
            Node node = new Node(label, data, idx);
            nodes.add(node);
            labels.put(label, node);
        }
        return idx;
    }

    public ArrayList<Node> getNodes() {
        return nodes;
    }

    public static class AdjacencyElemLessComparator implements Comparator<AdjElement> {
        // Comparator for the PriorityQueue
        // allows the comparison between AdjElement objects by their weight
        public int
        compare(AdjElement e1, AdjElement e2) {
            if (e1.getWeight() < e2.getWeight())
                return -1;
            else if (e1.getWeight() > e2.getWeight())
                return 1;
            else return 0;
        }
    }

    public int getNodeID(String label) {
        Integer i = labels.get(label).getID();
        if (i == null) {
            throw new NoSuchElementException();
        }
        return i;
    }

    public void addEdge(String src, String dest, int cost) {
        // Picks source node from labels and adds the destination node to the adjacency list
        ArrayList<AdjElement> adjList = nodes.get(getNodeID(src)).getAdjacencies();
        adjList.add(new AdjElement(getNodeID(dest), cost));
    }
}



public class BFSItem {
    String node_label;
    int dist;
    String pred;

    public BFSItem(int d, String p, String label) {
        node_label = label;
        dist = d;
        pred = p;
    }

    public String getNode_label() {
        return node_label;
    }

    public int getDist() {
        return dist;
    }

    public String getPred() {
        return pred;
    }
}

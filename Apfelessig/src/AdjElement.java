public class AdjElement
{
    public int nodeIndex;
    public int weight;
    public AdjElement(int idx, int c) {
        nodeIndex = idx;
        weight = c;
    }
    public int getWeight(){
        return weight;
    }
    public int getNodeIndex(){
        return nodeIndex;
    }

}


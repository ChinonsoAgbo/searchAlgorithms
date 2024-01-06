import java.util.ArrayList;

public class Node
{
    private String          key;
    private Object          data;
    private int             id;
    private ArrayList<AdjElement> adjacencies;
    private String          color;

    public Node(String key, Object data, int id)
    {
        this.key  = key;
        this.data = data;
        this.id   = id;
        // list of Adjacent nodes, implementing graph construction with adjacency lists
        this.adjacencies = new ArrayList<>();
        this.setColor("WHITE");
    }
    public String getKey()  {return key;}
    public Object getData() {return data;}
    public int    getID()   {return id;}
    public ArrayList<AdjElement> getAdjacencies() {return adjacencies;}
    public String getColor() {return color;}
    public void   setColor(String color) {this.color = color;}

}
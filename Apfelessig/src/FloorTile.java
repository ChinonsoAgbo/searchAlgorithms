import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class FloorTile extends JButton implements ActionListener {

    FloorTile parent;
    int col;
    int row;
    int gCost;
    int hCost;
    int fCost;
    boolean start;
    boolean goal;
    boolean open;
    boolean solid;
    boolean water;
    boolean coin;
    boolean checked;
    boolean normal = true;

    public FloorTile(int col, int row) {
        this.setFocusable(false);
        this.col = col;
        this.row = row;
        this.setAsNormal();
        addActionListener(this);

    }
    public void setAsStart() {
        setBackground(Color.blue);
        setForeground(Color.white);
        setText("Start");
        start = true;
    }
    public void setAsGoal() {
        setBackground(Color.yellow);
        setForeground(Color.black);
        setText("Goal");
        goal = true;
    }
    public void setAsNormal() {
        setBackground(Color.white);
        setForeground(Color.black);
        normal = true;
        water = false;
        solid = false;
        coin = false;
    }
    public void setAsSolid() {
        if (!goal && !start) {
            setBackground(Color.black);
            setForeground(Color.black);
            solid = true;
            water = false;
            normal = false;
            coin = false;
        }
    }
    public void setAsWater() {
        setBackground(Color.blue);
        setForeground(Color.black);
        water = true;
        normal = false;
        solid = false;
        coin = false;
    }
    public void setAsCoin() {
        setBackground(Color.yellow);
        setForeground(Color.black);
        water = false;
        normal = false;
        solid = false;
        coin = true;
    }
    public void setAsListed() {
        open = true;
    }
    public void setAsVisited() {
        if(start == false && goal == false){
            setBackground(Color.orange);
            setForeground(Color.black);
        }
        checked = true;
    }
    public void setAsPath() {
        setBackground(Color.green);
        setForeground(Color.black);
    }
    @Override
    public void actionPerformed(ActionEvent e) {
        if (normal) {
            setAsWater();
        }
        else if (water) {
            setAsSolid();
        }
        else if (solid) {
            setAsCoin();
        }
        else {
            setAsNormal();
        }
    }
}

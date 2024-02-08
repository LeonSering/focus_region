package focusmap;

/**
 * Displays a map with some roads.
 * @author Jan-Henrik Haunert
 */
//import org.jgrapht.graph.*;

public class Main {

    /**
     * Method main is the first to be executed.
     * @param args the program arguments
     */
    public static void main(String[] args) {
        RoadNetwork rn = RoadNetwork.importFromShapefile(); 
        
        //generate a new map
        MainFrame myJFrame = new MainFrame(rn.getName(), rn);
        
        myJFrame.setVisible(true);
    }
}

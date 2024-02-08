package focusmap;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class ScaleOptimizationActivator implements ActionListener {

    MapDisplay myMapDisplay;
    public ScaleOptimizationActivator(MapDisplay md) {
        myMapDisplay = md;
    }
    
    public void actionPerformed(ActionEvent arg0) {
        myMapDisplay.resetMouseCoordinates();
        myMapDisplay.fitMapToDisplay();
        myMapDisplay.repaint();
    }

}

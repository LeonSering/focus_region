package focusmap;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JTabbedPane;

public class SaveButtonListener implements ActionListener {
    
    private JTabbedPane mapPane; 
    
    public SaveButtonListener(JTabbedPane mapPane) {
        this.mapPane = mapPane;
    }
    
    @Override
    public void actionPerformed(ActionEvent e) {
    	((MapFrame)mapPane.getSelectedComponent()).getRoadNetwork().exportToShapeFile();
    }

}

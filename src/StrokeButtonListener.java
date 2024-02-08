package focusmap;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JOptionPane;
import javax.swing.JTabbedPane;

public class StrokeButtonListener implements ActionListener {
    
    private MainFrame myMainFrame;
    private JTabbedPane mapPane; 
    
    public StrokeButtonListener(JTabbedPane tp, MainFrame mf) {
        mapPane = tp;
        myMainFrame = mf;
    }
    
    @Override
    public void actionPerformed(ActionEvent e) {
        String s = JOptionPane.showInputDialog("Maximum Tolerance:", "1.0");
        double d = Double.parseDouble(s);
        RoadNetwork rnNew = ((MapFrame) mapPane.getSelectedComponent()).getRoadNetwork().buildStrokes(d);
        myMainFrame.addMapFrame("Strokes", rnNew);        
    }

}
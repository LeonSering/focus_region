package focusmap;

import java.util.List;
import com.vividsolutions.jts.index.quadtree.Quadtree;
import com.vividsolutions.jts.geom.Envelope;

/**
 * This class represents a layer that can be displayed
 * in a MapDisplay.
 * @author Jan-Henrik Haunert
 */
public class Layer {

    /**
     * The MapObjects of this Layer.
     */
    private Quadtree myMapObjects;

    /**
     * The ID of this Layer.
     * A Layer with a large ID will be drawn on top of a layer with a small ID.
     */
    private int id;

    /**
     * Constructs a new empty Layer with a specified ID.
     * @param myID the ID
     */
    public Layer(int myID) {
        myMapObjects = new Quadtree();
        id = myID;
    }
    
    /**
     * Returns the ID of this Layer.
     * @return the id
     */
    public int getID() {
        return id;
    }
    

    /**
     * Adds a MapObject to this Layer.
     * @param m the MapObject to be added
     */
    public void add(MapObject m) {
        Envelope myEnv = m.getBoundingBox();
        myMapObjects.insert(myEnv, m);
    }

    /**
     * Returns the map objects of this layer that intersect 
     * a specified envelope.
     * @param searchEnv the query envelope
     * @return the map objects of this layer that intersect the envelope
     */
    public List<?> query(Envelope searchEnv) {
        return myMapObjects.query(searchEnv);
    }
}

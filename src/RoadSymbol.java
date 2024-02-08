package focusmap;

import java.awt.Color;

public class RoadSymbol {
    
    //standard symbols
    public static final RoadSymbol TRI_EDGE           = new RoadSymbol("triedge",   -1, 1, Color.ORANGE, new Color(150,150,200));
    public static final RoadSymbol DEFAULT            = new RoadSymbol("default",    1, 5, Color.WHITE,      Color.DARK_GRAY);
    
    //symbols for OSM data
    public static final RoadSymbol OSM_MOTORWAY       = new RoadSymbol("motorway",   6, 13, Color.GREEN,      Color.DARK_GRAY);
    public static final RoadSymbol OSM_TRUNK          = new RoadSymbol("trunk",      5, 11, Color.RED,        Color.DARK_GRAY);
    public static final RoadSymbol OSM_PRIMARY_ROAD   = new RoadSymbol("primary",    5, 11, Color.RED,        Color.DARK_GRAY);
    public static final RoadSymbol OSM_SECONDARY_ROAD = new RoadSymbol("secondary",  4, 9, Color.ORANGE,     Color.DARK_GRAY);
    public static final RoadSymbol OSM_TERTIARY_ROAD  = new RoadSymbol("tertiary",   3, 7, Color.YELLOW,     Color.DARK_GRAY);
    public static final RoadSymbol OSM_PEDESTRIAN     = new RoadSymbol("pedestrian", 2, 5, Color.LIGHT_GRAY, Color.DARK_GRAY);
    public static final RoadSymbol OSM_FOOTWAY        = new RoadSymbol("footway",    0, 2, Color.WHITE,      Color.LIGHT_GRAY);
    public static final RoadSymbol OSM_CYCLEWAY       = new RoadSymbol("cycleway",   0, 2, Color.WHITE,      Color.LIGHT_GRAY);
    public static final RoadSymbol OSM_PATH           = new RoadSymbol("path",       0, 2, Color.WHITE,      Color.LIGHT_GRAY);
    public static final RoadSymbol OSM_STEPS          = new RoadSymbol("steps",      0, 2, Color.WHITE,      Color.LIGHT_GRAY);
   
    //symbol set for OSM data
    public static final RoadSymbol[] OSM_SYMBOLS = {OSM_MOTORWAY, OSM_TRUNK, OSM_PRIMARY_ROAD, 
        OSM_SECONDARY_ROAD, OSM_TERTIARY_ROAD, OSM_TERTIARY_ROAD, OSM_PEDESTRIAN,
        OSM_FOOTWAY, OSM_CYCLEWAY, OSM_PATH, OSM_STEPS};
    
    
    //symbols for MassGIS data (Massachusetts Department of Transportation Roads)
    public static final RoadSymbol MASS_GIS_LIMITED_ACCESS_HIGHWAY = new RoadSymbol("_1_", 6,  7, Color.GREEN,  Color.DARK_GRAY);
    public static final RoadSymbol MASS_GIS_MULTI_LANE_HIGHWAY =     new RoadSymbol("_2_", 5,  7, Color.RED,    Color.DARK_GRAY);
    public static final RoadSymbol MASS_GIS_NUMBERED_ROUTE =         new RoadSymbol("_3_", 4,  7, Color.ORANGE, Color.DARK_GRAY);
    public static final RoadSymbol MASS_GIS_MAJOR_ROAD =             new RoadSymbol("_4_", 3,  7, Color.YELLOW, Color.DARK_GRAY);
    public static final RoadSymbol MASS_GIS_MINOR_ROAD1 =            new RoadSymbol("_5_", 1,  5, Color.WHITE,  Color.DARK_GRAY);
    public static final RoadSymbol MASS_GIS_MINOR_ROAD2 =            new RoadSymbol("_6_", 1,  5, Color.WHITE,  Color.DARK_GRAY);
   
    //symbol set for MassGIS data
    public static final RoadSymbol[] MASS_GIS_SYMBOLS = {MASS_GIS_LIMITED_ACCESS_HIGHWAY, MASS_GIS_MULTI_LANE_HIGHWAY,
        MASS_GIS_NUMBERED_ROUTE, MASS_GIS_MAJOR_ROAD, MASS_GIS_MINOR_ROAD1, MASS_GIS_MINOR_ROAD2};
    
   
        
    private String className; 
    private int layer;
    private int width;
    private Color color;
    private Color strokeColor;
    
    RoadSymbol(String s, int l, int w, Color c, Color sc) {
        className = s;
        layer = l;
        width = w;
        color = c;
        strokeColor = sc;
    }
    public String getClassName() {
       return className; 
    } 
    public int getLayer() {
        return layer;
    }
    public int getWidth() {
        return width;
    }
    public Color getColor() {
        return color;
    }
    public Color getStrokeColor() {
        return strokeColor;
    }
    public static RoadSymbol getSymbolForClass(RoadSymbol[] symbolSet, String className) {
        if(symbolSet == null || className == null) return DEFAULT;
        for(RoadSymbol s : symbolSet) {
            if(className.startsWith(s.className)) return s;
        }
        return DEFAULT;
    }
}

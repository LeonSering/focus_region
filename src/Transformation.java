package focusmap;
public class Transformation {
    
    private double m;
    private int ColumnOrigin, RowOrigin;
    
    
    public Transformation() {
        m = 1.0;
        ColumnOrigin = 0;
        RowOrigin = 0;
    }
    public Transformation(double m, int ColumnOrigin, int RowOrigin) {
        this.m = m;
        this.ColumnOrigin = ColumnOrigin;
        this.RowOrigin = RowOrigin;
    }
    public double getM() {
        return m;
    }
    
    public void setM(double myM) {
        m = myM;
    }
    
    public int getColumnOrigin() {
        return ColumnOrigin;
    }
    
    public void setColumnOrigin(int cOrigin) {
        ColumnOrigin = cOrigin;
    }
    
    public int getRowOrigin() {
        return RowOrigin;
    }
    
    public void setRowOrigin(int rOrigin) {
        RowOrigin = rOrigin;
    }
    
    public int getLength(double value) { // like getRow/getColume ohne Verschiebung.
    	return (int) Math.rint(m * value);
    }
    
    public int getRow(double y) {
        return RowOrigin - (int) Math.rint(m * y);
    }
    public int getColumn(double x) {
        return ColumnOrigin + (int) Math.rint(m * x);
    }
    
    public double getX(int c) {
        return (c - ColumnOrigin) / m;
    }
    
    public double getY(int r) {
        return (RowOrigin - r) / m;
    }
    
    
}
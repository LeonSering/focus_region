package focusmap;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.util.LinkedList;

public class Statistic {
	private double initTime;
	private double startTime;
	private double endTime;
	public int nodeCount;
	public int edgeCount;
	public int scaleNodeCount;
	public double scaleValue;
	
	public LinkedList<Integer> constraintCounts;
	public LinkedList<Integer> orientationCounts;
	
	public int crossingCount;
	public int orientationChangedCount;
	
	public double optimalValue;
	
	public double maxScaleValue;
	public double minScaleValue;
	
	private double timeFirstOptimization;
	private double timeLongestOptimization;
	
	public Statistic(){
		initTime = System.currentTimeMillis();
		nodeCount = 0;
		edgeCount = 0;
		scaleNodeCount = 0;
		constraintCounts = new LinkedList<Integer>();
		orientationCounts = new LinkedList<Integer>();
		optimalValue = Double.POSITIVE_INFINITY;
		scaleValue = 0.0;
		maxScaleValue = 0.0;
		minScaleValue = Double.POSITIVE_INFINITY;
		endTime = initTime;
		timeFirstOptimization = 0.0;
		timeLongestOptimization = 0.0;
	}
	public void start() {
		startTime = System.currentTimeMillis();
		endTime = startTime;
	}
	
	public void stop() {
		endTime = System.currentTimeMillis();
		double time = getTime();
		if (timeFirstOptimization == 0.0)
			timeFirstOptimization = time;
		if (time > timeLongestOptimization)
			timeLongestOptimization = time;
	}
	
	public void totalStop() {
		endTime = System.currentTimeMillis(); 
	}
	
	public double getTime() {
		if(endTime != startTime)
			return endTime - startTime;
		else
			return System.currentTimeMillis() - startTime;
	}
	
	public double getTotalTime() {
		if(endTime != startTime && endTime != initTime)
			return endTime - initTime;
		else
			return System.currentTimeMillis() - initTime;
	}
	
	public void print() {
		print(System.out);
	}
	
	public void printToFile() {
		PrintStream out;
		File f;
		f = new File("statistics.txt");
		if (!f.isFile()) {
			try {
				f.createNewFile();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			System.out.println("statistic.txt created.");
		}
		
		System.out.println("Write statistic in statistic.txt.");

		
		try {
			out = new PrintStream(f);
			print(out);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		
		
		
	}
	
	private void print(PrintStream out) {
		out.println("### STATISTICS ###");
		out.println("GRAPH: nodes: " + nodeCount + " | edges: " + edgeCount);
		out.println("PROBLEM: nodes with scale: " + scaleNodeCount + " | scale: " + scaleValue);
		int cplexSolveCount = constraintCounts.size();
		if (orientationCounts.size() > cplexSolveCount)
			cplexSolveCount = orientationCounts.size();
		out.println("SOLUTION: optimal Value: " + optimalValue + " | cplex-solve-count: " + cplexSolveCount);
		out.print("constraint count: " + getTotalConstraintCount());
		if(constraintCounts.size() > 0) {		
			out.print(" (");
			for(int i = 0; i < constraintCounts.size()-1; i++) {
				out.print(constraintCounts.get(i) + ", ");
			}
			out.println(constraintCounts.get(constraintCounts.size()-1) + ")");
		} else {
			out.println();
		}
		
		out.print("orientation count: " + getTotalOrientationCount());
		if(orientationCounts.size() > 0) {		
			out.print(" (");
			for(int i = 0; i < orientationCounts.size()-1; i++) {
				out.print(orientationCounts.get(i) + ", ");
			}
			out.println(orientationCounts.get(orientationCounts.size()-1) + ")");
		} else {
			out.println();
		}
		
		out.println("TIME: total time: " + getTotalTime() + "ms | cplex-time without intersectionconstraints: " + timeFirstOptimization + "ms");
		out.println("longest cplex-time: " + timeLongestOptimization + "ms");
		
		out.println("END: Crossings: " + crossingCount + " | Changes of Orientation: " + orientationChangedCount);
	}
	
	public void addConstraint(int num) {
		constraintCounts.add(num);
	}
	
	public void addOrientation(int num) {
		orientationCounts.add(num);
	}
	
	private int getTotalConstraintCount() {
		int sum = 0;
		for(int i : constraintCounts) {
			sum += i;
		}
		return sum;
	}
	
	private int getTotalOrientationCount() {
		int sum = 0;
		for(int i : orientationCounts) {
			sum += i;
		}
		return sum;
	}
}

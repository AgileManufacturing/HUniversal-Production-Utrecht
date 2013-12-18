package simulation;

import simulation.data.Capability;
import simulation.mas_entities.Grid;

public class Main {

	public final static String pathToEquipletLayoutCsv = "/home/t/sim/equipletLayout.csv";
	public final static String pathToCapabilitiesCsv = "/home/t/sim/capabilities.csv";
	
	public static Simulation simulation;
	public static Grid grid;
	
	
	public static void main(String[] args) {
		Capability.loadCapabilities(pathToCapabilitiesCsv);
		
		simulation = new Simulation();
		
		grid = new Grid(pathToEquipletLayoutCsv);
		simulation.addUpdateable(grid);
		
		simulation.resumeSimulation();
		
		System.out.println("Hurray, we survived!");
	}
}

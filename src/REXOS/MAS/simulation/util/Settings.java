package MAS.simulation.util;

public class Settings {

	/*
	 * VERBOSITY gives the level of simulation output
	 * 0: none output is given while running the simulation
	 * 1: important simulation debug is printed to the console
	 * 2: the gui is used for information during the simulation
	 * 3: the gui is used in combination with the debug information in the console
	 * 4: all above with addition of scheduling information
	 */
	public static int VERBOSITY = 3;
	public final static boolean SCHEDULING_MATIX = true;
	public final static String TRAFFIC_AGENT = "traffic-controller";
	public final static long COMMUNICATION_TIMEOUT = 30000;
	public final static String SIMULATION_CONFIG = "simulation/simulation.xml";
	public final static String SIMULATION_EQUIPLET_CONFIG = "simulation/equiplets.csv";
	public final static String SIMULATION_OUTPUT = "simulation/output";

}

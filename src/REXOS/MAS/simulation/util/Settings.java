package MAS.simulation.util;

import MAS.util.Tick;

public class Settings {

	/**
	 * whether the simulation need to verification the simulation while running
	 * this has major consequence on the running time of the simulation 
	 */
	public static boolean VERIFICATION = false;
	
	/**
	 * whether the simulation uses stochastic processing times and other time consuming variables.
	 */
	public final static boolean STOCHASTICS = false;

	/**
	 * whether equiplets can breakdown
	 */
	public final static boolean BREAKDOWNS = false;

	/**
	 * time penalty of a reconfiguration. the time it takes to (re) config an equiplet
	 */
	public static double RECONFIGATION_TIME = 300;

	/**
	 * time needs to elapse before checking if equiplets need to be reconfigured
	 */
	public static final Tick RECONFIG_CHECK = new Tick(3000);
	
	/**
	 * the maximun number of jobs in an equiplet queue, when there is no scheduling
	 */
	public static final int QUEUE_CAPACITY = 40;

	/**
	 * Warm-up period after which in multiple simulation runs the products statistics are being averaged
	 */
	public final static Tick WARMUP = new Tick(3000);

	/**
	 * Target of utilization of equiplet for depending the interarrival time of new products
	 */
	public static final double UTILIZATION = 0.80;

	public static final boolean CONSTANT_NUMBER_OF_PRODUCT_STEPS = true;

	public static final int MEAN_PRODUCT_STEPS = 20;

	public static final int MIN_PRODUCT_STEPS = 10;

	/**
	 * input and output locations of the simulation variables
	 */
	public static String SIMULATION_CONFIG = "simulation/simulation.xml";
	public static String SIMULATION_EQUIPLET_CONFIG = "simulation/equiplets.csv";
	public static String SIMULATION_OUTPUT = "simulation/output";

	public static final String PRODUCT_LOG = "products-log";
}

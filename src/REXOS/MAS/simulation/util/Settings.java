package MAS.simulation.util;

public class Settings {

	/**
	 * VERBOSITY gives the level of simulation output
	 * 0: none output is given while running the simulation
	 * 1: important simulation debug is printed to the console
	 * 2: the gui is used for information during the simulation
	 * 3: the gui is used in combination with the debug information in the console
	 * 4: all above with addition of scheduling information
	 */
	public static int VERBOSITY = 0;

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
	public static final double RECONFIGATION_TIME = 0;
	
	/**
	 * time needs to elapse before checking if equiplets need to be reconfigured
	 */
	public static final Tick RECONFIG_CHECK = new Tick(10000);

	/**
	 * the number of jobs an just arrives job can overtake in the queue
	 */
	public static final int QUEUE_JUMP = 0;

	/**
	 * the maximun number of jobs in an equiplet queue, when there is no scheduling
	 */
	public static final int QUEUE_CAPACITY = 40;

	/**
	 * Scheduling algorithm used by product agent to schedule his product step
	 */
	public final static SchedulingAlgorithm SCHEDULING = SchedulingAlgorithm.MATRIX;

	/**
	 * Reshedule when start time of product step is not met
	 */
	public static boolean RESCHEDULE = true;
	/**
	 * Warm-up period after which in multiple simulation runs the products statistics are being averaged
	 */
	public final static Tick WARMUP = new Tick(3000);

	/**
	 * Target of utilization of equiplet for depending the interarrival time of new products
	 */
	public static final double UTILIZATION = 0.80;

	public static final int MEAN_PRODUCT_STEPS = 20;
	public static final int MIN_PRODUCT_STEPS = 10;

	/**
	 * input and output locations of the simulation variables
	 */
	public final static String SIMULATION_CONFIG = "simulation/simulation.xml";
	public final static String SIMULATION_EQUIPLET_CONFIG = "simulation/equiplets.csv";
	public final static String SIMULATION_OUTPUT = "simulation/output";

	/**
	 * name of the traffic controller agent
	 */
	public final static String TRAFFIC_AGENT = "traffic-controller";

	/**
	 * Communication time out, the time an agent wait when he is expecting a communication message until he continues
	 */
	public final static long COMMUNICATION_TIMEOUT = 5000;

	public static final String PRODUCT_LOG = "products-log";

}

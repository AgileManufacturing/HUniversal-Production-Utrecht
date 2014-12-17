package MAS.util;

public class MASConfiguration {

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
	 * the number of jobs an just arrives job can overtake in the queue
	 */
	public static final int QUEUE_JUMP = 0;

	/**
	 * Scheduling algorithm used by product agent to schedule his product step
	 */
	public final static SchedulingAlgorithm SCHEDULING = SchedulingAlgorithm.MATRIX;

	/**
	 * Reshedule when start time of product step is not met
	 */
	public static boolean RESCHEDULE = true;

	/**
	 * keep track of the equiplet history while the equiplet agent is alive
	 * this could lead to heavy memory use
	 */
	public final static boolean KEEP_FULL_EQUIPLET_HISORY = false;
	/**
	 * name of the traffic controller agent
	 */
	public final static String TRAFFIC_AGENT = "traffic-controller";

	/**
	 * Communication time out, the time an agent wait when he is expecting a communication message until he continues
	 */
	public final static long COMMUNICATION_TIMEOUT = 5000;

}

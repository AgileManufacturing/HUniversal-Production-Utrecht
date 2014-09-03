package simulation.simulation;

import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.wrapper.AgentController;
import jade.wrapper.ContainerController;
import jade.wrapper.ControllerException;
import jade.wrapper.StaleProxyException;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.TreeSet;

import simulation.config.IConfig;
import simulation.config.Config;
import simulation.graphics.Control;
import simulation.graphics.SimInterface;
import simulation.mas.equiplet.Capability;
import simulation.mas.equiplet.EquipletSimAgent;
import simulation.mas.equiplet.EquipletState;
import simulation.mas.equiplet.IEquipletSim;
import simulation.mas.product.IProductSim;
import simulation.mas.product.IProductSim;
import simulation.mas.product.ProductAgentSim;
import simulation.mas.product.ProductStep;
import simulation.util.Pair;
import simulation.util.Position;
import simulation.util.Triple;
import simulation.util.Tuple;

public class SimulationAgentx extends Agent implements Control, ISimulation {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	// product statistics
	private static final String STATS_TRAVEL = "Traveling";
	private static final String STATS_WAITING = "Waiting";
	private static final String STATS_BUSY = "Processing";
	private static final String STATS_FINISHED = "Finished";
	private static final String STATS_FAILED = "Failed";
	private static final String STATS_FAILED_CREATION = "Failed to create";
	private static final String STATS_SYSTEM = "In System";
	private static final String STATS_BROKEN = "Broken";

	private SimInterface gui;
	private IConfig config;
	private Stochastics stochastics;

	private int run;
	private int runs;
	private double run_length;

	private boolean finished;
	private boolean running;
	private int step;
	private int delay;

	protected final Object lock = new Object();
	private boolean eventReady;

	// State
	private TreeSet<Event> eventStack;
	private double time;

	private TreeMap<String, IEquipletSim> equiplets;
	private Map<String, IProductSim> products;
	private int productCount;

	// Performance
	private int totalSteps;

	// private int traveling;
	private HashMap<String, Double> throughput;
	private Map<String, TreeMap<Double, Double>> productStatistics;

	// TODO

	/**
	 * Setup the simulation agnet
	 */
	public void setup() {
		delay = 1; // if delay = 0, waits forever

		config = Config.read();
		stochastics = new Stochastics(config);
		gui = SimInterface.create(this);

		System.out.printf("Simulation: setup %s.\n", config);

		init();
	}

	/**
	 * initialize the simulation
	 */
	private void init() {
		eventStack = new TreeSet<>();

		run = 0;
		runs = config.getRuns();
		run_length = config.getRunLength();
		finished = false;
		running = false;
		step = 0;

		time = 0;

		totalSteps = 0;
		// traveling = 0;
		productCount = 0;
		throughput = new HashMap<String, Double>();
		productStatistics = new HashMap<String, TreeMap<Double, Double>>();

		TreeMap<Double, Double> initStats = new TreeMap<Double, Double>();
		initStats.put(time, 0d);

		productStatistics.put(STATS_TRAVEL, new TreeMap<Double, Double>(initStats));
		productStatistics.put(STATS_WAITING, new TreeMap<Double, Double>(initStats));
		productStatistics.put(STATS_BUSY, new TreeMap<Double, Double>(initStats));
		productStatistics.put(STATS_FINISHED, new TreeMap<Double, Double>(initStats));
		productStatistics.put(STATS_FAILED, new TreeMap<Double, Double>(initStats));
		productStatistics.put(STATS_FAILED_CREATION, new TreeMap<Double, Double>(initStats));
		productStatistics.put(STATS_SYSTEM, new TreeMap<Double, Double>(initStats));
		productStatistics.put(STATS_BROKEN, new TreeMap<Double, Double>(initStats));

		products = new HashMap<>();
		equiplets = new TreeMap<>();

		// initialize the equiplets in the grid
		for (Entry<String, Pair<Position, List<Capability>>> entry : config.getEquipletsConfigurations().entrySet()) {
			String equipletName = entry.getKey();
			Position position = entry.getValue().first;
			List<Capability> capabilities = entry.getValue().second;

			// start equiplet agent
			try {
				// Create and start the agent
				// Place the properties in an Object array to sent them to the new agent
				EquipletSimAgent equiplet = new EquipletSimAgent(position, capabilities);

				ContainerController cc = getContainerController();
				AgentController ac = cc.acceptNewAgent(equipletName, equiplet);
				ac.start();

				// equiplets.put(equipletName, new AID(equipletName, AID.ISLOCALNAME));
				equiplets.put(equipletName, equiplet);
			} catch (StaleProxyException e) {
				System.err.printf("Simulation: ERROR: equiplet agent %s creation was not possible.\n", equipletName);
				e.printStackTrace();
			} catch (NullPointerException e) {
				System.err.println("Simulation: not yet ready.");
				e.printStackTrace();
			}

			double breakdown = stochastics.generateBreakdownTime(entry.getKey());
			eventStack.add(new Event(time + breakdown, EventType.BREAKDOWN, equipletName));
		}

		eventReady = true;
		eventStack.add(new Event(time, EventType.PRODUCT));
		eventStack.add(new Event(run_length, EventType.DONE));

		addBehaviour(new SimulationBehaviour());
	}

	/**
	 * Simulation behaviour of the agent
	 */
	public class SimulationBehaviour extends Behaviour {
		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		@Override
		public void action() {
			while (!finished) {
				while (running || step > 0) {
					if (step > 0) {
						step--;
					}

					// wait if needed to continue with the next event
					synchronized (lock) {
						System.out.println("WAIT ON LOCK. " + eventReady);
						while (!eventReady) {
							try {
								lock.wait();
							} catch (InterruptedException e) {
								System.err.println("Simulation: waiting on lock InterruptedException");
							}
						}
					}

					Event e = eventStack.pollFirst();
					time = e.getTime();

					System.out.println("\n-----\nSimulation: event=" + e + " : " + eventStack);

					// handle the event
					switch (e.getType()) {
					case PRODUCT:
						productEvent();
						break;
					case ARRIVED:
						arrivedEvent(e.getProduct(), e.getEquiplet());
						break;
					case FINISHED:
						finishedEvent(e.getEquiplet());
						break;
					case BREAKDOWN:
						breakdownEvent(e.getEquiplet());
						break;
					case REPAIRED:
						repairedEvent(e.getEquiplet());
						break;
					case DONE:
						doneEvent();
					default:
						break;
					}

					update(e);

					// double busy = productStatistics.get(STATS_BUSY).lastEntry().getValue();
					// double waiting = productStatistics.get(STATS_WAITING).lastEntry().getValue();
					// double travel = productStatistics.get(STATS_TRAVEL).lastEntry().getValue();
					// double failed = productStatistics.get(STATS_FAILED).lastEntry().getValue();
					// double finished = productStatistics.get(STATS_FINISHED).lastEntry().getValue();
					// double system = productStatistics.get(STATS_SYSTEM).lastEntry().getValue();
					//
					// System.out.printf("\nSimulation: stats=[busy=%s, waiting=%.0f, travel=%.0f, failed=%.0f, finished=%.0f, in system=%.0f]\n\n",
					// productStatistics.get(STATS_BUSY).lastEntry(), waiting, travel, failed, finished, system);
					// System.out.printf("\nSTATS %s\n\n", productStatistics);

					/*
					 * System.out.println("\nSimulation state: " + Util.formatArray(equiplets)); for (Entry<String, IEquipletSim> eq : equiplets.entrySet()) {
					 * System.out.println("\nSimulation schedule " + eq.getKey() + " : " + eq.getValue().getSchedule()); }
					 */

					// System.out.println("\nSimulation products: " + formatArray(products) + "\n");

					// validate();

					// wait a certain time
					if (delay > 0) {
						// when delay is 0, the agent will wait eternal
						doWait(delay);
					}
				}

				synchronized (this) {
					doWait(1000); // wait until notify gets called in startThread
				}
			}
			saveStatistics();
			run++;
		}

		@Override
		public boolean done() {
			return finished;
		}
	}

	/**
	 * Validate the event stack to verify the simulation
	 */
	@SuppressWarnings("unused")
	private void validate() {
		ArrayList<String> finishedEquiplets = new ArrayList<>();
		for (Event event : eventStack) {
			if (event.getType() == EventType.FINISHED) {
				if (finishedEquiplets.contains(event.getEquiplet())) {
					throw new IllegalArgumentException("Equiplet two times in event list: " + event);
				} else {
					finishedEquiplets.add(event.getEquiplet());
				}
			}
		}
	}

	@Override
	public synchronized void step() {
		step++;
	}

	@Override
	public synchronized void start() {
		running = true;
	}

	@Override
	public synchronized void pause() {
		System.out.println("Simulation: " + (running ? "start" : "pause"));

		running = !running;
		notify();// wake up the wait

	}

	@Override
	public synchronized int getDelay() {
		return delay;
	}

	@Override
	public synchronized void setDelay(int delay) {
		this.delay = Math.max(1, delay);
	}

	@Override
	public synchronized void saveStatistics() {
		// TODO Auto-generated method stub

	}

	@Override
	public Map<String, List<Triple<String, Double, Double>>> getCompleteSchedule() {
		TreeMap<String, List<Triple<String, Double, Double>>> schedules = new TreeMap<String, List<Triple<String, Double, Double>>>();
		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			schedules.put(entry.getKey(), entry.getValue().getCompleteSchedule());
		}
		return schedules;
	}

	@Override
	public Map<String, List<Triple<String, Double, Double>>> getEquipletSchedule() {
		TreeMap<String, List<Triple<String, Double, Double>>> schedules = new TreeMap<String, List<Triple<String, Double, Double>>>();
		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			schedules.put(entry.getKey(), entry.getValue().getSchedule());
		}
		return schedules;
	}

	@Override
	public Map<String, List<Triple<String, Double, Double>>> getEquipletHistory() {
		TreeMap<String, List<Triple<String, Double, Double>>> histories = new TreeMap<String, List<Triple<String, Double, Double>>>();
		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			histories.put(entry.getKey(), entry.getValue().getHistory());
		}
		return histories;
	}

	@Override
	public Map<String, Triple<Double, Double, Double>> getEquipletUtilization() {
		TreeMap<String, Triple<Double, Double, Double>> data = new TreeMap<String, Triple<Double, Double, Double>>();
		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			data.put(entry.getKey(), entry.getValue().getStatistics(time));
		}
		return data;
	}

	@Override
	public Map<String, Map<Double, Double>> getEquipletLatency() {
		TreeMap<String, Map<Double, Double>> data = new TreeMap<String, Map<Double, Double>>();
		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			data.put(entry.getKey(), entry.getValue().getLatency());
		}
		return data;
	}

	@Override
	public Map<String, Map<Double, Double>> getProductStatistics() {
		HashMap<String, Map<Double, Double>> stats = new HashMap<String, Map<Double, Double>>(productStatistics);
		stats.remove(STATS_FINISHED);
		return stats;
	}

	@Override
	public Map<String, Map<Double, Double>> getEquipletStatistics() {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * Update the product statistics
	 * 
	 * @param type
	 *            of the statistic
	 * @param add
	 *            the addition of the statistic
	 */
	private void updateStats(String type, double add) {
		double lastValue = productStatistics.get(type).lastEntry().getValue();
		productStatistics.get(type).put(time, lastValue + add);
	}

	/**
	 * Change if the simulation is ready to continue
	 * Lock the simulation thread if the event is not handled completely i.e. waiting to be informed by an agent
	 * 
	 * @param ready
	 *            if the simulation is ready to continue
	 */
	private void changeReady(boolean ready) {
		System.out.println("change ready " + eventReady + " to " + ready);
		synchronized (lock) {
			if (ready) {
				eventReady = true;
				lock.notifyAll();
			} else {
				eventReady = false;
			}
		}
	}

	/**
	 * Procedure for the updating the interface
	 * 
	 * @param e
	 *            the event
	 */
	private void update(Event e) {
		// equiplet states = List of Tuple < name, position, services, Tuple < state, waiting, scheduled,executed > >
		List<Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>>> equipletStates = new ArrayList<>();
		for (Entry<String, IEquipletSim> equiplet : equiplets.entrySet()) {
			equipletStates.add(equiplet.getValue().getUpdateState());
		}

		double waitingTime = 0;
		List<Double> busy = new ArrayList<Double>();
		double throughput = 0;
		gui.update(time, e.getEquiplet() + ":" + e.getType(), products.size(), productCount, totalSteps, productStatistics.get(STATS_TRAVEL).lastEntry().getValue().intValue(), equipletStates, waitingTime, busy, throughput);
	}

	/**
	 * Event that signals the arrival of a new product in the system
	 * A product agent is created and started which will invoke the schedule behaviour
	 */
	private void productEvent() {
		// product agent settings
		LinkedList<ProductStep> productSteps = stochastics.generateProductSteps();

		String productName = "P" + productCount++;
		Position startPosition = new Position(-1, -1);

		try {
			ProductAgentSim productAgent = new ProductAgentSim(this, productSteps, startPosition, time);
			products.put(productName, productAgent);

			ContainerController cc = getContainerController();
			AgentController ac = cc.acceptNewAgent(productName, productAgent);
			ac.start();

			// update statistics
			totalSteps += productSteps.size();
			updateStats(STATS_SYSTEM, +1);

		} catch (StaleProxyException e1) {
			System.err.printf("Simulation: ERROR: product agent %s creation was not possible.\n", productName);
			e1.printStackTrace();
		}

		// wait for confirmation creation of product agent
		changeReady(false);
	}

	/**
	 * Event that signals the arrival of a product by an equiplet
	 *
	 * @param productName
	 *            name of the product agent
	 * @param equipletName
	 *            name of the equiplet agent
	 */
	private void arrivedEvent(String productName, String equipletName) {
		updateStats(STATS_TRAVEL, -1);
		updateStats(STATS_WAITING, +1);

		IProductSim productAgent = products.get(productName);
		productAgent.onProductArrived(time);
	}

	/**
	 * Event that signals that an equiplet should have finished with the current job
	 * 
	 * @param equipletName
	 *            name of the equiplet
	 */
	private void finishedEvent(String equipletName) {
		IEquipletSim equipletAgent = equiplets.get(equipletName);
		System.out.printf("Simulation: equiplet finished: %s \n", equipletAgent);

		if (equipletAgent.getEquipletState() == EquipletState.ERROR) {
			// equiplet broken down, wait until equiplet is repaired to continue with the remaining time
			equipletAgent.notifyJobFinished(time);
		} else if (equipletAgent.getEquipletState() == EquipletState.ERROR_REPAIRED) {
			equipletAgent.notifyJobFinished(time);

			double remainingTime = equipletAgent.getRemainingTime();
			eventStack.add(new Event(time + remainingTime, EventType.FINISHED, equipletName));
			System.out.printf("Simulation: reschedule event FINISHED after breakdown and equiplet is repaired %.0f + %.0f, %s\n", time, remainingTime, equipletName);
		} else {
			updateStats(STATS_BUSY, -1);

			// notify the equiplet his job is finished (without any more delay)
			equipletAgent.notifyJobFinished(time);
		}

		System.out.println("CHECKPOINT FOXTROT");
	}

	/**
	 * Event that signals the breakdown of an equiplet
	 * After informing the equiplet, a repair event is scheduled
	 * 
	 * @param equipletName
	 *            name of the equiplet
	 */
	private void breakdownEvent(String equipletName) {
		updateStats(STATS_BROKEN, +1);
		IEquipletSim equipletAgent = equiplets.get(equipletName);
		equipletAgent.notifyBreakdown(time);

		// schedele REPAIRED time + repairTime, equiplet
		double repairTime = stochastics.generateRepairTime(equipletName);
		eventStack.add(new Event(time + repairTime, EventType.REPAIRED, equipletName));
		System.out.printf("Simulation: schedule event REPAIRED %.0f + %.0f, %s\n", time, repairTime, equipletName);
	}

	/**
	 * Event that signals that an equiplet is repaired
	 * If the equiplet is finished during the repairing of the equiplet, a finished event is scheduled
	 * An event that the equiplet will breakdown again is scheduled
	 * 
	 * @param equipletName
	 *            name of the equiplet
	 */
	private void repairedEvent(String equipletName) {
		updateStats(STATS_BROKEN, -1);

		IEquipletSim equipletAgent = equiplets.get(equipletName);
		if (equipletAgent.getEquipletState() == EquipletState.ERROR_FINISHED) {
			equipletAgent.notifyRepaired(time);

			double remainingTime = equipletAgent.getRemainingTime();
			eventStack.add(new Event(time + remainingTime, EventType.FINISHED, equipletName));
			System.out.printf("Simulation: reschedule event FINISHED after breakdown of equiplet %s over %.0f + %.0f\n", equipletName, time, remainingTime);
		} else if (equipletAgent.getEquipletState() == EquipletState.ERROR_READY) {
			// there become a job ready during the time the equiplet was broken
			// this is handled by notify repaired, so the equiplet communicates with the product which let the simulation know
			equipletAgent.notifyRepaired(time);
		} else {
			equipletAgent.notifyRepaired(time);
		}

		// schedule BREAKDOWN time + breakdown, equiplet
		double breakdown = stochastics.generateBreakdownTime(equipletName);
		eventStack.add(new Event(time + breakdown, EventType.BREAKDOWN, equipletName));
		System.out.printf("Simulation: schedule event BREAKDOWN after repair %.0f + %.0f, %s\n", time, breakdown, equipletAgent);
	}

	/**
	 * Event that signals the end of the simulation
	 */
	private void doneEvent() {
		System.out.println("Simulation: simulation finished");
		running = false;
		finished = true;
	}

	/**
	 * Product notifies that the creation of the product failed
	 * 
	 * @param productName
	 *            name of the product
	 */
	@Override
	public void notifyProductCreationFailed(String productName) {
		System.out.printf("Simulation: product agent %s failed to create.\n", productName);
		updateStats(STATS_FAILED_CREATION, +1);
		updateStats(STATS_SYSTEM, -1);

		// remove agent from system
		products.remove(productName);

		ContainerController cc = getContainerController();
		try {
			AgentController agent = cc.getAgent(productName);
			agent.kill();
		} catch (ControllerException e) {
			e.printStackTrace();
		}

		// schedule next product arrival
		double arrivalTime = stochastics.generateProductArrival();
		eventStack.add(new Event(time + arrivalTime, EventType.PRODUCT));
		System.out.printf("Simulation: schedule event PRODUCT %.0f + %.0f\n", time, arrivalTime);

		// continue with simulation
		changeReady(true);
		System.out.println("CHECKPOINT GOLF");
	}

	/**
	 * Product notifies that the product is successful created
	 * The product will travel to the first equiplet, i.e. an arrive event is scheduled
	 * Further a new product event is scheduled
	 * 
	 * @param productName
	 *            name of the product
	 * @param equipletName
	 *            name of the equiplet where to the product to will travel
	 */
	@Override
	public void notifyProductCreated(String productName, String equipletName) {
		System.out.printf("Simulation: product agent %s created success and traveling to equiplet %s\n", productName, equipletName);

		IProductSim productAgent = products.get(productName);
		Position startPosition = productAgent.getPosition();

		IEquipletSim equipletAgent = equiplets.get(equipletName);
		Position nextPosition = equipletAgent.getPosition();

		// schedule ARRIVED time + travelTime, equiplet, product
		int travelSquares = Math.abs(startPosition.getX() - nextPosition.getX()) + Math.abs(startPosition.getY() - nextPosition.getY());
		double travelTime = stochastics.generateTravelTime(travelSquares);

		eventStack.add(new Event(time + travelTime, EventType.ARRIVED, productName, equipletName));
		System.out.printf("Simulation: schedule event ARRIVED %.0f + %.0f, %s, %s\n", time, travelTime, productName, productAgent.getPosition(), equipletName, equipletAgent.getPosition());

		// traveling++;
		updateStats(STATS_TRAVEL, +1);

		// schedule next product arrival
		double arrivalTime = stochastics.generateProductArrival();
		eventStack.add(new Event(time + arrivalTime, EventType.PRODUCT));
		System.out.printf("Simulation: schedule event PRODUCT %.0f + %.0f\n", time, arrivalTime);

		// continue with simulation
		changeReady(true);
		System.out.println("CHECKPOINT ALPHA");
	}

	/**
	 * Product notifies that an equiplet has informed him that he is started to execute a job for him
	 * A finished event is scheduled
	 * 
	 * @param productName
	 *            name of the product
	 * @param equipletName
	 *            name of the equiplet which is started with processing the job
	 * @param service
	 *            name of the service that the equiplet is started
	 */
	@Override
	public void notifyProductProcessing(String productName, String equipletName, String service) {
		updateStats(STATS_WAITING, -1);
		updateStats(STATS_BUSY, +1);

		System.out.printf("Simulation: product agent %s notifies processing.\n", productName);
		double productionTime = stochastics.generateProductionTime(equipletName, service);

		// schedule FINISHED time + productionTime, equiplet
		eventStack.add(new Event(time + productionTime, EventType.FINISHED, equipletName));
		System.out.printf("Simulation: schedule event FINISHED %.0f + %.0f, %s, %s\n", time, productionTime, productName, equipletName);

		// unblock simulation when notifying job finished
		// changeReady(true);
	}

	/**
	 * Product starts to travel to a new location
	 * An arrive event is scheduled
	 * 
	 * @param productName
	 *            name of the product
	 * @param equipletName
	 *            name of the equiplet to which the product is traveling
	 */
	@Override
	public void notifyProductTraveling(String productName, String equipletName) {
		updateStats(STATS_TRAVEL, +1);

		System.out.printf("Simulation: product agent %s notifies product step is finished and traveling to equiplet %s\n", productName, equipletName);

		IProductSim productAgent = products.get(productName);
		Position startPosition = productAgent.getPosition();

		IEquipletSim equipletAgent = equiplets.get(equipletName);
		Position nextPosition = equipletAgent.getPosition();

		// schedule ARRIVED time + travelTime, equiplet, product
		int travelSquares = Math.abs(startPosition.getX() - nextPosition.getX()) + Math.abs(startPosition.getY() - nextPosition.getY());
		double travelTime = stochastics.generateTravelTime(travelSquares);

		eventStack.add(new Event(time + travelTime, EventType.ARRIVED, productName, equipletName));
		System.out.printf("Simulation: schedule event ARRIVED %.0f + %.0f, %s, %s\n", time, travelTime, productName, equipletName);

		// unblock simulation when notifying job finished
		// changeReady(true);
	}

	/**
	 * The product is finished with executing all his product steps
	 * 
	 * @param productName
	 *            name of the product
	 */
	@Override
	public void notifyProductFinished(String productName) {
		System.out.printf("Simulation: product agent %s notifies he is finished.\n", productName);

		updateStats(STATS_FINISHED, +1);
		updateStats(STATS_SYSTEM, -1);

		// Product is finished
		IProductSim productAgent = products.get(productName);
		products.remove(productName);
		throughput.put(productName, time - productAgent.getCreated());
	}
}

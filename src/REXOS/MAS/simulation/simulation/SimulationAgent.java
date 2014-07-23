package simulation.simulation;

import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.wrapper.AgentController;
import jade.wrapper.ContainerController;
import jade.wrapper.StaleProxyException;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.TreeSet;

import simulation.config.Config;
import simulation.graphics.Control;
import simulation.graphics.SimInterface;
import simulation.mas.equiplet.EquipletAgent;
import simulation.mas.equiplet.EquipletState;
import simulation.mas.product.ProductAgent;
import simulation.mas.product.ProductStep;
import simulation.util.Capability;
import simulation.util.Position;
import simulation.util.Triple;
import simulation.util.Tuple;

public class SimulationAgent extends Agent implements Control, ISimulation {

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
	private static final String STATS_SYSTEM = "In System";

	private SimInterface gui;
	private Config config;
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

	private TreeMap<String, EquipletAgent> equiplets;
	private Map<String, ProductAgent> products;
	private int productCount;

	// Performance
	private int totalSteps;

	// private int traveling;
	private HashMap<String, Double> throughput;
	private Map<String, TreeMap<Double, Double>> productStatistics;

	// TODO

	public void setup() {
		delay = 1; // if delay = 0, waits forever

		config = Config.read();
		stochastics = new Stochastics(config);
		gui = SimInterface.create(this);

		System.out.printf("Simulation: setup %s.\n", config);

		init();
	}

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
		productStatistics.put(STATS_SYSTEM, new TreeMap<Double, Double>(initStats));

		products = new HashMap<>();
		equiplets = new TreeMap<>();

		// initialize the equiplets in the grid
		for (Entry<String, Triple<Position, List<Capability>, Map<String, Double>>> entry : config.getEquipletsConfigurations().entrySet()) {
			String equipletName = entry.getKey();
			Position position = entry.getValue().first;
			List<Capability> capabilities = entry.getValue().second;
			Map<String, Double> productionTimes = entry.getValue().third;

			// start equiplet agent
			try {
				// Create and start the agent
				// Place the properties in an Object array to sent them to the new agent
				EquipletAgent equiplet = new EquipletAgent(position, capabilities, productionTimes);

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

					update();

//					double busy = productStatistics.get(STATS_BUSY).lastEntry().getValue();
//					double waiting = productStatistics.get(STATS_WAITING).lastEntry().getValue();
//					double travel = productStatistics.get(STATS_TRAVEL).lastEntry().getValue();
//					double failed = productStatistics.get(STATS_FAILED).lastEntry().getValue();
//					double finished = productStatistics.get(STATS_FINISHED).lastEntry().getValue();
//					double system = productStatistics.get(STATS_SYSTEM).lastEntry().getValue();
//
//					System.out.printf("\nSimulation: stats=[busy=%s, waiting=%.0f, travel=%.0f, failed=%.0f, finished=%.0f, in system=%.0f]\n\n", productStatistics.get(STATS_BUSY).lastEntry(), waiting, travel, failed, finished, system);
//					System.out.printf("\nSTATS %s\n\n", productStatistics);

					// System.out.println("\nSimulation state: " + formatArray(grid.getEquiplets()));
					// System.out.println("\nSimulation products: " + formatArray(products) + "\n");

					// validate();

					doWait(delay);
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
		for (Entry<String, EquipletAgent> entry : equiplets.entrySet()) {
			schedules.put(entry.getKey(), entry.getValue().getCompleteSchedule());
		}
		return schedules;
	}

	@Override
	public Map<String, List<Triple<String, Double, Double>>> getEquipletSchedule() {
		TreeMap<String, List<Triple<String, Double, Double>>> schedules = new TreeMap<String, List<Triple<String, Double, Double>>>();
		for (Entry<String, EquipletAgent> entry : equiplets.entrySet()) {
			schedules.put(entry.getKey(), entry.getValue().getSchedule());
		}
		return schedules;
	}

	@Override
	public Map<String, List<Triple<String, Double, Double>>> getEquipletHistory() {
		TreeMap<String, List<Triple<String, Double, Double>>> histories = new TreeMap<String, List<Triple<String, Double, Double>>>();
		for (Entry<String, EquipletAgent> entry : equiplets.entrySet()) {
			histories.put(entry.getKey(), entry.getValue().getHistory());
		}
		return histories;
	}

	@Override
	public Map<String, Triple<Double, Double, Double>> getEquipletUtilization() {
		TreeMap<String, Triple<Double, Double, Double>> data = new TreeMap<String, Triple<Double, Double, Double>>();
		for (Entry<String, EquipletAgent> entry : equiplets.entrySet()) {
			data.put(entry.getKey(), entry.getValue().getStatistics(time));
		}
		return data;
	}

	@Override
	public Map<String, Map<Double, Double>> getEquipletLatency() {
		TreeMap<String, Map<Double, Double>> data = new TreeMap<String, Map<Double, Double>>();
		for (Entry<String, EquipletAgent> entry : equiplets.entrySet()) {
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

	private void updateStats(String type, double add) {
		double lastValue = productStatistics.get(type).lastEntry().getValue();
		// System.out.println(" UPDATE " + type + " : " + lastValue + " + " + add + " = " + (lastValue + add));
		productStatistics.get(type).put(time, lastValue + add);
	}

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

	private void update() {
		// equiplet states = List of Tuple < name, position, services, Tuple < state, waiting, scheduled,executed > >
		List<Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>>> equipletStates = new ArrayList<>();
		for (Entry<String, EquipletAgent> equiplet : equiplets.entrySet()) {
			equipletStates.add(equiplet.getValue().getUpdateState());
		}

		double waitingTime = 0;
		List<Double> busy = new ArrayList<Double>();
		double throughput = 0;
		gui.update(time, products.size(), productCount, totalSteps, productStatistics.get(STATS_TRAVEL).lastEntry().getValue().intValue(), equipletStates, waitingTime, busy, throughput);
	}

	private void productEvent() {
		// product agent settings
		LinkedList<ProductStep> productSteps = stochastics.generateProductSteps();

		String productName = "P" + productCount++;
		Position startPosition = new Position(-1, -1);

		try {
			ProductAgent productAgent = new ProductAgent(this, productSteps, startPosition, time);
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
		System.out.println("CHECKPOINT ECHO");
	}

	private void arrivedEvent(String productName, String equipletName) {
		// traveling--;
		updateStats(STATS_TRAVEL, -1);
		updateStats(STATS_WAITING, +1);

		ProductAgent productAgent = products.get(productName);
		productAgent.notifyProductArrived(time);
		// wait until an equiplet notify continue
		// changeReady(false);
		System.out.println("CHECKPOINT DELTA");
	}

	private void finishedEvent(String equipletName) {
		EquipletAgent equipletAgent = equiplets.get(equipletName);
		System.out.printf("Simulation: equiplet finished: %s \n", equipletAgent);

		if (equipletAgent.getEquipletState() == EquipletState.ERROR) {
			// equiplet broken down, wait until equiplet is repaired to continue with the remaining time
			equipletAgent.notifyJobFinished(time);
		} else if (equipletAgent.getEquipletState() == EquipletState.ERROR_REPAIRED) {
			double remainingTime = equipletAgent.getRemainingTime();
			eventStack.add(new Event(time + remainingTime, EventType.FINISHED, equipletName));
			System.out.printf("Simulation: reschedule event FINISHED after breakdown and equiplet is repaired %.0f + %.0f, %s\n", time, remainingTime, equipletName);

			equipletAgent.notifyJobFinished(time);
		} else {
			updateStats(STATS_BUSY, -1);

			// notify the equiplet his job is finished (without any more delay)
			equipletAgent.notifyJobFinished(time);
			// changeReady(false);
		}

		System.out.println("CHECKPOINT FOXTROT");
	}

	private void breakdownEvent(String equipletName) {

	}

	private void repairedEvent(String equipletName) {

	}

	private void doneEvent() {
		System.out.println("Simulation: simulation finished");
		running = false;
		finished = true;
	}

	@Override
	public void notifyProductCreated(boolean succeeded, String productName, String equipletName) {
		System.out.printf("Simulation: product agent %s created success=%b traveling to equiplet %s\n", productName, succeeded, equipletName);

		if (succeeded) {
			ProductAgent productAgent = products.get(productName);
			Position startPosition = productAgent.getPosition();

			EquipletAgent equipletAgent = equiplets.get(equipletName);
			Position nextPosition = equipletAgent.getPosition();

			// schedule ARRIVED time + travelTime, equiplet, product
			int travelSquares = Math.abs(startPosition.getX() - nextPosition.getX()) + Math.abs(startPosition.getY() - nextPosition.getY());
			double travelTime = stochastics.generateTravelTime(travelSquares);

			eventStack.add(new Event(time + travelTime, EventType.ARRIVED, productName, equipletName));
			System.out.printf("Simulation: schedule event ARRIVED %.0f + %.0f, %s, %s\n", time, travelTime, productName, productAgent.getPosition(), equipletName, equipletAgent.getPosition());

			// traveling++;
			updateStats(STATS_TRAVEL, +1);
		} else {
			// TODO statistics update
			updateStats(STATS_FAILED, +1);
		}

		// schedule next product arrival
		double arrivalTime = stochastics.generateProductArrival();
		eventStack.add(new Event(time + arrivalTime, EventType.PRODUCT));
		System.out.printf("Simulation: schedule event PRODUCT %.0f + %.0f\n", time, arrivalTime);

		
		// continue with simulation
		changeReady(true);
		System.out.println("CHECKPOINT ALPHA");
	}

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

	@Override
	public void notifyProductTraveling(String productName, String equipletName) {
		updateStats(STATS_TRAVEL, +1);

		System.out.printf("Simulation: product agent %s notifies product step is finished and traveling to equiplet %s\n", productName, equipletName);

		ProductAgent productAgent = products.get(productName);
		Position startPosition = productAgent.getPosition();

		EquipletAgent equipletAgent = equiplets.get(equipletName);
		Position nextPosition = equipletAgent.getPosition();

		// schedule ARRIVED time + travelTime, equiplet, product
		int travelSquares = Math.abs(startPosition.getX() - nextPosition.getX()) + Math.abs(startPosition.getY() - nextPosition.getY());
		double travelTime = stochastics.generateTravelTime(travelSquares);

		eventStack.add(new Event(time + travelTime, EventType.ARRIVED, productName, equipletName));
		System.out.printf("Simulation: schedule event ARRIVED %.0f + %.0f, %s, %s\n", time, travelTime, productName, equipletName);

		// unblock simulation when notifying job finished
		// changeReady(true);
	}

	@Override
	public void notifyProductFinished(String productName) {
		System.out.printf("Simulation: product agent %s notifies he is finished.\n", productName);

		updateStats(STATS_FINISHED, +1);
		updateStats(STATS_SYSTEM, -1);

		// Product is finished
		ProductAgent productAgent = products.get(productName);
		products.remove(productName);
		throughput.put(productName, time - productAgent.getCreated());
	}
}

package MAS.simulation.simulation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.TreeSet;

import MAS.simulation.config.Config;
import MAS.simulation.config.Configuration;
import MAS.simulation.config.Configuration.ConfigException;
import MAS.simulation.config.IConfig;
import MAS.simulation.graphics.IControl;
import MAS.simulation.graphics.SimInterface;
import MAS.simulation.mas.equiplet.Capability;
import MAS.simulation.mas.equiplet.EquipletState;
import MAS.simulation.mas.equiplet.IEquipletSim;
import MAS.simulation.mas.product.IProductSim;
import MAS.simulation.mas.product.ProductStep;
import MAS.simulation.util.Lock;
import MAS.simulation.util.Pair;
import MAS.simulation.util.Position;
import MAS.simulation.util.Tick;
import MAS.simulation.util.Triple;
import MAS.simulation.util.Tuple;

public class Simulation implements ISimulation, IControl {

	// product statistics
	private static final String STATS_TRAVEL = "Traveling";
	private static final String STATS_WAITING = "Waiting";
	private static final String STATS_BUSY = "Processing";
	private static final String STATS_FINISHED = "Finished";
	private static final String STATS_FAILED = "Failed";
	private static final String STATS_FAILED_CREATION = "Failed to create";
	private static final String STATS_SYSTEM = "In System";
	private static final String STATS_BROKEN = "Broken";

	// equiplet statistics
	// private static final String STATS_SCHEDULED = "Scheduled";
	// private static final String STATS_QUEUED = "Queued";
	// private static final String STATS_EXECUTED = "Executed";
	// private static final String STATS_STATE = "State";
	private static final String STATS_LOAD = "Load";
	private static final String STATS_LOAD_2 = "Load small window";
	private static final String STATS_LOAD_HISTORY = "Load history";
	private static final String STATS_LOAD_HISTORY_2 = "Load histor 2y";
	private static final Tick LOAD_WINDOW = new Tick(1000);

	private ISimControl simulation;
	private SimInterface gui;
	private IConfig config;
	private Stochastics stochastics;

	private Lock lock = new Lock();

	private int run;
	private int runs;
	private Tick run_length;

	private boolean finished;
	private boolean running;
	private int step;
	private int delay;

	// State
	private volatile TreeSet<Event> eventStack;
	private Tick time;

	private TreeMap<String, IEquipletSim> equiplets;
	private Map<String, IProductSim> products;
	private int productCount;

	// Performance
	private int totalSteps;

	// private int traveling;
	private HashMap<String, Tick> throughput;
	private Map<String, TreeMap<Tick, Double>> productStatistics;
	private Map<String, TreeMap<Tick, Double>> equipletStatistics;

	public Simulation(ISimControl simulation) {
		this.simulation = simulation;

		delay = 0;

		finished = false;
		running = false;
		step = 0;

		try {
			Config con = Config.read();
			config = Configuration.read(con);

			System.out.println("Simulation: configuration: " + config);

			stochastics = new Stochastics(config);
			gui = SimInterface.create(this);

			init();

		} catch (ConfigException e) {
			System.err.println("Configuration failed " + e.getMessage());
		}
	}

	/**
	 * initialize the simulation
	 */
	private void init() {
		eventStack = new TreeSet<>();

		run = 0;
		runs = config.getRuns();
		run_length = config.getRunLength();

		time = new Tick(0);

		totalSteps = 0;
		// traveling = 0;
		productCount = 0;
		throughput = new HashMap<String, Tick>();
		productStatistics = new HashMap<String, TreeMap<Tick, Double>>();
		equipletStatistics = new HashMap<String, TreeMap<Tick, Double>>();

		TreeMap<Tick, Double> initStats = new TreeMap<Tick, Double>();
		initStats.put(time, 0d);

		productStatistics.put(STATS_TRAVEL, new TreeMap<Tick, Double>(initStats));
		productStatistics.put(STATS_WAITING, new TreeMap<Tick, Double>(initStats));
		productStatistics.put(STATS_BUSY, new TreeMap<Tick, Double>(initStats));
		productStatistics.put(STATS_FINISHED, new TreeMap<Tick, Double>(initStats));
		productStatistics.put(STATS_FAILED, new TreeMap<Tick, Double>(initStats));
		productStatistics.put(STATS_FAILED_CREATION, new TreeMap<Tick, Double>(initStats));
		productStatistics.put(STATS_SYSTEM, new TreeMap<Tick, Double>(initStats));
		productStatistics.put(STATS_BROKEN, new TreeMap<Tick, Double>(initStats));

		// equipletStatistics.put(STATS_SCHEDULED, new TreeMap<Tick, Double>(initStats));
		// equipletStatistics.put(STATS_QUEUED, new TreeMap<Tick, Double>(initStats));
		// equipletStatistics.put(STATS_EXECUTED, new TreeMap<Tick, Double>(initStats));
		// equipletStatistics.put(STATS_STATE, new TreeMap<Tick, Double>(initStats));
		equipletStatistics.put(STATS_LOAD, new TreeMap<Tick, Double>(initStats));
		equipletStatistics.put(STATS_LOAD_2, new TreeMap<Tick, Double>(initStats));
		equipletStatistics.put(STATS_LOAD_HISTORY, new TreeMap<Tick, Double>(initStats));
		equipletStatistics.put(STATS_LOAD_HISTORY_2, new TreeMap<Tick, Double>(initStats));

		products = new HashMap<>();
		equiplets = new TreeMap<>();
		Map<String, Position> equipletPositions = new HashMap<String, Position>();

		// initialize the equiplets in the grid
		for (Entry<String, Pair<Position, List<Capability>>> entry : config.getEquipletsConfigurations().entrySet()) {
			String equipletName = entry.getKey();
			Position position = entry.getValue().first;
			List<Capability> capabilities = entry.getValue().second;

			equipletPositions.put(equipletName, position);

			// start equiplet agent
			try {
				IEquipletSim equiplet = simulation.createEquiplet(equipletName, position, capabilities);
				equiplets.put(equipletName, equiplet);
			} catch (Exception e) {
				e.printStackTrace();
			}

			Tick breakdown = stochastics.generateBreakdownTime(entry.getKey());
			// eventStack.add(new Event(time + breakdown, EventType.BREAKDOWN, equipletName));
		}

		try {
			simulation.createTrafficAgent(equipletPositions);
		} catch (Exception e) {
			e.printStackTrace();
		}

		eventStack.add(new Event(time, EventType.PRODUCT));
		eventStack.add(new Event(run_length, EventType.DONE));
	}

	public void handleEvent() {
		try {
			while (running || step > 0) {
				if (step > 0) {
					step--;
				}

				// wait if needed to continue with the next event
				System.out.println("Simulation: lock();");
				lock.lock();
				System.out.println("Simulation: continue");

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
					break;
				default:
					break;
				}

				lock.await();

				calculateEquipletLoad();
				update(e);

				for (Entry<String, IEquipletSim> equiplet : equiplets.entrySet()) {
					// System.out.println("EQ: " + equiplet.getValue());
				}

				// print the product statistics
				// double busy = productStatistics.get(STATS_BUSY).lastEntry().getValue();
				// double waiting = productStatistics.get(STATS_WAITING).lastEntry().getValue();
				// double travel = productStatistics.get(STATS_TRAVEL).lastEntry().getValue();
				// double failed = productStatistics.get(STATS_FAILED).lastEntry().getValue();
				// double finished = productStatistics.get(STATS_FINISHED).lastEntry().getValue();
				// double system = productStatistics.get(STATS_SYSTEM).lastEntry().getValue();
				//
				// System.out.printf("\nSimulation: stats=[busy=%s, waiting=%s, travel=%s, failed=%s, finished=%s, in system=%s]\n\n",
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
					simulation.delay(delay);
				}
			}
			simulation.delay(1000);
		} catch (InterruptedException ie) {
			ie.printStackTrace();
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

	/**
	 * Update the product statistics
	 * 
	 * @param type
	 *            of the statistic
	 * @param add
	 *            the addition of the statistic
	 */
	private void updateProductStats(String type, double add) {
		double lastValue = productStatistics.get(type).lastEntry().getValue();
		productStatistics.get(type).put(time, lastValue + add);
	}

	/**
	 * calculate load,
	 * TODO throw not an illegal argument exception
	 */
	private void calculateEquipletLoad() {
		double sumLoad = 0.0;
		double sumLoad2 = 0.0;
		double sumLoadH = 0.0;
		double sumLoadH2 = 0.0;

		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			double load = entry.getValue().load(time, LOAD_WINDOW);
			System.out.println("Simulation: " + entry.getKey() + " load=" + load);
			if (load > 1 || load < 0) {
				System.out.println(" ERROR: time=" + time + " - " + LOAD_WINDOW + ", load=" + load + " : " + entry.getValue());
				throw new IllegalArgumentException(" ERROR: time=" + time + ", window=" + LOAD_WINDOW + ", load=" + load + " : " + entry.getValue());
			}
			sumLoad += (1 - load);

			// second load with smaller window
			double load2 = entry.getValue().load(time, new Tick(100));
			if (load2 > 1 || load2 < 0) {
				System.out.println(" ERROR: time=" + time + " - " + 100 + ", load=" + load2 + " : " + entry.getValue());
				throw new IllegalArgumentException(" ERROR: time=" + time + ", window=" + 100 + ", load=" + load2 + " : " + entry.getValue());
			}
			sumLoad2 += (1 - load2);

			// load history
			double loadH = entry.getValue().loadHistory(time.minus(100), new Tick(100));
			if (loadH > 1 || loadH < 0) {
				System.out.println(" ERROR: time=" + time + " - " + 100 + ", load=" + loadH + " : " + entry.getValue());
				throw new IllegalArgumentException(" ERROR: time=" + time + ", window=" + 100 + ", load=" + loadH + " : " + entry.getValue());
			}
			sumLoadH += (1 - loadH);

			// load history a second time
			double loadH2 = entry.getValue().loadHistory(time.minus(LOAD_WINDOW), LOAD_WINDOW);
			if (loadH2 > 1 || loadH2 < 0) {
				System.out.println(" ERROR: time=" + time + " - " + LOAD_WINDOW + ", load=" + loadH2 + " : " + entry.getValue());
				throw new IllegalArgumentException(" ERROR: time=" + time + ", window=" + LOAD_WINDOW + ", load=" + loadH2 + " : " + entry.getValue());
			}
			sumLoadH2 += (1 - loadH2);
		}

		double l = sumLoad / equiplets.size();
		System.out.println("sum load " + sumLoad + " / " + equiplets.size() + " = " + l);
		equipletStatistics.get(STATS_LOAD).put(time, l);

		double l2 = sumLoad2 / equiplets.size();
		equipletStatistics.get(STATS_LOAD_2).put(time, l2);

		double lH = sumLoadH / equiplets.size();
		System.out.println("sum load history " + sumLoadH + " / " + equiplets.size() + " = " + lH);
		equipletStatistics.get(STATS_LOAD_HISTORY).put(time, lH);

		double lH2 = sumLoadH2 / equiplets.size();
		System.out.println("sum load history 2" + sumLoadH2 + " / " + equiplets.size() + " = " + lH2);
		equipletStatistics.get(STATS_LOAD_HISTORY_2).put(time, lH2);
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

		double sumThroughput = 0.0;
		for (Entry<String, Tick> entry : throughput.entrySet()) {
			sumThroughput += entry.getValue().doubleValue();
		}
		double avgThroughput = sumThroughput / throughput.size();

		gui.update(time, e.getEquiplet() + ":" + e.getType(), products.size(), productCount, totalSteps, productStatistics.get(STATS_TRAVEL).lastEntry().getValue().intValue(), equipletStates, avgThroughput);
	}

	/**
	 * Event that signals the arrival of a new product in the system
	 * A product agent is created and started which will invoke the schedule behaviour
	 */
	private void productEvent() {
		try {
			// product agent settings
			LinkedList<ProductStep> productSteps = stochastics.generateProductSteps();

			String productName = "P" + productCount++;
			Position startPosition = new Position(-1, -1);

			IProductSim product = simulation.createProduct(productName, startPosition, productSteps, time);
			products.put(productName, product);

			// update statistics
			totalSteps += productSteps.size();
			updateProductStats(STATS_SYSTEM, +1);
		} catch (Exception e) {
			e.printStackTrace();
		}

		// wait for confirmation creation of product agent
		// changeReady(false);
		System.out.println("CHECKPOINT BETA");
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
		updateProductStats(STATS_TRAVEL, -1);
		updateProductStats(STATS_WAITING, +1);

		IProductSim productAgent = products.get(productName);
		productAgent.onProductArrived(time);

		synchronized (this) {
			lock.unlock();
		}
		System.out.println("CHECKPOINT CHARLIE");
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

			Tick remainingTime = equipletAgent.getRemainingTime();
			eventStack.add(new Event(time.add(remainingTime), EventType.FINISHED, equipletName));
			System.out.printf("Simulation: reschedule event FINISHED after breakdown and equiplet is repaired %s + %s, %s\n", time, remainingTime, equipletName);
		} else {
			updateProductStats(STATS_BUSY, -1);

			// notify the equiplet his job is finished (without any more delay)
			equipletAgent.notifyJobFinished(time);
		}

		synchronized (this) {
			lock.unlock();
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
		updateProductStats(STATS_BROKEN, +1);
		IEquipletSim equipletAgent = equiplets.get(equipletName);
		equipletAgent.notifyBreakdown(time);

		// schedele REPAIRED time + repairTime, equiplet
		Tick repairTime = stochastics.generateRepairTime(equipletName);
		eventStack.add(new Event(time.add(repairTime), EventType.REPAIRED, equipletName));
		System.out.printf("Simulation: schedule event REPAIRED %s + %s, %s\n", time, repairTime, equipletName);

		synchronized (this) {
			lock.unlock();
		}
		System.out.println("CHECKPOINT JULIETT");
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
		updateProductStats(STATS_BROKEN, -1);

		IEquipletSim equipletAgent = equiplets.get(equipletName);
		if (equipletAgent.getEquipletState() == EquipletState.ERROR_FINISHED) {
			equipletAgent.notifyRepaired(time);

			Tick remainingTime = equipletAgent.getRemainingTime();
			eventStack.add(new Event(time.add(remainingTime), EventType.FINISHED, equipletName));
			System.out.printf("Simulation: reschedule event FINISHED after breakdown of equiplet %s over %s + %s\n", equipletName, time, remainingTime);
		} else if (equipletAgent.getEquipletState() == EquipletState.ERROR_READY) {
			// there become a job ready during the time the equiplet was broken
			// this is handled by notify repaired, so the equiplet communicates with the product which let the simulation know
			equipletAgent.notifyRepaired(time);
		} else {
			equipletAgent.notifyRepaired(time);
		}

		// schedule BREAKDOWN time + breakdown, equiplet
		Tick breakdown = stochastics.generateBreakdownTime(equipletName);
		eventStack.add(new Event(time.add(breakdown), EventType.BREAKDOWN, equipletName));
		System.out.printf("Simulation: schedule event BREAKDOWN after repair %s + %s, %s\n", time, breakdown, equipletAgent);

		synchronized (this) {
			lock.unlock();
		}
		System.out.println("CHECKPOINT KILO");
	}

	/**
	 * Event that signals the end of the simulation
	 */
	private void doneEvent() {
		System.out.println("Simulation: simulation finished");
		running = false;
		finished = true;

		synchronized (this) {
			lock.unlock();
		}
		System.out.println("CHECKPOINT LIMA");
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
		updateProductStats(STATS_FAILED_CREATION, +1);
		updateProductStats(STATS_SYSTEM, -1);

		// remove agent from system
		products.remove(productName);

		simulation.killAgent(productName);

		// schedule next product arrival
		Tick arrivalTime = stochastics.generateProductArrival();
		eventStack.add(new Event(time.add(arrivalTime), EventType.PRODUCT));
		System.out.printf("Simulation: schedule event PRODUCT %s + %s\n", time, arrivalTime);

		// continue with simulation
		// changeReady(true);
		synchronized (this) {
			lock.unlock();
		}
		System.out.println("CHECKPOINT GOLF");

		simulation.delay(10000);
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
		Tick travelTime = stochastics.generateTravelTime(travelSquares);

		eventStack.add(new Event(time.add(travelTime), EventType.ARRIVED, productName, equipletName));
		System.out.printf("Simulation: schedule event ARRIVED %s + %s, %s, %s\n", time, travelTime, productName, productAgent.getPosition(), equipletName, equipletAgent.getPosition());

		// traveling++;
		updateProductStats(STATS_TRAVEL, +1);

		// schedule next product arrival
		Tick arrivalTime = stochastics.generateProductArrival();
		eventStack.add(new Event(time.add(arrivalTime), EventType.PRODUCT));
		System.out.printf("Simulation: schedule event PRODUCT %s + %s\n", time, arrivalTime);

		// continue with simulation
		// changeReady(true);
		synchronized (this) {
			lock.unlock();
		}
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
		updateProductStats(STATS_WAITING, -1);
		updateProductStats(STATS_BUSY, +1);

		System.out.printf("Simulation: product agent %s notifies processing.\n", productName);
		Tick productionTime = stochastics.generateProductionTime(equipletName, service);

		// schedule FINISHED time + productionTime, equiplet
		eventStack.add(new Event(time.add(productionTime), EventType.FINISHED, equipletName));
		System.out.printf("Simulation: schedule event FINISHED %s + %s, %s, %s\n", time, productionTime, productName, equipletName);

		// unblock simulation when notifying job finished
		// changeReady(true);
		System.out.println("CHECKPOINT ECHO");
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
		updateProductStats(STATS_TRAVEL, +1);

		System.out.printf("Simulation: product agent %s notifies product step is finished and traveling to equiplet %s\n", productName, equipletName);

		IProductSim productAgent = products.get(productName);
		Position startPosition = productAgent.getPosition();

		IEquipletSim equipletAgent = equiplets.get(equipletName);
		Position nextPosition = equipletAgent.getPosition();

		// schedule ARRIVED time + travelTime, equiplet, product
		int travelSquares = Math.abs(startPosition.getX() - nextPosition.getX()) + Math.abs(startPosition.getY() - nextPosition.getY());
		Tick travelTime = stochastics.generateTravelTime(travelSquares);

		eventStack.add(new Event(time.add(travelTime), EventType.ARRIVED, productName, equipletName));
		System.out.printf("Simulation: schedule event ARRIVED %s + %s, %s, %s\n", time, travelTime, productName, equipletName);

		// unblock simulation when notifying job finished
		// changeReady(true);
		System.out.println("CHECKPOINT HOTEL");
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

		updateProductStats(STATS_FINISHED, +1);
		updateProductStats(STATS_SYSTEM, -1);

		// Product is finished
		IProductSim productAgent = products.get(productName);
		products.remove(productName);
		throughput.put(productName, time.minus(productAgent.getCreated()));
		System.out.println("CHECKPOINT INDIA");
	}

	public boolean isFinished() {
		return finished;
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
		// notify();// wake up the wait

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
	public Map<String, List<Triple<String, Tick, Tick>>> getCompleteSchedule() {
		TreeMap<String, List<Triple<String, Tick, Tick>>> schedules = new TreeMap<String, List<Triple<String, Tick, Tick>>>();
		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			schedules.put(entry.getKey(), entry.getValue().getCompleteSchedule());
		}
		return schedules;
	}

	@Override
	public Map<String, List<Triple<String, Tick, Tick>>> getEquipletSchedule() {
		TreeMap<String, List<Triple<String, Tick, Tick>>> schedules = new TreeMap<String, List<Triple<String, Tick, Tick>>>();
		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			schedules.put(entry.getKey(), entry.getValue().getSchedule());
		}
		return schedules;
	}

	@Override
	public Map<String, List<Triple<String, Tick, Tick>>> getEquipletHistory() {
		TreeMap<String, List<Triple<String, Tick, Tick>>> histories = new TreeMap<String, List<Triple<String, Tick, Tick>>>();
		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			histories.put(entry.getKey(), entry.getValue().getHistory());
		}
		return histories;
	}

	@Override
	public TreeMap<String, Triple<? extends Number, ? extends Number, ? extends Number>> getEquipletUtilization() {
		TreeMap<String, Triple<? extends Number, ? extends Number, ? extends Number>> data = new TreeMap<String, Triple<? extends Number, ? extends Number, ? extends Number>>();
		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			data.put(entry.getKey(), entry.getValue().getStatistics(time));
		}
		return data;
	}

	@Override
	public Map<String, Map<Tick, Tick>> getEquipletLatency() {
		TreeMap<String, Map<Tick, Tick>> data = new TreeMap<String, Map<Tick, Tick>>();
		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			data.put(entry.getKey(), entry.getValue().getLatency());
		}
		return data;
	}

	@Override
	public Map<String, Map<Tick, Double>> getProductStatistics() {
		Map<String, Map<Tick, Double>> stats = new HashMap<String, Map<Tick, Double>>(productStatistics);
		// / stats.remove(STATS_FINISHED);
		return stats;
	}

	@Override
	public Map<String, Map<Tick, Double>> getEquipletStatistics() {
		return new HashMap<String, Map<Tick, Double>>(equipletStatistics);
	}
}
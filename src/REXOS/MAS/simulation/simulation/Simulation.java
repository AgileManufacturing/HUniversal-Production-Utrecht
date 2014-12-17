package MAS.simulation.simulation;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import MAS.equiplet.Capability;
import MAS.equiplet.EquipletState;
import MAS.product.ProductStep;
import MAS.simulation.config.Config;
import MAS.simulation.config.Configuration;
import MAS.simulation.config.Configuration.ConfigException;
import MAS.simulation.config.IConfig;
import MAS.simulation.graphics.Chart;
import MAS.simulation.graphics.IControl;
import MAS.simulation.graphics.SimInterface;
import MAS.simulation.mas.equiplet.IEquipletSim;
import MAS.simulation.mas.product.IProductSim;
import MAS.simulation.util.Lock;
import MAS.simulation.util.Settings;
import MAS.util.MASConfiguration;
import MAS.util.Pair;
import MAS.util.Position;
import MAS.util.Tick;
import MAS.util.Triple;
import MAS.util.Tuple;
import MAS.util.Util;

public class Simulation implements ISimulation, IControl {

	// product statistics
	private static final String STATS_TRAVEL = "Traveling";
	private static final String STATS_WAITING = "Waiting";
	private static final String STATS_BUSY = "Processing";
	private static final String STATS_FINISHED = "Finished";
	private static final String STATS_RESCHEDULED = "Rescheduled";
	private static final String STATS_FAILED_CREATION = "Failed to schedule";
	private static final String STATS_FAILED_DEADLINE = "Failed to meet the deadline";
	private static final String STATS_SYSTEM = "In system";
	private static final String STATS_PHYSICAL = "Physically in system";
	private static final String STATS_BROKEN = "Broken";

	// equiplet statistics
	private static final String STATS_LOAD_AVG = "Load Average";
	private static final String STATS_LOAD_AVG_HISTORY = "Load History Average";
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

	private Tick timeReconfig;
	private double reconfigThreshold;
	// private boolean reconfiguration;
	private Tuple<String, String, Tick, Pair<List<Capability>, List<Capability>>> reconfiguring;
	// reconfigured :: equiplet, < time of reconfig start, reconfig finished, with new capabilities of equiplet >
	private Map<String, Tuple<String, Tick, Tick, String>> reconfigured;

	// Performance
	private int totalSteps;

	private HashMap<Tick, Tick> productionTimes;
	private Map<String, TreeMap<Tick, Integer>> productStatistics;
	private Map<String, TreeMap<Tick, Float>> equipletLoads;
	private Map<String, TreeMap<Tick, Float>> equipletLoadHistory;
	private Map<String, TreeMap<String, String>> debugInfo;

	// statistics per run with <created products,completed, failed, production times, equiplet load>
	private String outputFolder;
	private Map<Integer, Pair<Tuple<Integer, Integer, Integer, Float>, Float>> runStats;

	public Simulation(ISimControl simulation) {
		this.simulation = simulation;

		delay = 0;

		finished = false;
		running = false;
		step = 0;
		run = 1;
		runStats = new HashMap<Integer, Pair<Tuple<Integer, Integer, Integer, Float>, Float>>();

		try {
			Config con = Config.read();
			config = Configuration.read(con);

			System.out.println("Simulation: configuration: " + config);

			stochastics = new Stochastics(config);
			if (MASConfiguration.VERBOSITY > 1) {
				gui = SimInterface.create(this);
			}

			runs = config.getRuns();

		} catch (NullPointerException | ConfigException e) {
			System.err.println("Configuration failed " + e.getMessage());
			simulation.takeDown();
		}
	}

	/**
	 * initialize the simulation
	 */
	protected void init() {
		eventStack = new TreeSet<>();

		run_length = config.getRunLength();

		time = new Tick(0);

		totalSteps = 0;
		productCount = 0;
		productionTimes = new HashMap<Tick, Tick>();
		productStatistics = new HashMap<String, TreeMap<Tick, Integer>>();
		equipletLoads = new HashMap<String, TreeMap<Tick, Float>>();
		equipletLoadHistory = new HashMap<String, TreeMap<Tick, Float>>();

		TreeMap<Tick, Integer> initStats = new TreeMap<Tick, Integer>();
		initStats.put(time, 0);

		productStatistics.put(STATS_TRAVEL, new TreeMap<Tick, Integer>(initStats));
		productStatistics.put(STATS_WAITING, new TreeMap<Tick, Integer>(initStats));
		productStatistics.put(STATS_BUSY, new TreeMap<Tick, Integer>(initStats));
		productStatistics.put(STATS_FINISHED, new TreeMap<Tick, Integer>(initStats));
		productStatistics.put(STATS_RESCHEDULED, new TreeMap<Tick, Integer>(initStats));
		productStatistics.put(STATS_FAILED_CREATION, new TreeMap<Tick, Integer>(initStats));
		productStatistics.put(STATS_FAILED_DEADLINE, new TreeMap<Tick, Integer>(initStats));
		productStatistics.put(STATS_SYSTEM, new TreeMap<Tick, Integer>(initStats));
		productStatistics.put(STATS_PHYSICAL, new TreeMap<Tick, Integer>(initStats));
		productStatistics.put(STATS_BROKEN, new TreeMap<Tick, Integer>(initStats));

		equipletLoads.put(STATS_LOAD_AVG, new TreeMap<Tick, Float>());
		equipletLoadHistory.put(STATS_LOAD_AVG_HISTORY, new TreeMap<Tick, Float>());
		debugInfo = new HashMap<>();

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

				equipletLoads.put(equipletName, new TreeMap<Tick, Float>());
				equipletLoadHistory.put(equipletName, new TreeMap<Tick, Float>());

			} catch (Exception e) {
				e.printStackTrace();
			}

			if (Settings.BREAKDOWNS) {
				Tick breakdown = stochastics.generateBreakdownTime(entry.getKey());
				eventStack.add(new Event(time.add(breakdown), EventType.BREAKDOWN, equipletName));
			}
		}

		try {
			simulation.createTrafficAgent(equipletPositions);
		} catch (Exception e) {
			e.printStackTrace();
		}

		// reconfiguration variables
		timeReconfig = new Tick(0);
		reconfigThreshold = 1.10;

		// reconfiguration = false;
		reconfiguring = null;
		reconfigured = new HashMap<>();

		eventStack.add(new Event(time, EventType.PRODUCT));
		eventStack.add(new Event(run_length, EventType.DONE));
		//
		// simulation.delay(2000);
		//
		// IEquipletSim e1= equiplets.get("E1");
		// IEquipletSim e2= equiplets.get("E2");
		// IEquipletSim e7= equiplets.get("E7");
		// IEquipletSim e8= equiplets.get("E8");
		// IEquipletSim e9= equiplets.get("E9");
		//
		// e7.reconfigureFinished(e9.getCapabilities());
		// e8.reconfigureFinished(e9.getCapabilities());
		// e9.reconfigureFinished(e1.getCapabilities());
		//
		// reconfigured.put("E7", new Tuple<String, Tick, Tick, String>("E9", new Tick(0), time, "E7" + "-> " +"E9"));
		// reconfigured.put("E8", new Tuple<String, Tick, Tick, String>("E9", new Tick(0), time, "E8" + "-> " +"E9"));
		// reconfigured.put("E9", new Tuple<String, Tick, Tick, String>("E1", new Tick(0), time, "E9" + "-> " +"E1"));
	}

	/**
	 * handle an simulation event
	 */
	public void handleEvent() {
		try {
			while (running || step > 0) {
				if (step > 0) {
					step--;
				}

				if (Math.round(time.doubleValue() % 10000) == 0) {
					DateFormat df = DateFormat.getDateTimeInstance(DateFormat.MEDIUM, DateFormat.MEDIUM, new Locale("en", "EN"));
					System.err.println("Simulation: [run=" + run + ", time=" + time + " : " + Math.round(time.div(run_length).doubleValue() * 100) + "%] " + df.format(new Date()));
				}

				// wait if needed to continue with the next event
				// System.out.println("Simulation: lock();");
				lock.lock();
				// System.out.println("Simulation: continue");

				Event e = eventStack.pollFirst();

				if (time.greaterThan(e.getTime())) {
					throw new IllegalArgumentException("Damn time=" + time + " == " + e);
				}
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
					finishedEvent(e.getEquiplet(), e.getProduct());
					break;
				case BREAKDOWN:
					breakdownEvent(e.getEquiplet());
					break;
				case REPAIRED:
					repairedEvent(e.getEquiplet());
					break;
				case STARTED:
					startedEvent(e.getProduct(), e.getIndex());
					break;
				case RECONFIG:
					reconfigEvent(e.getEquiplet());
					break;
				case DONE:
					doneEvent();
					break;
				default:
					break;
				}

				lock.await();

				if (Settings.RECONFIGATION_TIME > 0) {
					reconfiguration();
				}
				calculateEquipletLoad();

				if (MASConfiguration.VERBOSITY > 1) {
					update(e);
				}

				for (Entry<String, IEquipletSim> equiplet : equiplets.entrySet()) {
					System.out.println("EQ: " + equiplet.getValue().toString()
							+ (equiplet.getValue().getSchedule().size() > 0 ? " " + equiplet.getValue().getSchedule().get(0) : ""));
				}

				if (Settings.VERIFICATION) {
					verification();
				}

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
	 * The function could be used to programmatically verify the simulation
	 */
	private void verification() {

		int busy = 0;
		for (Entry<String, IEquipletSim> equiplet : equiplets.entrySet()) {
			if (equiplet.getValue().getEquipletState() == EquipletState.BUSY
					|| (equiplet.getValue().getEquipletState() == EquipletState.RECONFIG && equiplet.getValue().isExecuting())) {
				busy++;
			}
		}

		if (busy != productStatistics.get(STATS_BUSY).lastEntry().getValue()) {
			System.out.println("BUSY:  " + Util.formatArray(productStatistics.get(STATS_BUSY)));
			throw new IllegalArgumentException("DAMN!! busy: " + busy + " == " + productStatistics.get(STATS_BUSY).lastEntry().getValue());
		}

		if (products.size() != productStatistics.get(STATS_SYSTEM).lastEntry().getValue()) {
			System.out.println("BUSY:  " + Util.formatArray(productStatistics.get(STATS_SYSTEM)));
			throw new IllegalArgumentException("DAMN!! products " + products.size() + " == " + productStatistics.get(STATS_SYSTEM).lastEntry().getValue());
		}

		// if an equiplet is busy, there need to be a finished event in the event stack
		List<String> busyEquiplets = new ArrayList<String>();

		// equiplets that are in error repaired, but never continuing with a the executing job
		List<String> equipletNames = new ArrayList<String>(equiplets.keySet());

		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {

			// equiplet busy validation
			if (entry.getValue().getEquipletState() == EquipletState.BUSY) {
				busyEquiplets.add(entry.getKey());
			}

			// error repaired validation
			if (entry.getValue().isExecuting() && entry.getValue().getEquipletState() == EquipletState.IDLE) {
				throw new IllegalArgumentException("Equiplet can not be idle and executing [eq=" + entry.getKey() + ", \n\nequiplets=" + equiplets + ", \n\nschedule="
						+ entry.getValue().getCompleteSchedule() + ", \n\nhistory=" + entry.getValue().getHistory() + "]");
			}

			if (entry.getValue().getEquipletState() == EquipletState.ERROR_REPAIRED) {
				boolean found = false;
				for (Event event : eventStack) {
					if (event.getType() == EventType.FINISHED && event.getEquiplet().equals(entry.getKey())) {
						found = true;
						break;
					}
				}
				if (!found) {
					throw new IllegalArgumentException("Equiplet error [eq=" + entry.getKey() + ", equiplets=" + equiplets + ", events=" + eventStack + "]");
				}
			}
		}

		// equiplet busy validation
		// error repaired validation
		for (Event event : eventStack) {
			if (event.getType() == EventType.FINISHED) {
				// remove equiplet, equiplet do not need to be in list as the equiplet can be broken down
				busyEquiplets.remove(event.getEquiplet());
			} else if (event.getType() == EventType.BREAKDOWN || event.getType() == EventType.REPAIRED) {
				if (equipletNames.contains(event.getEquiplet())) {
					equipletNames.remove(event.getEquiplet());
				} else {
					throw new IllegalArgumentException("Equiplet two times in event list: [eq=" + event.getEquiplet() + ", equiplets=" + equipletNames + ", events=" + event + "]");
				}
			}
		}

		if (!busyEquiplets.isEmpty()) {
			throw new IllegalArgumentException("Equiplet are buzy, but never finish " + busyEquiplets);
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
	private void updateProductStats(String type, int add) {
		int lastValue = productStatistics.get(type).lastEntry().getValue();
		System.out.println("UPDATE " + time + " : " + type + " = " + lastValue + " + " + add);
		productStatistics.get(type).put(time, lastValue + add);
	}

	/**
	 * calculate load,
	 * TODO throw not an illegal argument exception
	 */
	private void calculateEquipletLoad() {
		float sumLoad = 0f;
		float sumLoadHistory = 0f;
		for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
			Double load = 1 - entry.getValue().load(time, LOAD_WINDOW);
			equipletLoads.get(entry.getKey()).put(time, load.floatValue());
			sumLoad += load;

			if (time.greaterThan(LOAD_WINDOW)) {
				Double loadHistory = 1 - entry.getValue().loadHistory(time.minus(LOAD_WINDOW), LOAD_WINDOW);
				equipletLoadHistory.get(entry.getKey()).put(time, loadHistory.floatValue());
				sumLoadHistory += loadHistory;

				System.out.println("EA " + entry.getKey() + " = " + loadHistory);
			}
		}
		equipletLoads.get(STATS_LOAD_AVG).put(time, sumLoad / equiplets.size());
		if (time.greaterThan(LOAD_WINDOW)) {
			equipletLoadHistory.get(STATS_LOAD_AVG_HISTORY).put(time, sumLoadHistory / equiplets.size());
		}
	}

	private void reconfiguration() {
		if (time.minus(timeReconfig).greaterOrEqualThan(Settings.RECONFIG_CHECK) && reconfiguring == null) {
			System.out.println("CHECKPOINT MIKE");
			Map<Capability, List<Pair<String, Double>>> serviceLoads = new HashMap<Capability, List<Pair<String, Double>>>();

			for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
				IEquipletSim equiplet = entry.getValue();
				// change load to 1 high, 0 no load
				double load = 1 - equiplet.load(time, LOAD_WINDOW);
				List<Capability> capabilities = equiplet.getCapabilities();

				for (Capability capability : capabilities) {
					if (!serviceLoads.containsKey(capability)) {
						serviceLoads.put(capability, new ArrayList<Pair<String, Double>>());
					}
					serviceLoads.get(capability).add(new Pair<String, Double>(entry.getKey(), load));
				}
			}

			// set the highest and lowest equiplet loads
			// lowest can be null if all the two lowest are greater the the threshold
			Pair<String, Double> highest = new Pair<String, Double>(null, 0d);
			Pair<String, Double> lowest = new Pair<String, Double>(null, 3d);

			for (Entry<Capability, List<Pair<String, Double>>> entry : serviceLoads.entrySet()) {
				List<Pair<String, Double>> loadList = entry.getValue();

				// sort the load list on the basis of the load as we are only interested in the two lowest and the highest load
				Collections.sort(loadList, new Comparator<Pair<String, Double>>() {
					@Override
					public int compare(Pair<String, Double> o1, Pair<String, Double> o2) {
						return o1.second.compareTo(o2.second);
					}
				});

				// the lowest can only be set if there are more than 1 equiplet with the service
				// to prohibit that the service disappears from the grid
				if (loadList.size() > 1) {
					Pair<String, Double> load1 = loadList.get(0);
					Pair<String, Double> load2 = loadList.get(1);
					if (load1.second + load2.second < reconfigThreshold && load1.second < lowest.second) {
						lowest = load1;
					}
				}

				if (loadList.size() > 0) {
					Pair<String, Double> hLoad = loadList.get(loadList.size() - 1);
					if (hLoad.second > highest.second) {
						highest = hLoad;
					}
				}
			}

			// System.err.printf("Simulation: check service loads %s\n", Util.formatArray(serviceLoads));
			// System.err.printf("Simulation: equiplet reconfigure: [low=%s, high=%s]\n", lowest, highest);

			// if need to be reconfigured, then reconfig
			if (lowest.first != null && highest.first != null) {
				List<Capability> fromCapabilties = equiplets.get(lowest.first).getCapabilities();
				List<Capability> toCapabilities = equiplets.get(highest.first).getCapabilities();
				if (!toCapabilities.equals(fromCapabilties)) {
					equiplets.get(lowest.first).reconfigureStart();
					System.out.printf("Simulation: reconfig %s to capabilities %s\n", lowest.first, toCapabilities);

					String configEquiplet = reconfigured.containsKey(highest.first) ? reconfigured.get(highest.first).first : highest.first;
					reconfiguring = new Tuple<>(lowest.first, configEquiplet, time, new Pair<>(fromCapabilties, toCapabilities));

					log("reconfiguring", "Simulation", "Equiplet reconfigured [equiplet=" + lowest.first + ", load=" + lowest.second + ", toLoad=" + highest.second + ", form="
							+ fromCapabilties + ", to=" + toCapabilities + "(" + highest.first + ")]");
				}
			}
			timeReconfig = time;
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

		double sumProductionTime = 0.0;
		for (Entry<Tick, Tick> entry : productionTimes.entrySet()) {
			sumProductionTime += entry.getValue().doubleValue();
		}
		double avgProductionTimes = sumProductionTime / productionTimes.size();

		gui.update(time, e.getEquiplet() + ":" + e.getType(), products.size(), productCount, totalSteps, productStatistics.get(STATS_TRAVEL).lastEntry().getValue().intValue(), equipletStates, avgProductionTimes);
	}

	/**
	 * Event that signals the arrival of a new product in the system
	 * A product agent is created and started which will invoke the schedule
	 * behaviour
	 */
	private void productEvent() {
		try {
			// product agent settings
			LinkedList<ProductStep> productSteps = stochastics.generateProductSteps();

			String productName = "P" + productCount++;
			Position startPosition = new Position(-1, -1);

			Tick deadline = time.add(stochastics.generateDeadline());
			IProductSim product = simulation.createProduct(productName, startPosition, productSteps, time, deadline);
			products.put(productName, product);

			// update statistics
			totalSteps += productSteps.size();
			updateProductStats(STATS_SYSTEM, +1);
		} catch (Exception e) {
			e.printStackTrace();
		}

		// wait for confirmation creation of product agent
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
	 * Event that signals that an equiplet should have finished with the current
	 * job
	 * 
	 * @param equipletName
	 *            name of the equiplet
	 */
	private void finishedEvent(String equipletName, String productName) {
		IEquipletSim equipletAgent = equiplets.get(equipletName);
		System.out.printf("Simulation: equiplet finished: %s \n", equipletAgent);

		if (!equipletAgent.getExecutingProduct().equals(productName)) {
			throw new IllegalArgumentException("FINISHED event contains other product: " + productName + " than executing product: " + equipletAgent.getExecutingProduct());
		}

		if (equipletAgent.getEquipletState() == EquipletState.ERROR) {
			// equiplet broken down, wait until equiplet is repaired to continue with the remaining time
			equipletAgent.notifyJobFinished(time);
		} else if (equipletAgent.getEquipletState() == EquipletState.ERROR_REPAIRED) {
			equipletAgent.notifyJobFinished(time);

			Tick remainingTime = equipletAgent.getRemainingTime();
			eventStack.add(new Event(time.add(remainingTime), EventType.FINISHED, equipletAgent.getExecutingProduct(), equipletName));
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
		IEquipletSim equipletAgent = equiplets.get(equipletName);
		if (equipletAgent.getEquipletState() != EquipletState.RECONFIG) {
			updateProductStats(STATS_BROKEN, +1);

			equipletAgent.notifyBreakdown(time);

			// schedule REPAIRED time + repairTime, equiplet
			Tick repairTime = stochastics.generateRepairTime(equipletName);
			eventStack.add(new Event(time.add(repairTime), EventType.REPAIRED, equipletName));
			System.out.printf("Simulation: schedule event REPAIRED %s + %s, %s\n", time, repairTime, equipletName);
		} else {
			// schedule BREAKDOWN time + breakdown, equiplet
			Tick breakdown = stochastics.generateBreakdownTime(equipletName);
			eventStack.add(new Event(time.add(breakdown), EventType.BREAKDOWN, equipletName));
		}

		synchronized (this) {
			lock.unlock();
		}
		System.out.println("CHECKPOINT JULIETT");
	}

	/**
	 * Event that signals that an equiplet is repaired
	 * If the equiplet is finished during the repairing of the equiplet, a
	 * finished event is scheduled
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
			eventStack.add(new Event(time.add(remainingTime), EventType.FINISHED, equipletAgent.getExecutingProduct(), equipletName));
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
	 * event that signals that a product agent would check whether his product step is started
	 * 
	 * @param productName
	 *            name of product
	 * @param index
	 *            of the product step that should has started with processing
	 */
	private void startedEvent(String productName, int index) {
		try {
			// check if the product not terminated in the mean time i.e. finished before the started event occurs
			if (products.containsKey(productName)) {
				IProductSim productAgent = products.get(productName);
				System.out.println("Simulation: started event for product " + productName);
				productAgent.onProductStarted(time, index);
			} else {
				synchronized (this) {
					lock.unlock();
				}
			}
		} catch (IllegalArgumentException e) {
			for (Entry<String, IEquipletSim> equiplet : equiplets.entrySet()) {
				System.out.println("EQ: " + equiplet.getValue());
				List<Triple<String, Tick, Tick>> schedule = equiplet.getValue().getSchedule();

				System.out.println("EQ: scheduled: " + schedule.get(schedule.size() - 1));
			}
			throw e;
		}
		System.out.println("CHECKPOINT OSCAR");
	}

	/**
	 * event that signals the reconfiguration finished of an equiplet
	 * the capabilities changes of the equiplet and he registers by the df after which he becomes available for product agents.
	 * 
	 * @param equipletName
	 *            name of the equiplet
	 */
	private void reconfigEvent(String equipletName) {
		System.out.println("Simulation: reconfiged event for equiplet " + equipletName);

		// set next reconfiguration check time
		timeReconfig = time;

		IEquipletSim equiplet = equiplets.get(equipletName);

		// String equipletName = reconfiguring.first;
		String equipletConfig = reconfiguring.second;
		Tick reconfigStart = reconfiguring.third;
		List<Capability> oldCapabilties = reconfiguring.fourth.first;
		List<Capability> newCapabilities = reconfiguring.fourth.second;

		equiplet.notifyReconfigured(newCapabilities);

		reconfigured.put(equipletName, new Tuple<String, Tick, Tick, String>(equipletConfig, reconfigStart, time, oldCapabilties + "-> " + newCapabilities));
		reconfiguring = null;

		synchronized (this) {
			lock.unlock();
		}
	}

	/**
	 * Event that signals the end of the simulation
	 */
	private void doneEvent() {
		System.out.println("Simulation: simulation finished");
		running = false;

		synchronized (this) {
			lock.unlock();
		}
		System.out.println("CHECKPOINT LIMA");

		// calculate statistics of a run
		TreeMap<Tick, Integer> pFinished = productStatistics.get(STATS_FINISHED);
		int sFinished = pFinished.get(pFinished.headMap(Settings.WARMUP).lastKey());
		int productsFinished = pFinished.lastEntry().getValue() - sFinished;

		TreeMap<Tick, Integer> pFailed = productStatistics.get(STATS_FAILED_CREATION);
		int sFailed = pFailed.get(pFailed.headMap(Settings.WARMUP).lastKey());
		int productsFailed = pFailed.lastEntry().getValue() - sFailed;

		TreeMap<Tick, Integer> pOverdue = productStatistics.get(STATS_FAILED_DEADLINE);
		int sOverdue = pOverdue.get(pOverdue.headMap(Settings.WARMUP).lastKey());
		int productsOverdue = pOverdue.lastEntry().getValue() - sOverdue;

		double sumProductionTimes = 0;
		int countProductionTimes = 0;
		for (Entry<Tick, Tick> entry : productionTimes.entrySet()) {
			if (entry.getKey().greaterOrEqualThan(Settings.WARMUP)) {
				sumProductionTimes += entry.getValue().doubleValue();
				countProductionTimes++;
			}
		}

		float avgProductionTimes = (float) (sumProductionTimes / countProductionTimes);

		double sumLoad = 0;
		double countLoad = 0;

		for (Entry<String, TreeMap<Tick, Float>> entry : equipletLoadHistory.entrySet()) {
			for (Entry<Tick, Float> time : entry.getValue().entrySet()) {
				if (time.getKey().greaterOrEqualThan(Settings.WARMUP)) {
					double eLoad = time.getValue();
					double l = sumLoad + time.getValue();
					if ((sumLoad < 0 && eLoad < 0 && l >= 0) || (sumLoad > 0 && eLoad > 0 && l <= 0)) {
						// Overflow occurred
						throw new RuntimeException("load calculation overflow. write method for discarding values");
					}
					sumLoad = l;
					countLoad++;
				}
			}
		}
		float avgLoad = (float) (sumLoad / countLoad);

		runStats.put(run, new Pair<>(new Tuple<Integer, Integer, Integer, Float>(productsFinished, productsFailed, productsOverdue, avgProductionTimes), avgLoad));

		System.err.printf("Simlation: run finished stats=[finished=%d, failed=%d, overdue=%d, production times=%.2f, load=%.4f]\n", productsFinished, productsFailed, productsOverdue, avgProductionTimes, avgLoad);

		saveStatistics();

		// take down all agents
		for (Entry<String, IEquipletSim> equiplet : equiplets.entrySet()) {
			equiplet.getValue().kill();
		}

		for (Entry<String, IProductSim> product : products.entrySet()) {
			product.getValue().kill();
		}

		simulation.killAgent(MASConfiguration.TRAFFIC_AGENT);

		// TODO check if needed to wait until messages are received and all is taken down / killed
		simulation.delay(2000);
		System.gc();

		// something is needed, there is a (although little) that some agent are not correctly started and therefore the simulation fails to run correctly without the possibility
		// to check it.
		try {
			Thread.sleep(3000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		if (run == runs) {
			finished = true;
			saveRunsStatistics();
		} else {
			run++;
			init();

			System.out.println("Simulation: run " + run);

			running = true;
		}
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
		IProductSim productAgent = products.get(productName);
		productAgent.kill();
		products.remove(productName);

		// schedule next product arrival
		Tick arrivalTime = stochastics.generateProductArrival(time);
		eventStack.add(new Event(time.add(arrivalTime), EventType.PRODUCT));
		System.out.printf("Simulation: schedule event PRODUCT %s + %s\n", time, arrivalTime);

		// continue with simulation
		synchronized (this) {
			lock.unlock();
		}
		System.out.println("CHECKPOINT GOLF");
	}

	/**
	 * Product notifies that the product is successful created
	 * The product will travel to the first equiplet, i.e. an arrive event is
	 * scheduled
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
		System.out.printf("Simulation: schedule event ARRIVED %s + %s, from %s at %s to %s at %s\n", time, travelTime, productName, productAgent.getPosition(), equipletName, equipletAgent.getPosition());

		// traveling++;
		updateProductStats(STATS_TRAVEL, +1);

		// schedule next product arrival
		Tick arrivalTime = stochastics.generateProductArrival(time);
		eventStack.add(new Event(time.add(arrivalTime), EventType.PRODUCT));
		System.out.printf("Simulation: schedule event PRODUCT %s + %s\n", time, arrivalTime);

		// continue with simulation
		synchronized (this) {
			lock.unlock();
		}
		System.out.println("CHECKPOINT ALPHA");
	}

	/**
	 * Product notifies that he is not going to meet the deadline.
	 * 
	 * @param productName
	 *            name of the product
	 */
	@Override
	public void notifyProductOverdue(String productName) {
		System.out.printf("Simulation: product agent %s not completed within deadline\n", productName);
		/*
		 * The question is whether or not the product should continue with producing or he should just quit and die.
		 * For now, the product will just continue and when he is finished the statistics will be updated.
		 */

		// updateProductStats(STATS_FAILED_DEADLINE, +1);
		// updateProductStats(STATS_SYSTEM, -1);
		//
		// // remove agent from system
		// IProductSim productAgent = products.get(productName);
		// productAgent.kill();
		// products.remove(productName);
		//
		// simulation.killAgent(productName);

		System.out.println("CHECKPOINT NOVEMBER");
	}

	/**
	 * Product notifies that an equiplet has informed him that he is started to
	 * execute a job for him
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
	public void notifyProductProcessing(String productName, String equipletName, String service, int index) {
		updateProductStats(STATS_WAITING, -1);
		updateProductStats(STATS_BUSY, +1);
		if (index == 0) {
			updateProductStats(STATS_PHYSICAL, +1);
		}

		System.out.printf("Simulation: product agent %s notifies processing.\n", productName);
		String eqName = reconfigured.containsKey(equipletName) ? reconfigured.get(equipletName).first : equipletName;

		try {
			Tick productionTime = stochastics.generateProductionTime(eqName, service);

			// schedule FINISHED time + productionTime, equiplet
			eventStack.add(new Event(time.add(productionTime), EventType.FINISHED, productName, equipletName));
			System.out.printf("Simulation: schedule event FINISHED %s + %s, %s, %s\n", time, productionTime, productName, equipletName);
		} catch (NullPointerException e) {
			// TODO no error handling
			e.printStackTrace();
			System.out.println("reconfigured " + reconfigured);
			for (Entry<String, IEquipletSim> equiplet : equiplets.entrySet()) {
				System.out.println("EQ: " + equiplet.getValue());
			}
			throw new IllegalArgumentException("WHAA");
		}

		// unblock simulation when notifying job finished
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
		System.out.printf("Simulation: schedule event ARRIVED %s + %s, from %s at %s to %s at %s\n", time, travelTime, productName, productAgent.getPosition(), equipletName, equipletAgent.getPosition());

		// unblock simulation when notifying job finished
		System.out.println("CHECKPOINT HOTEL");
	}

	/**
	 * when a product arrives at an equiplet, he (probably) sets a timer that goes off on the last moment a product steps could start affecting the next product step
	 */
	public void notifyProductShouldStart(String productName, Tick start, int index) {
		if (MASConfiguration.RESCHEDULE) {
			eventStack.add(new Event(start.max(time), EventType.STARTED, productName, index));
			System.out.printf("Simulation: schedule STARTED event for product %s at %s with prodcut step %d \n", productName, start, index);
		}
		System.out.println("CHECKPOINT ROMEO");
	}

	/**
	 * 
	 */
	@Override
	public void notifyProductRescheduled(boolean rescheduled) {
		// product has started already so rescheduling is not needed, continue with the simulation
		synchronized (this) {
			lock.unlock();
		}
		System.out.println("CHECKPOINT PAPA");
	}

	@Override
	public void notifyProductRescheduled(String productName, String equipletName, boolean rescheduled) {
		if (rescheduled) {
			updateProductStats(STATS_RESCHEDULED, +1);
			updateProductStats(STATS_WAITING, -1);

			IProductSim productAgent = products.get(productName);
			Position startPosition = productAgent.getPosition();

			IEquipletSim equipletAgent = equiplets.get(equipletName);
			Position nextPosition = equipletAgent.getPosition();

			// schedule ARRIVED time + travelTime, equiplet, product
			int travelSquares = Math.abs(startPosition.getX() - nextPosition.getX()) + Math.abs(startPosition.getY() - nextPosition.getY());
			Tick travelTime = stochastics.generateTravelTime(travelSquares);

			eventStack.add(new Event(time.add(travelTime), EventType.ARRIVED, productName, equipletName));
			System.out.printf("Simulation: schedule event ARRIVED %s + %s, from %s at %s to %s at %s\n", time, travelTime, productName, productAgent.getPosition(), equipletName, equipletAgent.getPosition());

			// traveling++;
			updateProductStats(STATS_TRAVEL, +1);

		}

		if (!rescheduled) {
			throw new IllegalArgumentException("Wait What?!");
		}

		synchronized (this) {
			lock.unlock();
		}
		System.out.println("CHECKPOINT QUEBEC");
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
		updateProductStats(STATS_PHYSICAL, -1);

		// Product is finished
		IProductSim productAgent = products.get(productName);
		products.remove(productName);
		productionTimes.put(time, time.minus(productAgent.getCreated()));

		Tick deadline = productAgent.getDeadline();
		if (time.greaterThan(deadline)) {
			updateProductStats(STATS_FAILED_DEADLINE, +1);
		}

		System.out.println("CHECKPOINT INDIA");
	}

	@Override
	public void notifyReconfigReady(String equipletName) {
		Tick reconfigTime = stochastics.generateReconfigTime();
		eventStack.add(new Event(time.add(reconfigTime), EventType.RECONFIG, equipletName));
		System.out.printf("Simulation: schedule RECONFIG event for equiplet %s over %s\n", time, equipletName, reconfigTime);
		System.out.println("CHECKPOINT SIERRA");
	}

	/**
	 * log function available for agents that want to log certain information during the simulation
	 */
	@Override
	public void log(String info, String agent, String message) {
		if (debugInfo.containsKey(info)) {
			TreeMap<String, String> entry = debugInfo.get(info);

			String s = (entry.containsKey(agent) ? entry.get(agent) : "") + "\n" + time + ": " + message;
			entry.put(agent, s);
			debugInfo.put(info, entry);
		} else {
			TreeMap<String, String> map = new TreeMap<>();
			map.put(agent, time + ": " + message);
			debugInfo.put(info, map);
		}
	}

	public boolean isFinished() {
		return finished;
	}

	/**
	 * handle one event
	 */
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
	}

	@Override
	public synchronized int getDelay() {
		return delay;
	}

	@Override
	public synchronized void setDelay(int delay) {
		this.delay = Math.max(1, delay);
	}

	/**
	 * save the statistics of one simulation run
	 */
	private void saveRunsStatistics() {
		String path = outputFolder;

		File configFile = new File(path + "config.txt");
		File statFile = new File(path + "stats.csv");
		try {
			saveConfig(configFile);

			PrintWriter writer = new PrintWriter(statFile);
			writer.println("run;finished;failed;overdue;production time;load;");
			Tuple<Integer, Integer, Integer, Double> sumPStats = new Tuple<Integer, Integer, Integer, Double>(0, 0, 0, 0d);
			double sumEStats = 0;
			for (Entry<Integer, Pair<Tuple<Integer, Integer, Integer, Float>, Float>> entry : runStats.entrySet()) {
				Pair<Tuple<Integer, Integer, Integer, Float>, Float> stats = entry.getValue();
				writer.printf("%d;%d;%d;%d;%.2f;%.4f;\n", entry.getKey(), stats.first.first, stats.first.second, stats.first.third, stats.first.fourth, stats.second);
				sumPStats.first += stats.first.first;
				sumPStats.second += stats.first.second;
				sumPStats.third += stats.first.third;
				sumPStats.fourth += stats.first.fourth;
				sumEStats += stats.second;
			}

			writer.printf("avg;%.2f;%.2f;%.2f;%.2f;%.4f;\n", 1.0 * sumPStats.first / runStats.size(), 1.0 * sumPStats.second / runStats.size(), 1.0 * sumPStats.third
					/ runStats.size(), 1.0 * sumPStats.fourth / runStats.size(), sumEStats / runStats.size());

			writer.println();
			writer.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
	}

	/**
	 * save the configurations of a simulation run
	 * 
	 * @param configFile
	 *            to save the configurations in
	 * @throws FileNotFoundException
	 */
	private void saveConfig(File configFile) throws FileNotFoundException {
		PrintWriter writer = new PrintWriter(configFile);
		writer.println("Simulation: " + new SimpleDateFormat("dd MM yyyy 'at' HH:mm:ss").format(new Date()));
		writer.printf("Scheduling: %s\n", MASConfiguration.SCHEDULING);
		writer.printf("Stochastics: %s\n", Settings.STOCHASTICS);
		writer.printf("Reschedule: %s\n", MASConfiguration.RESCHEDULE);
		writer.printf("Breakdowns: %s\n", Settings.BREAKDOWNS);
		writer.printf("Queue jump: %s\n", MASConfiguration.QUEUE_JUMP);
		writer.printf("Reconfiguration time: %s\n", Settings.RECONFIGATION_TIME);
		writer.printf("Warm-up period: %s\n", Settings.WARMUP);
		writer.printf("Product generation: (%.0f * %d) / (%.2f * %d) = %.2f\n", stochastics.getMeanProcessingTime(), Settings.MEAN_PRODUCT_STEPS, Settings.UTILIZATION, config.getEquipletsConfigurations().size(), (stochastics.getMeanProcessingTime() * Settings.MEAN_PRODUCT_STEPS)
				/ (Settings.UTILIZATION * config.getEquipletsConfigurations().size()));
		writer.printf("Verbosity: %d\n", MASConfiguration.VERBOSITY);
		writer.printf("Communication timeout: %s\n", MASConfiguration.COMMUNICATION_TIMEOUT);
		writer.printf("Configuration:\n%s\n", config);
		writer.println();
		writer.close();
	}

	/**
	 * save statistics into a comma separated values file. these are time based statistics
	 * with the files graph could be reproduced
	 * 
	 * @param path
	 *            of the file to save
	 * @param title
	 *            or name of the file
	 * @param data
	 *            a map of statistic name with a map of values per time
	 * @throws FileNotFoundException
	 */
	public <M extends Map<Tick, T>, T extends Number> void saveCSVStats(String path, String title, Map<String, M> data) throws FileNotFoundException {
		File file = new File(path + title + ".csv");
		PrintWriter writer = new PrintWriter(file);

		TreeSet<Tick> ticks = new TreeSet<>();

		for (Entry<String, M> entry : data.entrySet()) {
			for (Entry<Tick, T> tick : entry.getValue().entrySet()) {
				ticks.add(new Tick(Math.round(tick.getKey().doubleValue())));
			}
		}

		if (!ticks.isEmpty()) {
			Tick header = new Tick(-1);

			Map<String, M> lastValues = new HashMap<>();
			TreeMap<Tick, String> lines = new TreeMap<>();

			lines.put(header, "");
			for (Entry<String, M> entry : data.entrySet()) {
				double lastValue = 0;

				// print header
				lines.put(header, lines.get(header).concat(";" + entry.getKey()));

				// print values per tick of statistic
				Map<Tick, T> statsData = entry.getValue();
				Iterator<Tick> dataIterator = statsData.keySet().iterator();
				Iterator<Tick> tickIterator = ticks.iterator();

				if (statsData.isEmpty()) {
					break;
				}

				Tick lastTick = tickIterator.next();
				Tick time = dataIterator.next();

				while (tickIterator.hasNext()) {
					Tick tick = tickIterator.next();
					double value = 0d;
					int counter = 0;

					while (dataIterator.hasNext()) {
						if (time.greaterOrEqualThan(lastTick) && time.lessThan(tick)) {
							value += statsData.get(time).doubleValue();
							counter++;
							time = dataIterator.next();
						} else {
							break;
						}
					}

					if (!lines.containsKey(tick)) {
						lines.put(tick, "");
					}

					double v = counter > 0 ? value / counter : lastValues == null ? 0 : lastValue;
					String s = lines.get(tick).concat(";" + String.valueOf(v));
					lines.put(tick, s);

					lastValue = v;
					lastTick = tick;
				}
			}

			for (Entry<Tick, String> line : lines.entrySet()) {
				if (line.getKey().lessThan(0)) {
					writer.println(line.getValue());
				} else {
					writer.println(line.getKey().toString() + line.getValue());
				}
			}

			writer.close();
		} else {
			System.err.println("Simulation: no ticks in data to save :" + data);
		}
	}

	@Override
	public synchronized void saveStatistics() {
		System.out.println("Simulation: save statistics");

		String path = Settings.SIMULATION_OUTPUT;

		File file = new File(path);
		if (!file.exists()) {
			file.mkdir();
		}

		if (outputFolder == null) {
			int number = 1;
			Pattern pattern = Pattern.compile("run([0-9]+)");
			File[] fileList = file.listFiles();
			for (File f : fileList) {
				Matcher matcher = pattern.matcher(f.getName());
				if (f.isDirectory() && matcher.find()) {
					number = Math.max(Integer.valueOf(matcher.group(1)) + 1, number);
				}
			}

			path += String.format("/run%d/", number);
			file = new File(path);
			file.mkdir();
			outputFolder = path;
		} else {
			path = outputFolder;
		}

		if (runs > 1) {
			path += String.format("run%d/", run);
			file = new File(path);
			file.mkdir();
		}

		File configFile = new File(path + "config.txt");
		File statFile = new File(path + "stats.txt");
		File productFile = new File(path + "products.txt");
		File equipletFile = new File(path + "equiplet.txt");
		PrintWriter writer;
		try {
			saveConfig(configFile);

			// debug information
			for (Entry<String, TreeMap<String, String>> entry : debugInfo.entrySet()) {
				File logFile = new File(path + entry.getKey() + ".txt");
				writer = new PrintWriter(logFile);
				for (Entry<String, String> agent : entry.getValue().entrySet()) {
					writer.write(agent.getValue());
				}
				writer.close();
			}

			// products
			writer = new PrintWriter(productFile);

			TreeMap<String, IProductSim> sortedProducts = new TreeMap<String, IProductSim>(new Comparator<String>() {
				private final Pattern pattern = Pattern.compile("\\D*(\\d*)");

				@Override
				public int compare(String a, String b) {
					Matcher matcher1 = pattern.matcher(a);
					Matcher matcher2 = pattern.matcher(b);
					if (matcher1.matches() && matcher2.matches()) {
						return Integer.valueOf(matcher1.group(1)).compareTo(Integer.valueOf(matcher2.group(1)));
					} else {
						return 0;
					}
				}
			});
			sortedProducts.putAll(products);

			for (Entry<String, IProductSim> entry : sortedProducts.entrySet()) {
				IProductSim product = entry.getValue();
				writer.println(product);
			}

			writer.close();

			// Equiplets
			writer = new PrintWriter(equipletFile);
			for (Entry<String, IEquipletSim> entry : equiplets.entrySet()) {
				writer.println(entry.getValue().toFullString());
			}
			writer.close();

			// write simulation statistics
			writer = new PrintWriter(statFile);
			writer.println("Grid Simulation run");
			// writer.printf("Product Statistics: \t%s\r\n", productStatistics);
			writer.println();
			writer.println();
			// writer.printf("Equiplet Statistics: \t%s\r\n", equipletStatistics);
			writer.println("\r\t");
			writer.println("reconfigured:");

			// sort reconfigured the ugly way I think, sort hashmap on value
			List<Entry<String, Tuple<String, Tick, Tick, String>>> list = new LinkedList<Entry<String, Tuple<String, Tick, Tick, String>>>(reconfigured.entrySet());

			Collections.sort(list, new Comparator<Entry<String, Tuple<String, Tick, Tick, String>>>() {
				@Override
				public int compare(Entry<String, Tuple<String, Tick, Tick, String>> o1, Entry<String, Tuple<String, Tick, Tick, String>> o2) {
					return o1.getValue().second.compareTo(o2.getValue().second);
				}
			});
			writer.println(list);

			writer.println("more statistics");

			writer.println();
			writer.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}

		HashMap<String, TreeMap<Tick, Float>> mvAVGLoad = new HashMap<String, TreeMap<Tick, Float>>();
		for (Entry<String, TreeMap<Tick, Float>> entry : equipletLoads.entrySet()) {
			mvAVGLoad.put(entry.getKey(), Util.movingAverage(entry.getValue(), 1000));
		}
		HashMap<String, TreeMap<Tick, Float>> mvAVGLoadHistory = new HashMap<String, TreeMap<Tick, Float>>();
		for (Entry<String, TreeMap<Tick, Float>> entry : equipletLoadHistory.entrySet()) {
			mvAVGLoadHistory.put(entry.getKey(), Util.movingAverage(entry.getValue(), 1000));
		}

		Chart.save(path, "Product Statistics", "Products", productStatistics);
		Chart.save(path, "Equiplet Loads", "Equiplets", mvAVGLoad);
		Chart.save(path, "Equiplet Load Histories", "Equiplets", mvAVGLoadHistory);

		try {
			saveCSVStats(path, "Equiplet Loads", mvAVGLoad);
			saveCSVStats(path, "Equiplet Load Histories", mvAVGLoadHistory);
			saveCSVStats(path, "Product Statistics", productStatistics);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}

		HashMap<String, Map<Tick, Tick>> map = new HashMap<>();
		map.put("ProductionTime", productionTimes);
		Chart.save(path, "Production Time", "Prodcuts", map);

		Map<String, Map<Tick, Tick>> latency = getEquipletLatency();
		Chart.save(path, "Equiplet Latency", "Latency", latency);
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
	public Map<String, TreeMap<Tick, Integer>> getProductStatistics() {
		Map<String, TreeMap<Tick, Integer>> stats = new HashMap<>(productStatistics);
		stats.remove(STATS_RESCHEDULED);
		return stats;
	}

	@Override
	public Map<String, Map<Tick, Float>> getEquipletStatistics() {
		return new HashMap<String, Map<Tick, Float>>(equipletLoads);
	}

	@Override
	public Map<String, Map<Tick, Float>> getEquipletLoadHistories() {
		return new HashMap<String, Map<Tick, Float>>(equipletLoadHistory);
	}

	@Override
	public Map<Tick, Tick> getProductionTimes() {
		return productionTimes;
	}
}

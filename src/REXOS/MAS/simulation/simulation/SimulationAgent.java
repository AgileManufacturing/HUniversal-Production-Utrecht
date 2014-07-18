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
import java.util.TreeSet;

import org.json.JSONException;

import simulation.config.Config;
import simulation.graphics.Control;
import simulation.graphics.SimInterface;
import simulation.mas.Equiplet;
import simulation.mas.EquipletAgent;
import simulation.mas.Product;
import simulation.mas.ProductAgent;
import simulation.util.Capability;
import simulation.util.Parser;
import simulation.util.Position;
import simulation.util.ProductStep;
import simulation.util.ProductionStep;
import simulation.util.Triple;
import simulation.util.Tuple;

public class SimulationAgent extends Agent implements Control, ISimulation {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private SimInterface gui;
	private Config config;
	private Stochastics stochastics;

	private int run;
	private int runs;
	private double run_length;
	private boolean finished;
	private boolean running;
	private int delay;

	protected final Object lock = new Object();
	private boolean eventReady;

	// State
	private TreeSet<Event> eventStack;
	private double time;

	private Map<String, EquipletAgent> equiplets;
	private Map<String, ProductAgent> products;
	private int productCount;

	private Map<String, Triple<Double, Double, Double>> equipletHistories;

	// Performance
	private int totalSteps;

	private int traveling;
	private HashMap<String, Double> throughput;

	// TODO

	public void setup() {
		System.out.println("Simulation: setup.");
		gui = SimInterface.create(this);
		config = Config.read();
		stochastics = new Stochastics(config);
		delay = 1500; // if delay = 0, waits forever

		init();
	}

	private void init() {
		eventStack = new TreeSet<>();

		run = 0;
		runs = config.getRuns();
		run_length = config.getRunLength();
		finished = false;
		running = false;

		time = 0;

		totalSteps = 0;
		productCount = 0;
		throughput = new HashMap<String, Double>();

		products = new HashMap<>();
		equiplets = new HashMap<>();

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
				String configurations = Parser.parseEquipletConfiguration(position, capabilities, productionTimes);

				EquipletAgent equiplet = new EquipletAgent(position, capabilities, productionTimes);

				// Object[] properties = { configurations };
				ContainerController cc = getContainerController();
				// AgentController ac = cc.createNewAgent(equipletName, EquipletAgent.class.getName(), properties);
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
			} catch (JSONException e) {
				System.err.printf("Simulation: failed to construct equiplet agent %s construct configurations.\n", equipletName);
				System.err.printf("Simulation: %s\n", e.getMessage());
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
				while (running) {

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

					System.out.println("\n-----\nSimulation iteration: " + e + " : " + eventStack);

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
	public synchronized Map<String, Product> getProducts() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public List<Equiplet> getEquiplets() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public synchronized Map<String, Triple<Double, Double, Double>> getEquipletHistory() {
		return equipletHistories;
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
		this.delay = delay;
	}

	@Override
	public synchronized void saveStatistics() {
		// TODO Auto-generated method stub

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
		gui.update(time, products.size(), productCount, totalSteps, traveling, equipletStates, waitingTime, busy, throughput);
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
		} catch (StaleProxyException e1) {
			System.err.printf("Simulation: ERROR: product agent %s creation was not possible.\n", productName);
			e1.printStackTrace();
		}
		// wait for confirmation creation of product agent
		changeReady(false);
		System.out.println("CHECKPOINT ECHO");
	}

	private void arrivedEvent(String productName, String equipletName) {
		traveling--;
		ProductAgent productAgent = products.get(productName);
		productAgent.notifyProductArrived(time);

		ProductionStep step = productAgent.getCurrentStep();
		EquipletAgent equipletAgent = equiplets.get(equipletName);

		// check if the just scheduled product step is ready of execution
		if (equipletAgent.isExecutingStep(productName, step.getService(), step.getCriteria())) {
			double productionTime = stochastics.generateProductionTime(equipletName, step.getService());

			// schedule FINISHED time + productionTime, equiplet
			eventStack.add(new Event(time + productionTime, EventType.FINISHED, equipletName));
			System.out.printf("Simulation: schedule event FINISHED %.0f + %.0f, %s, %s\n", time, productionTime, productName, equipletName);
		}

		// wait until an equiplet notify continue
		// changeReady(false);
		System.out.println("CHECKPOINT DELTA");
	}

	private void finishedEvent(String equipletName) {
		EquipletAgent equipletAgent = equiplets.get(equipletName);
		equipletAgent.notifyJobFinished(time);

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
			System.out.printf("Simulation: schedule event ARRIVED %.0f + %.0f, %s, %s\n", time, travelTime, productName, equipletName);
			traveling++;
		} else {
			// TODO statistics update
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
	public void notifyProductTraveling(String productName, String equipletName) {
		System.out.printf("Simulation: product agent %s product step finished and traveling to equiplet %s\n", productName, equipletName);
		
		ProductAgent productAgent = products.get(productName);
		Position startPosition = productAgent.getPosition();

		EquipletAgent equipletAgent = equiplets.get(equipletName);
		Position nextPosition = equipletAgent.getPosition();

		// schedule ARRIVED time + travelTime, equiplet, product
		int travelSquares = Math.abs(startPosition.getX() - nextPosition.getX()) + Math.abs(startPosition.getY() - nextPosition.getY());
		double travelTime = stochastics.generateTravelTime(travelSquares);

		eventStack.add(new Event(time + travelTime, EventType.ARRIVED, productName, equipletName));
		System.out.printf("Simulation: schedule event ARRIVED %.0f + %.0f, %s, %s\n", time, travelTime, productName, equipletName);
		traveling++;
	}

	@Override
	public void notifyProductFinished(String productName) {
		System.out.printf("Simulation: product agent %s is finished.\n", productName);
		
		// Product is finished
		ProductAgent productAgent = products.get(productName);
		products.remove(productName);
		throughput.put(productName, time - productAgent.getCreated());
	}
}

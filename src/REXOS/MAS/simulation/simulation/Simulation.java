package simulation.simulation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeSet;

import simulation.config.Config;
import simulation.graphics.SimInterface;
import simulation.mas.Equiplet;
import simulation.mas.EquipletState;
import simulation.mas.Product;
import simulation.mas.Product.ProductState;
import simulation.util.Position;
import simulation.util.ProductStep;
import simulation.util.ProductionStep;
import simulation.util.Triple;

public class Simulation extends Thread {

	// private final static Logger Log = Logger.getLogger("Simulation");

	private SimInterface gui;
	private Grid grid;
	private Stochastics stochastics;
	private Config config;

	private TreeSet<Event> eventStack;

	private int run;
	private int runs;
	private double run_length;
	private boolean finished;
	private boolean running;
	private int delay;

	// State
	private double time;

	private int productCount;
	private Map<String, Product> products;

	// Performance
	private double Q; // time of waiting products
	private double Q_n; // sum of waiting products
	private HashMap<Integer, Double> B;// time equiplets producing
	private HashMap<String, Double> throughput;
	private int traveling;
	private int totalSteps;

	public Simulation(SimInterface gui) {
		this.gui = gui;
		grid = Grid.getInstance();
		config = Config.read();
		stochastics = new Stochastics(config);
		delay = 0;
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
		products = new HashMap<>();

		Q = 0;
		Q_n = 0;
		B = new HashMap<>();
		throughput = new HashMap<>();
		traveling = 0;

		for (Entry<String, Equiplet> entry : grid.getEquiplets().entrySet()) {
			double breakdown = stochastics.generateBreakdownTime(entry.getKey());
			// eventStack.add(new Event(time + breakdown, EventType.BREAKDOWN, entry.getKey()));
		}

		eventStack.add(new Event(time, EventType.PRODUCT));
		scheduleDone();
	}

	public void run() {
		while (run < runs) {
			while (!finished) {
				while (running) {
					Event e = eventStack.pollFirst();
					// double elapsedTime = e.getTime() - time;
					time = e.getTime();

					System.out.println("\n-----\nSimulation iteration: " + e + " : " + eventStack);

					switch (e.getType()) {
					case PRODUCT:
						productEvent();
						break;
					case ARRIVED:
						startEvent(e.getProduct(), e.getEquiplet());
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

					//System.out.println("\nSimulation state: " + formatArray(grid.getEquiplets()));
					//System.out.println("\nSimulation products: " + formatArray(products) + "\n");
					
					//validate();

					try {
						sleep(delay);
					} catch (InterruptedException ex) {
						ex.printStackTrace();
					}
				}
				try {
					synchronized (this) {
						while (!running)
							wait();//wait until notify gets called in startThread
					}
				} catch (InterruptedException ie) {
				}

				//				if (!running) {
				//					try {
				//						sleep(2000);
				//					} catch (InterruptedException e) {
				//						e.printStackTrace();
				//					}
				//				}
			}
			saveStatistics();
			run++;
		}
		gui.reset();
	}

	private void productEvent() {
		// double deadline = stochastics.generateDeadline();
		// double duetime = stochastics.generateDuetime();
		LinkedList<ProductStep> productSteps = stochastics.generateProductSteps();
		totalSteps += productSteps.size();

		String product = "P" + productCount++;
		Position startPosition = new Position(-1, -1);
		Product productAgent = new Product(product, time, productSteps, startPosition);
		String equiplet = productAgent.getNextEquipet();

		Equiplet equipletAgent = grid.getEquiplet(equiplet);

		//schedule START time + travelTime, equiplet, product
		double travelTime = grid.getTravelTime(startPosition, equipletAgent.getPosition());
		eventStack.add(new Event(time + travelTime, EventType.ARRIVED, product, equiplet));
		products.put(product, productAgent);
		traveling++;

		System.out.printf("Simulation: schedule event START %.0f + %.0f, %s, %s\n", time, travelTime, product, equiplet);

		// schedule next product arrival
		double arrivalTime = stochastics.generateProductArrival();
		eventStack.add(new Event(time + arrivalTime, EventType.PRODUCT));
		System.out.printf("Simulation: schedule event PRODUCT %.0f + %.0f\n", time, arrivalTime);
	}

	private void startEvent(String product, String equiplet) {
		traveling--;

		// notitfy that the product arrived at the destination
		// let the product agent in turn notify the equiplet agent that he is ready to produce
		Product productAgent = products.get(product);
		productAgent.notifyProductArrived();
		ProductionStep step = productAgent.getExecutingStep();

		Equiplet equipletAgent = grid.getEquiplet(equiplet);
		equipletAgent.notifyProductArrived(time, product, step.getService());

		// check if the just scheduled product step is ready of execution
		if (equipletAgent.isExecutingStep(product, step.getService(), step.getCriteria())) {
			productAgent.notifyProductProcessing();
			double productionTime = stochastics.generateProductionTime(equiplet, step.getService());
			// schedule FINISHED time + productionTime, equiplet

			// TODO delete product, only for debug purpose
			eventStack.add(new Event(time + productionTime, EventType.FINISHED, equiplet));
			System.out.printf("Simulation: schedule event FINISHED %.0f + %.0f, %s, %s\n", time, productionTime, product, equiplet);
		}
	}

	private void finishedEvent(String equiplet) {
		Equiplet equipletAgent = grid.getEquiplet(equiplet);

		if (equipletAgent.getEquipletState() == EquipletState.ERROR) {
			equipletAgent.notifyJobFinished(time);
		} else if (equipletAgent.getEquipletState() == EquipletState.BROKENDOWN) {
			double remainingTime = equipletAgent.getTimeBroken();
			equipletAgent.notifyJobFinished(time);
			// schedule FINISHED time + productionTime, equiplet
			// TODO delete product, only for debug purpose
			eventStack.add(new Event(time + remainingTime, EventType.FINISHED, equiplet));
			System.out.printf("Simulation: schedule event FINISHED when breakdown happend %.0f + %.0f, %s\n", time, remainingTime, equiplet);

		} else {
			System.out.println("Equiplet Finished: " + equipletAgent);
			String product = equipletAgent.getExecutingProduct();
			equipletAgent.notifyJobFinished(time);

			Product productAgent = products.get(product);
			productAgent.notifyProductStepFinished();
			if (productAgent.isFinished()) {
				// Product is finished
				products.remove(product);
				throughput.put(product, time - productAgent.getCreated());
			} else {
				// product has next steps in production path
				String nextEquiplet = productAgent.getNextEquipet();
				Equiplet nextEquipletAgent = grid.getEquiplet(nextEquiplet);

				double travelTime = grid.getTravelTime(productAgent.getPosition(), nextEquipletAgent.getPosition());
				// schedule START time + travelTime, equiplet, product
				eventStack.add(new Event(time + travelTime, EventType.ARRIVED, product, nextEquiplet));
				System.out.printf("Simulation: schedule event START for next product step %.0f + %.0f, %s, %s\n", time, travelTime, product, nextEquiplet);
			}

			// schedule next FINISHED event
			if (equipletAgent.isExecuting()) {
				String nextProduct = equipletAgent.getExecutingProduct();
				Product nextProductAgent = products.get(nextProduct);
				// nextProductAgent.notifyProductStepFinished();

				if (nextProductAgent.isFinished()) {
					System.out.println("WHAT THE FUCK");
				}

				ProductionStep step = nextProductAgent.getExecutingStep();

				double productionTime = stochastics.generateProductionTime(equiplet, step.getService());

				// schedule FINISHED time + productionTime, equiplet
				// TODO remove product agent from event.
				eventStack.add(new Event(time + productionTime, EventType.FINISHED, equiplet));
				System.out.printf("Simulation: schedule event FINISHED for next equiplet job %.0f + %.0f, %s\n", time, productionTime, equiplet);
			}
		}
		System.out.println("Equiplet state after finished: " + grid.getEquiplets());
	}

	private void breakdownEvent(String equiplet) {
		Equiplet equipletAgent = grid.getEquiplet(equiplet);

		equipletAgent.notifyBreakdown(time);
		double repairTime = stochastics.generateRepairTime(equiplet);
		// schedele REPAIRED time + repairTime, equiplet
		eventStack.add(new Event(time + repairTime, EventType.REPAIRED, equiplet));
		System.out.printf("Simulation: schedule event REPAIRED %.0f + %.0f, %s\n", time, repairTime, equiplet);
	}

	private void repairedEvent(String equiplet) {
		Equiplet equipletAgent = grid.getEquiplet(equiplet);

		if (equipletAgent.hasFinishedDuringRepair()) {
			double remainingTime = time - equipletAgent.getShouldHaveFinish();
			equipletAgent.notifyRepaired(time);

			String pr = equipletAgent.getExecutingProduct();
			// schedule FINISHED time + remainingTime, equiplet

			// TODO delete product, only for debug purpose
			eventStack.add(new Event(time + remainingTime, EventType.FINISHED, equiplet));
			System.out.printf("Simulation: schedule event FINISHED after repaired %.0f + %.0f, %s\n", time, remainingTime, equiplet);
		} else {
			equipletAgent.notifyRepaired(time);
		}

		double breakdown = stochastics.generateBreakdownTime(equiplet);
		//schedule BREAKDOWN time + breakdown, equiplet
		eventStack.add(new Event(time + breakdown, EventType.BREAKDOWN, equiplet));
		System.out.printf("Simulation: schedule event BREAKDOWN after repaired %.0f + %.0f, %s\n", time, breakdown, equiplet);
	}

	private void doneEvent() {
		running = false;
		finished = true;
	}

	private void scheduleDone() {
		eventStack.add(new Event(run_length, EventType.DONE));
	}

	private void validate() {
		List<String> productNames = new ArrayList<>(products.keySet());

		for (Event e : eventStack) {
			productNames.remove(e.getProduct());
		}
		
		for(String pName : productNames) {
			if (products.get(pName).getState() == ProductState.WAITING) {
				productNames.remove(pName);
			}
		}
		
		System.out.println("VALIDATION of product events " + productNames.isEmpty() + " : " + productNames + "- " + products);
		return;
		/*
		 * // An equiplet has not a job that is ready in the schedule when not
		 * executing
		 * for (Entry<String, Equiplet> entry : grid.getEquiplets().entrySet())
		 * {
		 * Equiplet equiplet = entry.getValue();
		 * equiplet.validateSchedule(time);
		 * }
		 */
	}

	public void saveStatistics() {
		// TODO Auto-generated method stub

	}

	private void update() {
		if (gui != null) {
			List<Triple<String, List<String>, Triple<String, Integer, Integer>>> equipletStates = new ArrayList<>();
			for (Entry<String, Equiplet> entry : grid.getEquiplets().entrySet()) {
				Equiplet equiplet = entry.getValue();
				equipletStates.add(new Triple<String, List<String>, Triple<String, Integer, Integer>>(equiplet.getEquipletName(), equiplet.getServices(), new Triple<String, Integer, Integer>(equiplet.getEquipletState().toString(), equiplet.getWaiting(), equiplet.executedJobs())));
			}

			System.out.printf("Update: [time=%.2f, product=%d, equiplets=%s]\n\n", time, products.size(), equipletStates);
			ArrayList<Double> busyPercentage = new ArrayList<>();
			for (Entry<Integer, Double> entry : B.entrySet()) {
				busyPercentage.add(entry.getValue() / time * 100);
			}

			double sum = 0;
			for (Entry<String, Double> entry : throughput.entrySet()) {
				sum += entry.getValue();
			}
			double avg = throughput.size() > 0 ? sum / throughput.size() : 0;

			gui.update(time, products.size(), productCount, totalSteps, grid.getEquiplets().values(), Q_n > 0 ? Q / Q_n : 0.0, busyPercentage, avg);
		}
	}

	public synchronized void pause() {
		System.out.println("Simulation: " + (running ? "start" : "pause"));

		running = !running;
		notify();//wake up the wait
	}

	public int getDelay() {
		return delay;
	}

	public void setDelay(int delay) {
		this.delay = delay;
	}

	public Map<String, Product> getProducts() {
		return products;
	}

	public List<Equiplet> getEquiplets() {
		return new ArrayList<Equiplet>(grid.getEquiplets().values());
	}
	
	public Map<String, Triple<Double, Double, Double>> getEquipletHistory() {
		Map<String, Triple<Double, Double, Double>> histories = new HashMap<String, Triple<Double, Double, Double>>();
		for (Entry<String, Equiplet> entry : grid.getEquiplets().entrySet()) {
			histories.put(entry.getValue().getEquipletName(), entry.getValue().getStatistics(time));
		}
		return histories;
	}

	private String formatArray(Map<String, ?> map) {
		StringBuffer buffer = new StringBuffer();
		for (Object item : map.entrySet()) {
			buffer.append("\n\t");
			buffer.append(item);
		}
		return buffer.toString();
	}
}

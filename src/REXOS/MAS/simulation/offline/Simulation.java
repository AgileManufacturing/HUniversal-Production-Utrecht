package simulation.offline;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import simulation.config.Config;
import simulation.config.DurationType;
import simulation.config.IConfig;
import simulation.graphics.StaticSimInterface;
import simulation.mas.equiplet.Capability;
import simulation.util.Pair;
import simulation.util.Position;
import simulation.util.Tick;
import simulation.util.Triple;

public class Simulation extends Thread {

	private Random random;
	private IConfig config;
	private StaticSimInterface gui;

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
	private HashMap<Integer, Product> products;
	private HashMap<Integer, Equiplet> equiplets;

	// Performance
	private double Q; // time of waiting products
	private double Q_n; // sum of waiting products
	private HashMap<Integer, Double> B;// time equiplets producing
	private HashMap<Integer, Double> throughput;
	private int traveling;
	private int totalSteps;

	public Simulation(StaticSimInterface staticSim) {
		this.gui = staticSim;
		config = Config.read();
		random = new Random();
		delay = 0;
		init();
	}

	private void init() {
		eventStack = new TreeSet<>();

		run = 0;
		runs = config.getRuns();
		run_length = config.getRunLength().doubleValue();
		finished = false;
		running = false;

		time = 0;

		totalSteps = 0;
		productCount = 0;
		products = new HashMap<>();
		equiplets = new HashMap<>();

		Q = 0;
		Q_n = 0;
		B = new HashMap<>();
		throughput = new HashMap<>();
		traveling = 0;

		// fill equiplets
		int equipletCounter = 0;
		for (Entry<String, Pair<simulation.util.Position, List<Capability>>> equiplet : config.getEquipletsConfigurations().entrySet()) {
			Position position = new Position(equiplet.getValue().first.getX(), equiplet.getValue().first.getY());
			List<String> services = new ArrayList<>();
			for (Capability capability : equiplet.getValue().second) {
				services.add(capability.getService());
			}
			equiplets.put(equipletCounter, new Equiplet(equiplet.getKey(), services, position));
			B.put(equipletCounter, 0.0);

			scheduleBreakdown(time, equipletCounter);
			equipletCounter++;
		}
		scheduleProduct(time);
		scheduleDone();
	}

	public void run() {
		while (run < runs) {
			while (!finished) {
				while (running) {
					Event e = eventStack.pollFirst();
					double elapsedTime = e.getTime() - time;
					time = e.getTime();

					// update statistics
					Q += Q_n * elapsedTime;
					for (Entry<Integer, Equiplet> entry : equiplets.entrySet()) {
						if (entry.getValue().getState() == EquipletState.BUSY) {
							double busy = B.get(entry.getKey()) + elapsedTime;
							B.put(entry.getKey(), busy);
						}
					}

					System.out.println("Simulation iteration: " + e + " : " + eventStack);

					switch (e.getType()) {
					case PRODUCT:
						// product is created in the system
						LinkedList<ProductStep> steps = e.getSteps();
						Product product = new Product(time, steps);
						products.put(productCount, product);
						totalSteps += steps.size();

						ProductStep step = product.getNextStep();
						String service = step.getService();

						// find best suitable equiplet to produce productStep
						int equipletID = findEquiplet(service);
						Position pos = equiplets.get(equipletID).getPosition();
						scheduleStart(time, new Position(-1, -1), pos, equipletID, productCount, step);
						productCount++;
						traveling++;

						// schedule new product arrival
						scheduleProduct(time);
						break;
					case EQUIPLET_START:
						// product comes to equiplet to perform product step
						traveling--;

						// Triple <Equiplet ID, Product ID, ProductStep>
						Triple<Integer, Integer, ProductStep> startData = e.getStartData();
						Equiplet equiplet = equiplets.get(startData.first);
						// add product step to the queue
						boolean goToProduce = equiplet.addJob(time, startData.second, startData.third);

						// if the equiplet is idle schedule when the product step is finished
						if (goToProduce) {
							scheduleFinish(time, startData.first);
						} else {
							// the product is waiting in the queue
							Q_n++;
						}
						break;
					case EQUIPLET_FINISHED:
						equiplet = equiplets.get(e.getEquiplet());
						if (equiplet.getState() == EquipletState.BROKEN) {
							// equiplet is broken while finishing the product step, remember the time is finished so it can continue when the equiplet is repaired
							equiplet.setShouldFinish(time);
						} else if (equiplet.getState() == EquipletState.WAS_BROKEN) {
							// equiplet was broken and repaired while producing the product step, continue the product step
							double timeBroken = equiplet.continueAfterBreakdown(time);
							eventStack.add(new Event(time + timeBroken, EventType.EQUIPLET_FINISHED, e.getEquiplet()));
						} else {
							// equiplet finished with the product step
							// remove the product step from the queue
							Pair<Integer, Boolean> finishResult = equiplet.currentJobFinished(time);
							int productID = finishResult.first;
							boolean canContinue = finishResult.second;

							// if product steps waiting in the queue, continue producing
							if (canContinue) {
								Q_n--;
								scheduleFinish(time, e.getEquiplet());
							}

							// check if product finished with all his product steps
							product = products.get(productID);
							if (!product.isFinished()) {
								ProductStep nextStep = product.getNextStep();
								int nextEquiplet = findEquiplet(nextStep.getService());
								Position nextPos = equiplets.get(nextEquiplet).getPosition();
								scheduleStart(time, equiplet.getPosition(), nextPos, nextEquiplet, productID, nextStep);
								traveling++;
							} else {
								// product finished
								products.remove(productID);
								// keep track of the throughput time of a product
								throughput.put(productID, time - product.getCreated());
							}
						}
						break;
					case EQUIPLET_BREAKDOWN:
						equiplet = equiplets.get(e.getEquiplet());
						equiplet.setTimeBroken(time);
						scheduleRepaired(time, e.getEquiplet());
						break;
					case EQUIPLET_REPAIRED:
						equiplet = equiplets.get(e.getEquiplet());
						if (equiplet.hasFinished()) {
							double remainingTime = time - equiplet.getShouldFinish();
							equiplet.continueAfterBreakdown(time);
							eventStack.add(new Event(time + remainingTime, EventType.EQUIPLET_FINISHED, e.getEquiplet()));
						} else {
							equiplet.setRepaired(time);
						}

						scheduleBreakdown(time, e.getEquiplet());
						break;
					case DONE:
						running = false;
						finished = true;
					default:
						break;
					}
					System.out.println("Equiplets: " + formatMap(equiplets));

					update();

					try {
						sleep(delay);
					} catch (InterruptedException ex) {
						ex.printStackTrace();
					}
				}

				if (!running) {
					try {
						sleep(1000);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
			saveStatistics();
			run++;
			// init();
		}
		gui.reset();
	}

	public void saveStatistics() {
		String path = "output";
		File file = new File(path);
		if (!file.exists()) {
			file.mkdir();
		}

		int number = 1;
		Pattern pattern = Pattern.compile("run([0-9]+)");
		File[] fileList = file.listFiles();
		for (File f : fileList) {
			Matcher matcher = pattern.matcher(f.getName());
			if (f.isDirectory() && matcher.find()) {
				number = Integer.valueOf(matcher.group(1)) + 1;
			}
		}

		path = String.format("%s/run%d/", path, number);
		File dir = new File(path);
		if (!dir.exists()) {
			dir.mkdir();
		}

		// TODO
		// StackedBarChart.save(path + "equiplets.png", "Equiplets production", getEquipletHistory());
	}

	private int findEquiplet(String service) {
		// choose equiplet to produce product step
		// the equiplet capable to perform the service and with the lowest queue is chosen
		int equipletID = -1;
		int score = Integer.MAX_VALUE;
		for (Entry<Integer, Equiplet> entry : equiplets.entrySet()) {
			Equiplet equiplet = entry.getValue();
			if (equiplet.isCapable(service) && (equiplet.getState() == EquipletState.IDLE || equiplet.getWaiting() < score)) {
				equipletID = entry.getKey();
				score = equiplet.getWaiting();
			}
		}
		return equipletID;
	}

	private void scheduleDone() {
		eventStack.add(new Event(run_length, EventType.DONE));
	}

	private void scheduleProduct(double time) {
		List<simulation.mas.product.ProductStep> productSteps = config.getProductSteps();
		LinkedList<ProductStep> steps = new LinkedList<ProductStep>();

		int n = 1 + (int) (random.nextDouble() * (productSteps.size() - 1));
		for (int i = 0; i < n; i++) {
			double u = random.nextDouble() * 100;
			int sum = 0;
			for (simulation.mas.product.ProductStep productStep : productSteps) {
				sum += config.getProductStepProbablity(productStep);
				if (u <= sum) {
					steps.add(new ProductStep(productStep.getService()));
					break;
				}
			}
		}

		double arrivalTime = time(config.getProductArrival());
		eventStack.add(new Event(time + arrivalTime, EventType.PRODUCT, steps));
	}

	private void scheduleStart(double time, Position a, Position b, int equiplet, int product, ProductStep step) {
		int travelSquares = Math.abs(a.getX() - b.getX()) + Math.abs(a.getY() - b.getY());
		double travelTime = travelSquares * time(config.getTravelTime());
		eventStack.add(new Event(time + travelTime, EventType.EQUIPLET_START, new Triple<Integer, Integer, ProductStep>(equiplet, product, step)));
	}

	private void scheduleFinish(double time, int equipletID) {
		Equiplet equiplet = equiplets.get(equipletID);
		ProductStep step = equiplet.getCurrentProductStep();

		System.out.printf("Schedule equiplet finish: [time=%.2f, equiplet id=%d, equiplet=%s\n", time, equipletID, equiplet);
		System.out.println("" + step);

		double productionTime = time(config.equipletProductionTime(equiplet.getName(), step.getService()));
		eventStack.add(new Event(time + productionTime, EventType.EQUIPLET_FINISHED, equipletID));
	}

	private void scheduleBreakdown(double time, int equipletID) {
		Equiplet equiplet = equiplets.get(equipletID);
		double breakdownTime = time(config.equipletBreakdownTime(equiplet.getName()));
		eventStack.add(new Event(time + breakdownTime, EventType.EQUIPLET_BREAKDOWN, equipletID));
	}

	private void scheduleRepaired(double time, int equipletID) {
		Equiplet equiplet = equiplets.get(equipletID);
		double repairTime = time(config.equipletRepaireTime(equiplet.getName()));
		eventStack.add(new Event(time + repairTime, EventType.EQUIPLET_REPAIRED, equipletID));
	}

	private void update() {
		if (gui != null) {
			List<Triple<String, List<String>, Triple<String, Integer, Integer>>> equipletStates = new ArrayList<>();
			for (Entry<Integer, Equiplet> entry : equiplets.entrySet()) {
				Equiplet equiplet = entry.getValue();
				equipletStates.add(new Triple<String, List<String>, Triple<String, Integer, Integer>>(equiplet.getName(), equiplet.getServices(), new Triple<String, Integer, Integer>(equiplet.getState().toString(), equiplet.getWaiting(), equiplet.executedJobs())));
			}

			System.out.printf("Update: [time=%.2f, product=%d, equiplets=%s]\n\n", time, products.size(), equipletStates);
			ArrayList<Double> busyPercentage = new ArrayList<>();
			for (Entry<Integer, Double> entry : B.entrySet()) {
				busyPercentage.add(entry.getValue() / time * 100);
			}

			double sum = 0;
			for (Entry<Integer, Double> entry : throughput.entrySet()) {
				sum += entry.getValue();
			}
			double avg = throughput.size() > 0 ? sum / throughput.size() : 0;
			gui.update(time, products.size(), productCount, totalSteps, traveling, equipletStates, Q_n > 0 ? Q / Q_n : 0.0, busyPercentage, avg);
		}
	}

	public Map<String, Triple<Double, Double, Double>> getEquipletHistory() {
		Map<String, Triple<Double, Double, Double>> histories = new HashMap<String, Triple<Double, Double, Double>>();
		for (Entry<Integer, Equiplet> entry : equiplets.entrySet()) {
			histories.put(entry.getValue().getName(), entry.getValue().getHistory(time));
		}
		return histories;
	}

	public void pause() {
		running = !running;
		System.out.println("Simulation: " + (running ? "start" : "pause"));
	}

	public int getDelay() {
		return delay;
	}

	public void setDelay(int delay) {
		this.delay = delay;
	}

	private double time(Pair<Tick, DurationType> time) {
		switch (time.second) {
		case EXP:
			return exp(time.first.doubleValue());
		case WEIBULL:
		case GAMMA:
		case NORMAL:
		case DETERMINISTIC:
		default:
			return time.first.doubleValue();
		}
	}

	private double exp(double mean) {
		double u = random.nextDouble();
		return -mean * Math.log(1 - u);
	}

	@SuppressWarnings("unused")
	private String formatArray(List<?> list) {
		StringBuffer text = new StringBuffer();
		for (Object item : list) {
			text.append(item.toString()).append('\n');
		}
		return text.toString();
	}

	private String formatMap(Map<?, ?> map) {
		StringBuffer text = new StringBuffer();
		for (Entry<?, ?> item : map.entrySet()) {
			text.append("\t" + item.getKey().toString() + "=" + item.getValue().toString()).append('\n');
		}
		return text.toString();
	}
}

package MAS.simulation.offline;

import jade.core.AID;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import MAS.product.Node;
import MAS.product.ProductStep;
import MAS.product.ProductionStep;
import MAS.product.Scheduling;
import MAS.product.SchedulingException;
import MAS.simulation.config.Config;
import MAS.simulation.simulation.Simulation;
import MAS.util.MASConfiguration;
import MAS.util.Pair;
import MAS.util.Position;
import MAS.util.SchedulingAlgorithm;
import MAS.util.Tick;

public class Sim extends Simulation<Product, Equiplet> {

	// dit ding ook maar twee keer bijhouden, jeeh
	// private StaticSimulation simulation;
	private Tick travelTime; 

	public Sim(StaticSimulation simulation) {
		super(simulation);
		// this.simulation = simulation;
		this.travelTime = Config.read().getTravelTime().first;
	}

	/**
	 * Event that signals the arrival of a new product in the system
	 * A product agent is created and started which will invoke the schedule
	 * behaviour
	 */
	@Override
	protected void productEvent() {
		try {
			// product agent settings
			LinkedList<ProductStep> productSteps = stochastics.generateProductSteps();

			String productName = "P" + productCount++;
			Position startPosition = new Position(-1, -1);

			Tick deadline = time.add(stochastics.generateDeadline());
			Product product = simulation.createProduct(productName, startPosition, productSteps, time, deadline);
			products.put(productName, product);
			product.schedule(time);

			// update statistics
			totalSteps += productSteps.size();
			updateProductStats(STATS_SYSTEM, +1);
		} catch (Exception e) {
			e.printStackTrace();
		}

		// wait for confirmation creation of product agent
		System.out.println("CHECKPOINT BRAVO");
	}

	public void scheduleProduct(String productName, Tick time, Position position, LinkedList<ProductStep> productSteps, Tick deadline) throws SchedulingException {
		// find capable equiplets
		// calculate travel times between equiplets
		// calculate production path
		// schedule equiplets

		System.out.printf(System.currentTimeMillis() + "\tPA:%s starts schedule behaviour at %s, product steps: %s  before deadline: %s\n", productName, time, productSteps, deadline);

		// option to execute product step ::
		// Map < product step index, Options to execute product step <Equiplet, Pair < estimate duration of service, List of possibilities < from time, until time> > > >
		Map<Integer, Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>>> options = new HashMap<>();

		// equiplet info :: list of equiplet with the load and position of the equiplet
		Map<AID, Pair<Double, Position>> equipletInfo = new HashMap<>();

		for (ProductStep productStep : productSteps) {
			Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>> option = new HashMap<>();

			for (Entry<String, Equiplet> equiplet : equiplets.entrySet()) {
				if (equiplet.getValue().isCapable(productStep.getService(), productStep.getCriteria())) {
					Tick duration = equiplet.getValue().estimateService(productStep.getService());
					List<Pair<Tick, Tick>> available = equiplet.getValue().available(time, duration, deadline);
					option.put(new EquipletAID(equiplet.getKey()), new Pair<>(duration, available));

					if (!equipletInfo.containsKey(equiplet.getKey())) {
						Tick window = deadline.minus(time);

						double load = equiplet.getValue().load(time, window);
						equipletInfo.put(new EquipletAID(equiplet.getKey()), new Pair<>(load, equiplet.getValue().getPosition()));
					}
				}
			}
			options.put(productStep.getIndex(), option);
		}
		System.out.printf(System.currentTimeMillis() + "\tPA:%s filter the capable equiplets %s\n", productName, equipletInfo.keySet());

		// the routes of equiplet to equiplet for which the time is needed for scheduling
		Map<Pair<Position, Position>, Tick> travelTimes = new HashMap<>();

		// constructing the routes, map for each product step the possible equiplet position to the next possibilities.
		// this creates a list of the edges the complete multipartite graph
		for (int i = productSteps.size() - 1; i > 0; i--) {
			// check if the product step is executable by one of the equiplets
			// note that at first there is no difference between the index of the product steps (i.e. productStep.get(i)) and the index of the product step (i.e.
			// productStep.get(i).getIndex()), when rescheduling the index do not match anymore
			int index = productSteps.get(i).getIndex();
			if (!options.containsKey(index)) {
				throw new SchedulingException("failed to find product step in options, i.e. there isn't a equiplet capable to execute the product step: " + productSteps.get(i));
			}
			if (!options.containsKey(index - 1)) {
				throw new SchedulingException("failed to find product step in options, i.e. there isn't a equiplet capable to execute the product step: " + productSteps.get(i - 1));
			}

			// System.out.println("[i=" + i + ", index=" + index + ", ps=" + productSteps.get(i) + ", options=" + options.get(i) + "]");
			// System.out.println("[i=" + i + ", index=" + index + ", ps=" + productSteps.get(i) + ", options=" + options.get(i) + "]");

			// map the possibilities for a product step to the next possibilities for the product step;
			Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>> previousPossibilities = options.get(index - 1);
			Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>> nextPossibilities = options.get(index);

			for (Entry<AID, Pair<Tick, List<Pair<Tick, Tick>>>> previousEquiplet : previousPossibilities.entrySet()) {
				for (Entry<AID, Pair<Tick, List<Pair<Tick, Tick>>>> nextEquiplet : nextPossibilities.entrySet()) {
					Position previousPos = equipletInfo.get(previousEquiplet.getKey()).second;
					Position nextPos = equipletInfo.get(nextEquiplet.getKey()).second;
					Pair<Position, Position> route = new Pair<>(previousPos, nextPos);
					if (!travelTimes.containsKey(route) && previousPos != nextPos) {
						travelTimes.put(route, caclulateTravelTime(previousPos, nextPos));
					}
				}
			}
		}

		// add routes from the current product position to the position of the possible equiplet for the first product step
		int firstIndex = productSteps.get(0).getIndex();
		if (options.containsKey(firstIndex)) {
			Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>> possibilities = options.get(firstIndex);
			for (Entry<AID, Pair<Tick, List<Pair<Tick, Tick>>>> equiplet : possibilities.entrySet()) {
				travelTimes.put(new Pair<>(position, equipletInfo.get(equiplet.getKey()).second), caclulateTravelTime(position, equipletInfo.get(equiplet.getKey()).second));
			}
		} else {
			throw new SchedulingException("failed to find product step in options, i.e. there isn't a equiplet capable to execute the product step: " + productSteps.get(0));
		}

		System.out.printf(System.currentTimeMillis() + "\tPA:%s retrieved travel times %s\n", productName, travelTimes);

		Scheduling scheduling = new Scheduling(productName, time, deadline, position, productSteps, options, equipletInfo, travelTimes);
		LinkedList<ProductionStep> productionPath;

		if (MASConfiguration.SCHEDULING == SchedulingAlgorithm.MATRIX) {
			productionPath = scheduling.calculateMatrixPath();

			System.out.printf(System.currentTimeMillis() + "\tPA:%s path calculated %s\n", productName, productionPath);

		} else {
			LinkedList<Node> nodes;
			if (MASConfiguration.SCHEDULING == SchedulingAlgorithm.EDD) {
				nodes = scheduling.calculateEDDPath();
			} else if (MASConfiguration.SCHEDULING == SchedulingAlgorithm.LOAD) {
				nodes = scheduling.calculateLoadPath();
			} else {
				nodes = scheduling.calculateSuprimePath();
			}

			System.out.printf(System.currentTimeMillis() + "\tPA:%s path calculated %s\n", productName, nodes);

			LinkedList<ProductionStep> path = new LinkedList<>();
			for (int i = 0; i < nodes.size(); i++) {
				Node node = nodes.get(i);
				AID equiplet = node.getEquipletAID();

				ProductStep step = productSteps.get(i);
				Position equipletPosition = equipletInfo.get(equiplet).second;

				ProductionStep production = new ProductionStep(step, equiplet, equipletPosition, node.getTime(), node.getDuration());
				path.add(production);
			}

			productionPath = path;
		}

		for (ProductionStep productionStep : productionPath) {
			Equiplet equiplet = equiplets.get(productionStep.getEquipletName());
			equiplet.schedule(productName, productionStep.getIndex(), productionStep.getStart(), deadline, productionStep.getService(), productionStep.getCriteria());
		}

		System.out.printf(System.currentTimeMillis() + "\tPA:%s scheduled equiplets.\n", productName);

		Product product = products.get(productName);
		product.schedulingFinished(time, true, productionPath);

		System.out.printf(System.currentTimeMillis() + "\tPA:%s scheduling done.\n", productName);
	}

	private Tick caclulateTravelTime(Position a, Position b) {
		int travelSquares = Math.abs(a.getX() - b.getX()) + Math.abs(a.getY() - b.getY());
		return travelTime.multiply(travelSquares);
	}

	public void informProductArrived(Tick time, String productName, String equipletName) {
		System.out.println("inform: ARRIVED [time=" + time + ", product=" + productName + ", equiplet=" + equipletName + "]");
		Equiplet equiplet = equiplets.get(equipletName);
		equiplet.notifyProductArrived(productName, time);
	}

	public void informProductProcessing(Tick time, String productName, String equipletName) {
		System.out.println("inform: PROCESSING [time=" + time + ", product=" + productName + ", equiplet=" + equipletName + "]");
		Product product = products.get(productName);
		product.informProductProcessing(time, equipletName);
	}

	public void informProductStepFinished(String productName, Tick time, int index) {
		System.out.println("inform: FINISHED [time=" + time + ", product=" + productName + ", index=" + index + "]");
		Product product = products.get(productName);
		product.informProductStepFinished(time, index);
	}

	public void informProductRelease(String productName, HashSet<String> equipletNames) {
		System.out.println("inform: RELEASE [product=" + productName + ", equiplets=" + equipletNames + "]");
		for (String equipletName : equipletNames) {
			Equiplet equiplet = equiplets.get(equipletName);
			if (!equiplet.release(productName)) {
				throw new IllegalArgumentException("Failed to release jobs");
			}
		}
	}
}

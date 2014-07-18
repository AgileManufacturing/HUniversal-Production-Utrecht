package simulation.mas;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import simulation.mas.scheduling.Node;
import simulation.mas.scheduling.Scheduling;
import simulation.simulation.Grid;
import simulation.util.Pair;
import simulation.util.Position;
import simulation.util.ProductStep;
import simulation.util.ProductionStep;
import simulation.util.Tuple;

public class Product {
	private String name;
	private double created;
	private LinkedList<ProductStep> productSteps;
	private LinkedList<ProductionStep> productionPath;
	private Position position;
	private double deadline;
	private ProductState state;

	public Product(String name, double time, LinkedList<ProductStep> steps, Position position) {
		this.name = name;
		this.created = time;
		this.productSteps = steps;
		this.position = position;
		this.productionPath = new LinkedList<>();

		this.deadline = Double.MAX_VALUE;
		this.state = ProductState.SCHEDULING;
		schedule(time);
	}

	private void schedule(double time) {
		Grid grid = Grid.getInstance();

		// Equiplets should be asked to the directory facilitator.
		// TODO check first if equiplet really capabale, criteria also match
		LinkedList<Pair<ProductStep, Map<String, Tuple<Double, Double, Double, Position>>>> steps = new LinkedList<>();
		for (ProductStep step : productSteps) {
			Map<String, Tuple<Double, Double, Double, Position>> suitedEquiplets = new HashMap<>();
			List<Equiplet> equipletList = grid.serviceSearch(step.getService());
			for (Equiplet equiplet : equipletList) {
				if (equiplet.isCapable(step.getService(), step.getCriteria())) {
					double available = equiplet.available(time, step.getService());
					double duration = equiplet.estimateService(step.getService());
					double load = equiplet.load(time, deadline);
					suitedEquiplets.put(equiplet.getEquipletName(), new Tuple<Double, Double, Double, Position>(available, duration, load, equiplet.getPosition()));
				}
			}

			steps.add(new Pair<ProductStep, Map<String, Tuple<Double, Double, Double, Position>>>(step, suitedEquiplets));
		}

		System.out.printf("P:%s scheduling: [at=%.0f, steps=%s\n", name, time, steps);
		Scheduling scheduling = new Scheduling();
		LinkedList<Node> nodes = scheduling.calculateEDDPath(time, steps, deadline, grid.getTravelCost(), position);

		if (nodes == null || nodes.size() != productSteps.size()) {
			System.out.println("P:" + name + "  FAILED to find production path nodes=" + (nodes != null ? nodes : "null"));
			state = ProductState.ERROR;
			return;
		}

		// TODO Check for failure in scheduling
		// Plan all the equiplets

		LinkedList<ProductionStep> path = new LinkedList<>();
		for (int i = 0; i < nodes.size(); i++) {
			Node node = nodes.get(i);
			ProductStep step = productSteps.get(i);
			String equiplet = node.getEquiplet();
			path.add(new ProductionStep(step, equiplet, node.getTime(), node.getDuration()));

			grid.getEquiplet(equiplet).schedule(node.getTime(), node.getTime() + node.getDuration(), name, step.getService(), new HashMap<String, Object>());
			// node.getEquiplet().schedule(node.getTime(), node.getDuration(),
			// getName());
		}

		// if all succeed:
		// state = State.WAITING;
		productionPath = path;

		System.out.println("P:" + name + "  productionpath=" + productionPath);
	}

	public String getName() {
		return name;
	}

	public double getCreated() {
		return created;
	}

	public ProductState getState() {
		return state;
	}

	public Position getPosition() {
		return position;
	}

	@Override
	public String toString() {
		return String.format("Product: %s [state=%s, created=%.2f, position=%s, current step=%s, product steps=%s, path=%s]", name, state, created, position,
				(productionPath.size() > 0 ? productionPath.peek() : "ERROR"), Arrays.toString(productSteps.toArray()), Arrays.toString(productSteps.toArray()));
	}

	public LinkedList<ProductionStep> getProductionPath() {
		return productionPath;
	}

	public String getNextEquipet() {
		state = ProductState.TRAVELING;
		return productionPath.peek().getEquipletName();
	}

	public ProductionStep getExecutingStep() {
		return productionPath.peek();
	}

	public boolean isFinished() {
		return productionPath.isEmpty();
	}

	public void notifyProductArrived() {
		// change state from travelling to ready
		state = ProductState.WAITING;
	}

	public void notifyProductStepFinished() {
		productionPath.poll();
		if (productionPath.isEmpty()) {
			state = ProductState.FINISHED;
		}
	}

	public void notifyProductProcessing() {
		state = ProductState.PROCESSING;
	}
}

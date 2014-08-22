package simulation.mas.product;

import jade.core.Agent;

import java.util.Arrays;
import java.util.LinkedList;

import simulation.util.Position;

public class Product extends Agent {
	
	private double created;
	private LinkedList<ProductStep> productSteps;
	private LinkedList<ProductionStep> productionPath;
	private Position position;
	private double deadline;
	private ProductState state;

	public void init(double time, LinkedList<ProductStep> steps, Position position) {
		this.created = time;
		this.productSteps = steps;
		this.position = position;
		this.productionPath = new LinkedList<>();

		this.deadline = Double.MAX_VALUE;
		this.state = ProductState.SCHEDULING;
	}
	public String getProductName() {
		return getLocalName();
	}

	public double getCreated() {
		return created;
	}

	public ProductState getProductState() {
		return state;
	}

	public Position getPosition() {
		return position;
	}

	@Override
	public String toString() {
		return String.format("Product: %s [state=%s, created=%.2f, position=%s, current step=%s, product steps=%s, path=%s]", getLocalName(), state, created, position, (productionPath.size() > 0 ? productionPath.peek()
				: "ERROR"), Arrays.toString(productSteps.toArray()), Arrays.toString(productSteps.toArray()));
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

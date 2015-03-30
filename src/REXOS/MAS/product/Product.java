package MAS.product;

import jade.core.Agent;

import java.util.Arrays;
import java.util.LinkedList;

import MAS.util.Position;
import MAS.util.Tick;

public class Product extends Agent {
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Tick created;
	private LinkedList<ProductStep> productSteps;
	private LinkedList<ProductionStep> productionPath;
	private Position position;
	private Tick deadline;
	private ProductState state;

	public void init(Tick time, Tick deadline, LinkedList<ProductStep> steps, Position position) {
		this.created = time;
		this.productSteps = steps;
		this.position = position;
		this.productionPath = new LinkedList<>();

		this.deadline = deadline;
		this.state = ProductState.SCHEDULING;
	}
	public String getProductName() {
		return getLocalName();
	}

	public Tick getCreated() {
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
		return String.format("Product: %s [state=%s, created=%s, position=%s, current step=%s, product steps=%s, path=%s]", getLocalName(), state, created, position, (productionPath.size() > 0 ? productionPath.peek()
				: "ERROR"), Arrays.toString(productSteps.toArray()), Arrays.toString(productSteps.toArray()));
	}

	public LinkedList<ProductionStep> getProductionPath() {
		return productionPath;
	}

	public String getNextEquiplet() {
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

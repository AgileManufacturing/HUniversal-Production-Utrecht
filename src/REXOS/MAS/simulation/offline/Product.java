package MAS.simulation.offline;

import java.util.Arrays;
import java.util.LinkedList;

class Product {
	private double created;
	private LinkedList<ProductStep> productSteps;
	private LinkedList<ProductStep> productionPath;

	public Product(double time, LinkedList<ProductStep> steps) {
		this.created = time;
		this.productSteps = steps;
		schedule();
	}

	private void schedule() {
		this.productionPath = productSteps;
	}

	public double getCreated() {
		return created;
	}

	public ProductStep getNextStep() {
		return productionPath.pop();
	}

	public boolean isFinished() {
		return productSteps.isEmpty();
	}

	@Override
	public String toString() {
		return String.format("Product:[created=%.2f, product steps=%s, current step=%s]", created, Arrays.toString(productSteps.toArray()), productionPath.peek());
	}
}

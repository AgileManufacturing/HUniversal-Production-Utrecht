package MAS.simulation.mas.product;

import java.util.LinkedList;

import MAS.simulation.simulation.Simulation;
import MAS.simulation.util.Position;
import MAS.simulation.util.Tick;

public class ProductSim implements IProductSim {

	private Product product;

	public ProductSim(Simulation simulation, String name, LinkedList<ProductStep> steps, Position position, Tick time, Tick deadline) {
		product = new Product();
		product.init(time, deadline, steps, position);
	}

	public void onProductArrived(Tick time) {
		product.notifyProductArrived();

	}

	@Override
	public Position getPosition() {
		return product.getPosition();
	}

	@Override
	public Tick getCreated() {
		return product.getCreated();
	}

}

package simulation.mas.product;

import java.util.LinkedList;

import simulation.simulation.Simulation;
import simulation.util.Position;

public class ProductSim implements IProductSim {

	private Product product;
	
	public ProductSim(Simulation simulation, String name, LinkedList<ProductStep> steps, Position position, double time) {
		product = new Product();
		product.init(time, steps, position);
	}

	public void onProductArrived(double time) {
		product.notifyProductArrived();
		
	}

	@Override
	public Position getPosition() {
		return product.getPosition();
	}

	@Override
	public double getCreated() {
		return product.getCreated();
	}

}

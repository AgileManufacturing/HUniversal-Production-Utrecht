package simulation.mas.product;

import simulation.util.Position;

public interface IProductSim {

	void onProductArrived(double time);

	Position getPosition();

	double getCreated();

}

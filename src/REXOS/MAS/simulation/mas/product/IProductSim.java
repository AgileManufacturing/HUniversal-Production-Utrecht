package simulation.mas.product;

import simulation.util.Position;
import simulation.util.Tick;

public interface IProductSim {

	void onProductArrived(Tick time);

	Position getPosition();

	Tick getCreated();

}

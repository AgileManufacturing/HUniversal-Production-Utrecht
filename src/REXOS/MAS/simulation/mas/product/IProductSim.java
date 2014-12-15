package MAS.simulation.mas.product;

import MAS.util.Position;
import MAS.util.Tick;

public interface IProductSim {

	void onProductArrived(Tick time);

	Position getPosition();

	Tick getCreated();

	Tick getDeadline();

	void kill();

}

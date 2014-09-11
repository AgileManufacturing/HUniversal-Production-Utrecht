package MAS.simulation.mas.product;

import MAS.simulation.util.Position;
import MAS.simulation.util.Tick;

public interface IProductSim {

	void onProductArrived(Tick time);

	Position getPosition();

	Tick getCreated();

}

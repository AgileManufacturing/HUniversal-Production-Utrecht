package MAS.simulation.mas.product;

import MAS.util.Position;
import MAS.util.Tick;

public interface IProductSim {
	
	void schedule(Tick time);

	void onProductArrived(Tick time);

	void onProductStarted(Tick time, int index);

	Position getPosition();

	Tick getCreated();

	Tick getDeadline();

	void kill();
}

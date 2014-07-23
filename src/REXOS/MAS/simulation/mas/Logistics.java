package simulation.mas;

import java.util.LinkedList;
import java.util.List;

import simulation.mas.equiplet.EquipletAgent;
import simulation.mas.product.ProductStep;
import simulation.util.Pair;

public class Logistics {
	
	public double getTravelTime() {
		return 10;
	}

	public Object getTravelTimes(LinkedList<Pair<ProductStep, List<Pair<EquipletAgent, Double>>>> steps) {
		return null;
	}
}

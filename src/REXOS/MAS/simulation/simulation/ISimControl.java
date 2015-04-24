package MAS.simulation.simulation;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import MAS.equiplet.Capability;
import MAS.product.ProductStep;
import MAS.simulation.mas.equiplet.IEquipletSim;
import MAS.simulation.mas.product.IProductSim;
import MAS.util.Position;
import MAS.util.Tick;

public interface ISimControl<Product extends IProductSim, Equiplet extends IEquipletSim> {

	void delay(long delay);

	Product createProduct(String name, Position position, LinkedList<ProductStep> productSteps, Tick time, Tick deadline) throws Exception;

	Equiplet createEquiplet(String name, Position position, List<Capability> capabilities) throws Exception;

	void killAgent(String name);

	void createTrafficAgent(Map<String, Position> equipletPositions) throws Exception;

	void takeDown();
}

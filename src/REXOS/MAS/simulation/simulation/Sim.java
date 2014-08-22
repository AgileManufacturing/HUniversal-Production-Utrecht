package simulation.simulation;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import simulation.mas.equiplet.Capability;
import simulation.mas.equiplet.EquipletSim;
import simulation.mas.equiplet.IEquipletSim;
import simulation.mas.product.IProductSim;
import simulation.mas.product.ProductSim;
import simulation.mas.product.ProductStep;
import simulation.util.Position;

public class Sim extends Thread implements ISimControl {

	private Simulation simulation;

	public Sim() {
		simulation = new Simulation(this);
	}
	
	@Override 
	public void run() {
		while(!simulation.isFinished()) {
			simulation.handleEvent();
		}
	}

	@Override
	public void delay(long delay) {
		try {
			sleep(delay);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	@Override
	public IProductSim createProduct(String name, Position position, LinkedList<ProductStep> productSteps, double time) throws Exception {
		return new ProductSim(simulation, name,productSteps, position, time);
	}

	@Override
	public IEquipletSim createEquiplet(String name, Position position, List<Capability> capabilities) throws Exception {
		return new EquipletSim(name, position, capabilities);
	}

	@Override
	public void killAgent(String name) {
		Grid grid = Grid.getInstance();
		grid.killAgent(name);
	}

	@Override
	public void createTrafficAgent(Map<String, Position> equipletPositions) {
		// TODO Auto-generated method stub
		
	}
}

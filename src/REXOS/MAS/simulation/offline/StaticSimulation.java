package MAS.simulation.offline;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import MAS.equiplet.Capability;
import MAS.product.ProductStep;
import MAS.simulation.simulation.ISimControl;
import MAS.util.MASConfiguration;
import MAS.util.Position;
import MAS.util.Tick;

public class StaticSimulation extends Thread implements ISimControl<Product, Equiplet> {

	public static void main(String[] args) {
		StaticSimulation sim = new StaticSimulation();
		sim.start();
	}

	private Sim simulation;
	
	public StaticSimulation() {
		setOutput();
		simulation = new Sim(this);
		simulation.init();

		// no use of gui, so start direct the simulation
		if (MASConfiguration.VERBOSITY < 2) {
			simulation.start();
		}
	}

	public void run() {
		while (!simulation.isFinished()) {
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
	public Product createProduct(String productName, Position position, LinkedList<ProductStep> productSteps, Tick time, Tick deadline) throws Exception {
		return new Product(simulation, productName, time, productSteps, position, deadline);
	}

	@Override
	public Equiplet createEquiplet(String equipletName, Position position, List<Capability> capabilities) throws Exception {
		return new Equiplet(simulation, equipletName, position, capabilities);
	}

	@Override
	public void killAgent(String name) {
		// let the gc do his job
	}

	@Override
	public void createTrafficAgent(Map<String, Position> equipletPositions) throws Exception {
		// no
	}

	@Override
	public void takeDown() {
		// let the gc do his job
	}

	private void setOutput() {
		if (MASConfiguration.VERBOSITY == 0 || MASConfiguration.VERBOSITY == 2) {
			System.setOut(new DummyPrint());
		}
	}

	/**
	 * because correct logging takes to much time to investigate a good way to log in a distributed system
	 */
	public class DummyPrint extends PrintStream {
		public DummyPrint() {
			super(new OutputStream() {
				@Override
				public void write(int b) throws IOException {

				}
			});
		}
	}
}

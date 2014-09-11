package simulation.simulation;

import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.wrapper.AgentController;
import jade.wrapper.ContainerController;
import jade.wrapper.ControllerException;
import jade.wrapper.StaleProxyException;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import simulation.mas.TrafficManager;
import simulation.mas.equiplet.Capability;
import simulation.mas.equiplet.EquipletSimAgent;
import simulation.mas.equiplet.IEquipletSim;
import simulation.mas.product.IProductSim;
import simulation.mas.product.ProductAgentSim;
import simulation.mas.product.ProductStep;
import simulation.util.Position;
import simulation.util.Settings;
import simulation.util.Tick;

public class SimulationAgent extends Agent implements ISimControl {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Simulation simulation;

	/**
	 * Setup the simulation agent
	 */
	public void setup() {
		simulation = new Simulation(this);

		addBehaviour(new SimulationBehaviour());
	}

	/**
	 * Simulation behaviour of the agent
	 */
	public class SimulationBehaviour extends Behaviour {
		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		@Override
		public void action() {
			System.out.printf("Simulation: action \n");
			while (!simulation.isFinished()) {
				simulation.handleEvent();
			}
		}

		@Override
		public boolean done() {
			return simulation.isFinished();
		}
	}

	@Override
	public void delay(long delay) {
		doWait(Math.max(1, delay));
	}

	@Override
	public IEquipletSim createEquiplet(String name, Position position, List<Capability> capabilities) throws Exception {
		try {
			// Create and start the agent
			EquipletSimAgent equiplet = new EquipletSimAgent(position, capabilities);

			ContainerController cc = getContainerController();
			AgentController ac = cc.acceptNewAgent(name, equiplet);
			ac.start();

			return equiplet;
		} catch (StaleProxyException e) {
			System.err.printf("Simulation: ERROR: equiplet agent %s creation was not possible.\n", name);
			e.printStackTrace();
			throw new Exception("Failed to create agent");
		} catch (NullPointerException e) {
			System.err.println("Simulation: not yet ready.");
			e.printStackTrace();
			throw new Exception("Failed to create agent");
		}

	}

	@Override
	public IProductSim createProduct(String name, Position position, LinkedList<ProductStep> productSteps, Tick time) throws Exception {
		try {
			System.out.println("Simulation: create product");
			// Create and start the agent
			Tick deadline = time.add(1000);
			ProductAgentSim productAgent = new ProductAgentSim(simulation, productSteps, position, time, deadline);

			ContainerController cc = getContainerController();
			AgentController ac = cc.acceptNewAgent(name, productAgent);
			ac.start();

			return productAgent;

		} catch (StaleProxyException e1) {
			System.err.printf("Simulation: ERROR: product agent %s creation was not possible.\n", name);
			e1.printStackTrace();
			throw new Exception("Failed to create agent");
		}
	}

	@Override
	public void killAgent(String name) {
		// kill the agent
		ContainerController cc = getContainerController();
		try {
			AgentController agent = cc.getAgent(name);
			agent.kill();
		} catch (ControllerException e) {
			e.printStackTrace();
		}

	}

	@Override
	public void createTrafficAgent(Map<String, Position> equiplets) throws Exception {
		try {
			System.out.println("Simulation: create traffic agent");

			// Create and start the agent
			TrafficManager trafficAgent = new TrafficManager(equiplets);

			ContainerController cc = getContainerController();
			AgentController ac = cc.acceptNewAgent(Settings.TRAFFIC_AGENT, trafficAgent);
			ac.start();
		} catch (StaleProxyException e1) {
			System.err.printf("Simulation: ERROR: traffic agent %s creation was not possible.\n", Settings.TRAFFIC_AGENT);
			e1.printStackTrace();
			throw new Exception("Failed to create agent");
		}
	}
}

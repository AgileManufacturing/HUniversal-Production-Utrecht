package MAS.simulation.simulation;

import jade.content.lang.Codec;
import jade.content.lang.sl.SLCodec;
import jade.content.onto.Ontology;
import jade.content.onto.basic.Action;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.domain.JADEAgentManagement.JADEManagementOntology;
import jade.domain.JADEAgentManagement.ShutdownPlatform;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.ContainerController;
import jade.wrapper.ControllerException;
import jade.wrapper.StaleProxyException;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import MAS.simulation.mas.TrafficManager;
import MAS.simulation.mas.equiplet.Capability;
import MAS.simulation.mas.equiplet.EquipletSimAgent;
import MAS.simulation.mas.equiplet.EquipletSimAgentDealWithItTemporyName;
import MAS.simulation.mas.equiplet.IEquipletSim;
import MAS.simulation.mas.product.IProductSim;
import MAS.simulation.mas.product.ProductAgentSim;
import MAS.simulation.mas.product.ProductAgentSimDealWithIt;
import MAS.simulation.mas.product.ProductStep;
import MAS.simulation.util.Position;
import MAS.simulation.util.SchedulingAlgorithm;
import MAS.simulation.util.Settings;
import MAS.simulation.util.Tick;

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
		try {
			int verbosity = Integer.parseInt(getProperty("verbosity", String.valueOf(Settings.VERBOSITY)));
			System.out.println("Simulation: verbosity is set on " + verbosity);
			Settings.VERBOSITY = verbosity;
		} catch (NumberFormatException e) {
			System.err.println("Simulation: parsing error verbosity");
		}
		setOutput();
		simulation = new Simulation(this);
		simulation.init();
		addBehaviour(new SimulationBehaviour());

		// no use of gui, so start direct the simulation
		if (Settings.VERBOSITY < 2) {
			simulation.start();
		}
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

			// take down the whole simulation when the gui is not used
			if (Settings.VERBOSITY < 1) {
				doDelete();
			}
		}

		@Override
		public boolean done() {
			return simulation.isFinished();
		}
	}

	@Override
	public void takeDown() {
		// TODO check if warning is because of bad coding and need to be fixed "WARNING: Cannot kill container Main-Container: Unreachable. "
		Codec codec = new SLCodec();
		Ontology jmo = JADEManagementOntology.getInstance();
		getContentManager().registerLanguage(codec);
		getContentManager().registerOntology(jmo);
		ACLMessage msg = new ACLMessage(ACLMessage.REQUEST);
		msg.addReceiver(getAMS());
		msg.setLanguage(codec.getName());
		msg.setOntology(jmo.getName());
		try {
			getContentManager().fillContent(msg, new Action(getAID(), new ShutdownPlatform()));
			send(msg);
		} catch (Exception e) {
		}
	};

	@Override
	public void delay(long delay) {
		doWait(Math.max(1, delay));
	}

	@Override
	public IEquipletSim createEquiplet(String name, Position position, List<Capability> capabilities) throws Exception {
		try {
			// Create and start the agent
			if (Settings.SCHEDULING == SchedulingAlgorithm.NONE) {
				EquipletSimAgentDealWithItTemporyName equiplet = new EquipletSimAgentDealWithItTemporyName(simulation, position, capabilities);

				ContainerController cc = getContainerController();
				AgentController ac = cc.acceptNewAgent(name, equiplet);
				ac.start();

				return equiplet;
			} else {
				EquipletSimAgent equiplet = new EquipletSimAgent(simulation, position, capabilities);

				ContainerController cc = getContainerController();
				AgentController ac = cc.acceptNewAgent(name, equiplet);
				ac.start();

				return equiplet;
			}
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
	public IProductSim createProduct(String name, Position position, LinkedList<ProductStep> productSteps, Tick time, Tick deadline) throws Exception {
		try {
			System.out.println("Simulation: create product");

			// Create and start the agent
			if (Settings.SCHEDULING == SchedulingAlgorithm.NONE) {
				ProductAgentSimDealWithIt productAgent = new ProductAgentSimDealWithIt(simulation, productSteps, position, time, deadline);

				ContainerController cc = getContainerController();
				AgentController ac = cc.acceptNewAgent(name, productAgent);
				ac.start();

				return productAgent;
			} else {
				ProductAgentSim productAgent = new ProductAgentSim(simulation, productSteps, position, time, deadline);

				ContainerController cc = getContainerController();
				AgentController ac = cc.acceptNewAgent(name, productAgent);
				ac.start();

				return productAgent;
			}
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

	private void setOutput() {
		if (Settings.VERBOSITY <= 2) {
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

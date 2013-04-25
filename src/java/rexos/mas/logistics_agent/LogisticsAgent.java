package rexos.mas.logistics_agent;

//import rexos.mas.logistics_agent.behaviours.ArePartsAvailable;
import jade.core.Agent;

public class LogisticsAgent extends Agent {
	private static final long serialVersionUID = 1L;
	
	@Override
	public void setup() {
		System.out.println("I spawned as a logistics agent.");
		//addBehaviour(new ArePartsAvailable(this));
	}
	
	@Override
	public void takeDown() {
		//
	}
}

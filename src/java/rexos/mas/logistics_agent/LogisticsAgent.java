package rexos.mas.logistics_agent;

import rexos.mas.logistics_agent.behaviours.GetTransportDurationResponse;
import jade.core.Agent;

public class LogisticsAgent extends Agent {
	private static final long serialVersionUID = 1L;
	
	@Override
	public void setup() {
		System.out.println("I spawned as a logistics agent.");
		addBehaviour(new GetTransportDurationResponse(this));
	}
	
	@Override
	public void takeDown() {
		//
	}
}

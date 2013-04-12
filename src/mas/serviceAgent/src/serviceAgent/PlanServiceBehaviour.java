package serviceAgent;

import jade.core.Agent;
import jade.core.behaviours.OneShotBehaviour;

import org.bson.types.ObjectId;

public class PlanServiceBehaviour extends OneShotBehaviour {
	private static final long serialVersionUID = 1L;

	private String service;

	public PlanServiceBehaviour(Agent agent, ObjectId stepID) {
		super(agent);
		this.service = service;
	}

	@Override
	public void action() {
		// TODO negotiate with logistics agent to determine when parts can
		// be here
		// send a message to equiplet agent with the answer.

		System.out.format("planning service %s%n", service);
	}
}

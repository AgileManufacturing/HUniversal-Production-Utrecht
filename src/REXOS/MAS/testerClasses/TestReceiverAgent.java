package MAS.testerClasses;

import jade.core.Agent;

public class TestReceiverAgent extends Agent{

	private static final long serialVersionUID = 1L;
	
	protected void setup() {
		addBehaviour(new TestReceiverAgentListenerBehaviour(this));
		System.out.println("Agent: " + getLocalName() + " started");
	}
	
	public void logMessage(String name, int messageID, long time){
		System.out.println("Name: " + name + " ID: " + messageID + " time: " + time + " time now: " + System.currentTimeMillis());
	}
}

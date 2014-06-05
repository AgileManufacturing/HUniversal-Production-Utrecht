package grid_server;

import java.util.ArrayList;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.TickerBehaviour;
import jade.lang.acl.ACLMessage;
/*
public class PulseBehaviour extends TickerBehaviour {
	private final String pulseMessage = "Is this agent still alive?";
	public PulseBehaviour(Agent a, long period) {
		super(a, period);
	}

	@Override
	protected void onTick() {
		ArrayList<AgentData> pulseAgents =(ArrayList<AgentData>) agents.DirectoryAgent.agents.clone();
		for(int i = 0; i < agents.DirectoryAgent.agents.size(); i++){
			agents.DirectoryAgent.agents.remove(i);
		}
		for(int i = 0; i < pulseAgents.size(); i++){
			ACLMessage acl = new ACLMessage(ACLMessage.INFORM);
			acl.setConversationId("Pulse");
			AID aid=new AID(pulseAgents.get(i).getName().toString(),AID.ISGUID);
			aid.addAddresses(pulseAgents.get(i).getAddress().toString());

			acl.addReceiver(aid);
			acl.setContent(pulseMessage);
			getAgent().send(acl);
		}
	}
}*/

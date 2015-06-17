package SCADA;

import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;

public class SCADAAgentListenerBehaviour extends CyclicBehaviour {

	@Override
	public void action() {
		ACLMessage msg = myAgent.blockingReceive();
		if(msg != null) {
			switch(msg.getPerformative()) {
			case ACLMessage.INFORM: 
				break;
			}
		}
	}

}

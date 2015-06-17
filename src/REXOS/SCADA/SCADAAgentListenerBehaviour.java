package SCADA;

import MAS.util.Ontology;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;

public class SCADAAgentListenerBehaviour extends CyclicBehaviour {
	SCADAAgent scada;
	
	public SCADAAgentListenerBehaviour(SCADAAgent scada){
		this.scada = scada;
	}
	@Override
	public void action() {
		ACLMessage msg = myAgent.blockingReceive();
		if(msg != null) {
			switch(msg.getPerformative()) {
			case ACLMessage.PROPOSE: 
				if(msg.getConversationId().equals(Ontology.CONVERSATION_LISTENER_COMMAND)) {
					System.out.println("SCADA AGENT: "+ msg.getContent());
					scada.onBasicUpdate(msg.getSender(), msg.getContent());
				}
				break;
			}
		}
	}
}

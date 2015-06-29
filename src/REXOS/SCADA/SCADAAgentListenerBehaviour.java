package SCADA;

import org.json.JSONException;
import org.json.JSONObject;

import MAS.util.Ontology;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;

public class SCADAAgentListenerBehaviour extends CyclicBehaviour {
	private static final long serialVersionUID = 1L;
	SCADAAgent scada;
	
	public SCADAAgentListenerBehaviour(SCADAAgent scada){
		this.scada = scada;
	}
	
	/**
	 * Handles the incoming ACLMessage on the SCADA agent.
	 */
	@Override
	public void action() {
		ACLMessage msg = myAgent.blockingReceive();
		if(msg != null) {
			switch(msg.getPerformative()) {
			case ACLMessage.PROPOSE: 
				if(msg.getConversationId().equals(Ontology.CONVERSATION_LISTENER_COMMAND)) {
					scada.onBasicUpdate( msg.getSender(), msg.getContent());
				}
				break;
			case ACLMessage.INFORM:
				if(msg.getConversationId().equals(Ontology.CONVERSATION_GET_DATA)){
					try {
						JSONObject content = new JSONObject(msg.getContent());
						String command = content.getString("command");
						switch(command){
						case "GET_OVERVIEW":
							scada.handleGetOverview(content);
							break;
						case "GET_DETAILED_INFO":
							scada.handleGetDetailedInfo(content);
							break;
						}
					} catch (JSONException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					
				} else if(msg.getConversationId().equals(Ontology.CONVERSATION_AGENT_TAKEDOWN)) {
					scada.removeAgentConnection(msg);
				} else if(msg.getConversationId().equals(Ontology.CONVERSATION_SCADA_COMMAND)) {
					scada.onDetailedUpdate(msg.getSender(), msg.getContent());
				}
				break;
			}
		}
	}
}

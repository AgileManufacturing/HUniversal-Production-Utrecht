package MAS.grid_server;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.Logger;
import MAS.util.Ontology;
import jade.core.AID;
import jade.core.behaviours.Behaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.SearchConstraints;
import jade.lang.acl.ACLMessage;
import jade.proto.SubscriptionInitiator;

public class GridAgentListenerBehaviour extends Behaviour{
	boolean done = false;
	GridAgent gridAgent = null;
	SubscriptionInitiator test  = null;
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public GridAgentListenerBehaviour(GridAgent gridAgent) {
		this.gridAgent = gridAgent;
		this.done = false;
		subscribeByDF();
	}
	
	void subscribeByDF(){
		DFAgentDescription description = new DFAgentDescription();
		SearchConstraints sc = new SearchConstraints();
		gridAgent.send(DFService.createSubscriptionMessage(gridAgent, gridAgent.getDefaultDF(), description, sc));
		System.out.println("GA subscribed by DF");
	}

	
	@Override
	public void action() {
		ACLMessage msg = gridAgent.blockingReceive();
		if (msg != null) {
			System.out.printf("GA:%s received message [sender=%s, performative=%s, conversation=%s, content=%s]\n", gridAgent.getLocalName(), msg.getSender().getLocalName(), msg.getPerformative(), msg.getConversationId(), msg.getContent());
			switch (msg.getPerformative()) {
				case ACLMessage.INFORM:
					if(msg.getConversationId().equals(Ontology.CONVERSATION_INFORMATION_REQUEST)) {
						System.out.println("GA inform conversation inform");
						gridAgent.sendAgentInfo(msg.getContent());
					}
//					if (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_ARRIVED)) {
//						handleProductArrived(msg);
//					} else if (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_RELEASE)) {
//						handleProductRelease(msg);
//					}
					break;
				// Request of other agent to get information to schedule a job
				// will send confirm or disconfirm message in return
				case ACLMessage.REQUEST:
					if(msg.getContent().equals("GETOVERVIEW")) {
						System.out.println("Request: GetOverview received!");
						this.gridAgent.getOverview();
						//Get all agents and send them to the SCADA agent.
					}
//					handleScheduling(msg);
					break;
				// query for information of the GridAgent
				case ACLMessage.QUERY_REF:
					//handleCanExecute(msg);
					break;
				case ACLMessage.QUERY_IF:
//					handleInformationRequest(msg);
					break;
				// messagetype holding the requested state for the equiplet
				case ACLMessage.PROPOSE:
					if(msg.getConversationId().equals(Ontology.CONVERSATION_LISTENER_COMMAND)){
						handleListenerCommand(msg);
					}
					break;
				default:
					break;
			}
		}
		
	}

	@Override
	public boolean done() {
		// TODO Auto-generated method stub
		return done;
	}

	
	private void handleListenerCommand(ACLMessage msg) {
		if(msg != null){
			try{
				JSONObject command = new JSONObject(msg.getContent());
				
				//Debug output
				Logger.log("Content of ACL message: " + command.toString());
				
				//Identifying modules
				String requestedEquipletCommand = command.getString("requested-listener-command").toString();
				
				// Program if statements that will appropriately handle messages sent to the GridAgent.
				if(requestedEquipletCommand.equals("AddDetailedListener")){
					System.out.println("addDetailedListener "+ gridAgent.toString());
					gridAgent.addBasicListener(msg.getSender());
				}else if(requestedEquipletCommand.equals("AddBasicListener")){
					System.out.println("addBasicListener "+ gridAgent.toString());
					gridAgent.addDetailedListener(msg.getSender());
				}else{
					Logger.log("An error occured while deserializing the ACLMessage, missing info or command not recognized.");
				}
				
			//Error handling
			} catch (JSONException e) {
				Logger.log("Invalid JSON.");
			}
		}		
	}
}

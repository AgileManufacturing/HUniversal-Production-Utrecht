package MAS.grid_server;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.Logger;
import MAS.util.Ontology;
import SCADA.BasicAgentInfo;
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
					//Subscribe to newly made agent.
					if(msg.getSender().equals(gridAgent.getDefaultDF())) {
						handleNewAgent(msg);
					} else if(msg.getConversationId().equals(Ontology.CONVERSATION_GET_DATA)) {
						handleDataResponse(msg);
					} else if(msg.getConversationId().equals(Ontology.CONVERSATION_INFORMATION_REQUEST)) {
						System.out.println("GA inform conversation inform");
						gridAgent.sendAgentInfo(msg.getContent());
					}else if(msg.getConversationId().equals(Ontology.CONVERSATION_AGENT_TAKEDOWN)){
						System.out.println("GA: ON TAKEDOWN: " + msg.getContent());
						gridAgent.onAgentTakeDown(msg.getContent());
					} else if(msg.getConversationId().equals(Ontology.CONVERSATION_SCADA_COMMAND)) {
						gridAgent.updateStateInfo(msg.getSender(), msg.getContent());
						gridAgent.onBasicUpdate(msg.getSender(), msg.getContent());
					}else{
						System.err.println("GA: Unkown message" + msg.getContent());
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
					try {
						JSONObject object = new JSONObject(msg.getContent().toString());
						String command = object.getString("command");
						if(command.equals("GET_OVERVIEW")) {
							System.out.println("Request: GetOverview received!");
							int clientHash = object.getInt("client");
							sendOverviewToSCADAAgent(msg, clientHash);
							//Get all agents and send them to the SCADA agent.
						}
					} catch (JSONException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
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
				JSONObject object = new JSONObject(msg.getContent());
				
				//Debug output
				Logger.log("Content of ACL message: " + object.toString());
				
				//Identifying modules
				String requestedEquipletCommand = object.getString("command").toString();
				
				// Program if statements that will appropriately handle messages sent to the GridAgent.
				if(requestedEquipletCommand.equals("AddDetailedListener")){
					System.out.println("addDetailedListener "+ gridAgent.toString());
					gridAgent.addDetailedListener(msg.getSender());
				}else if(requestedEquipletCommand.equals("AddBasicListener")){
					System.out.println("addBasicListener "+ gridAgent.toString());
					gridAgent.addBasicListener(msg.getSender());
				}else{
					Logger.log("An error occured while deserializing the ACLMessage, missing info or command not recognized.");
				}
				
			//Error handling
			} catch (JSONException e) {
				Logger.log("Invalid JSON.");
			}
		}		
	}
	
	private void handleNewAgent(ACLMessage msg) {
		try {
			DFAgentDescription[] results = DFService.decodeNotification(msg.getContent());
			System.out.println("AGENTS FOUND: " + results.length);
			for(int i = 0; i < results.length; i++) {
				DFAgentDescription dfd = results[i];
				AID agent = dfd.getName();
				
				//Ask Agent for BASIC INFO
				ACLMessage message = new ACLMessage(ACLMessage.QUERY_IF);
				message.setOntology(Ontology.GRID_ONTOLOGY);
				message.setConversationId(Ontology.CONVERSATION_GET_DATA);
				JSONObject object = new JSONObject();
				object.put("command", "GET_BASIC_INFO");
				message.setContent(object.toString());
				message.addReceiver(agent);
				gridAgent.send(message);
				
				//Subcribe on Agent Updates.
				ACLMessage reply = new ACLMessage(ACLMessage.PROPOSE);
				reply.addReceiver(agent);
				reply.setOntology(Ontology.GRID_ONTOLOGY);
				reply.setConversationId(Ontology.CONVERSATION_LISTENER_COMMAND);
				object = new JSONObject();
				object.put("command","ON_EQUIPLET_STATE_CHANGED");
				object.put("action", "REGISTER_LISTENER");
				reply.setContent(object.toString());
				gridAgent.send(reply);
			}
		} catch (FIPAException | JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private void handleDataResponse(ACLMessage msg) {
		try {
			System.out.println(msg);
			JSONObject object = new JSONObject(msg.getContent());
			switch(object.getString("command").toString()){
			case "GET_BASIC_INFO":
				JSONObject agent = object.getJSONObject("agent");
				String type  = agent.getString("type");
				String state = agent.getString("state");
				AID aid      = new AID(agent.getString("id"), AID.ISGUID);
				BasicAgentInfo bai = new BasicAgentInfo(aid,state,type);
				gridAgent.addBasicAgentInfo(bai);
				
				// Update all basicListeners (add agent)
				object = new JSONObject(msg.getContent());
				object.put("command", "ADDAGENT");
				object.put("agent", bai.getJSONObject());
				gridAgent.onBasicUpdate(msg.getSender(), object.toString());
				break;
			}
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private void sendOverviewToSCADAAgent(ACLMessage msg, int client) {
		ACLMessage reply = msg.createReply();
		reply.setConversationId(Ontology.CONVERSATION_GET_DATA);
		reply.setPerformative(ACLMessage.INFORM);
		try {
			JSONObject o = gridAgent.getJSONOfOverview().put("client", client);
			reply.setContent(o.toString());
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		gridAgent.send(reply);
	}
}

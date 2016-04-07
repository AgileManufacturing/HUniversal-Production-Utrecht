package MAS.grid_server;

import util.log.Logger;
import SCADA.BasicAgentInfo;
import jade.core.AID;
import jade.core.behaviours.Behaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.SearchConstraints;
import jade.lang.acl.ACLMessage;
import jade.proto.SubscriptionInitiator;
import java.util.LinkedList;

import jade.wrapper.AgentController;
import jade.wrapper.ContainerController;
import jade.wrapper.StaleProxyException;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import MAS.util.Tick;
import MAS.util.Position;
import MAS.util.Parser;
import MAS.util.Ontology;

import MAS.product.ProductAgent;
import MAS.product.ProductAgent2;
import MAS.product.ProductStep;

public class GridAgentListenerBehaviour extends Behaviour {
    private long productCounter = 0;
	boolean done = false;
	GridAgent gridAgent = null;
	SubscriptionInitiator test = null;

	private static final long serialVersionUID = 1L;

	public GridAgentListenerBehaviour(GridAgent gridAgent) {
		this.gridAgent = gridAgent;
		this.done = false;
		subscribeToDF();
	}

	/**
	 * Subscribe to the DF
	 * All registered/registering agents will be given to the GridAgent
	 */
	void subscribeToDF() {
		DFAgentDescription description = new DFAgentDescription();
		SearchConstraints sc = new SearchConstraints();
		gridAgent.send(DFService.createSubscriptionMessage(gridAgent,
				gridAgent.getDefaultDF(), description, sc));
	}

	/**
	 * This method is used for receiving ACLMessages
	 */
	@Override
	public void action() {
		ACLMessage msg = gridAgent.blockingReceive();
		if (msg != null) {
			switch (msg.getPerformative()) {
			case ACLMessage.INFORM:
				if (msg.getSender().equals(gridAgent.getDefaultDF())) {
					// Ugly hack to see if a new agent is registered or the agent is deregistered
					if (msg.getContent().contains(":services")) {
						handleNewAgent(msg);
					}else{
						// The agent is deregistered
					}
				} else if (msg.getConversationId().equals(
						Ontology.CONVERSATION_GET_DATA)) {
					handleDataResponse(msg);
				} else if (msg.getConversationId().equals(
						Ontology.CONVERSATION_INFORMATION_REQUEST)) {
					gridAgent.sendAgentInfo(msg.getContent());
				} else if (msg.getConversationId().equals(
						Ontology.CONVERSATION_AGENT_TAKEDOWN)) {
					gridAgent.onAgentTakeDown(msg.getContent());
				} else if (msg.getConversationId().equals(
						Ontology.CONVERSATION_SCADA_COMMAND)) {
					gridAgent
							.updateStateInfo(msg.getSender(), msg.getContent());
					gridAgent.sendAgentInfo(msg.getContent());
				} else {
					System.err.println("GA: Unkown message" + msg.getContent());
				}
				/**
				 * TODO SCADA check this
				 */
				// if
				// (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_ARRIVED))
				// {
				// handleProductArrived(msg);
				// } else if
				// (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_RELEASE))
				// {
				// handleProductRelease(msg);
				// }
				break;
			case ACLMessage.REQUEST:
				try {
					JSONObject object = new JSONObject(msg.getContent()
							.toString());
					String command = object.getString("command");
					
					if (command.equals("GET_OVERVIEW")) {
						int clientHash = object.getInt("client");
						sendOverviewToSCADAAgent(msg, clientHash);
					}else{
						System.err.println("GA: Unkown request " + msg.getContent());
					}
				} catch (JSONException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				break;
			/**
			 * TODO SCADA check this
			 */
			// query for information of the GridAgent
			case ACLMessage.QUERY_REF:
				// handleCanExecute(msg);
				break;
			case ACLMessage.QUERY_IF:
				// handleInformationRequest(msg);
				break;
			// messagetype holding the requested state for the equiplet
			case ACLMessage.PROPOSE:
				if (msg.getConversationId().equals(
						Ontology.CONVERSATION_LISTENER_COMMAND)) {
					handleListenerCommand(msg);
				} else if (msg.getConversationId().equals(
						Ontology.CONVERSATION_CREATE_AGENT)) {
					handleCreateAgent(msg);
				}
				break;
            case ACLMessage.ACCEPT_PROPOSAL:
                System.out.println(gridAgent.getLocalName() + ": received new request to spwan agent.");

				try {
					ContainerController cc = gridAgent.getContainerController();
					String name = "GPA" + productCounter++;

					// parse configurations
					LinkedList<ProductStep> productSteps = gridAgent.parseConfigurationProductSteps(msg.getContent());

					for (ProductStep productStep : productSteps) {
						// replace the criteria in each productstep by the actual identifiers of crates and objects
						productStep.updateCriteria(gridAgent.fillProductCriteria(productStep.getCriteria()));
					}

					// TODO hard coded, need to come from arguments
					Position startPosition = new Position(0, 0);
					Tick deadline = new Tick().add(1000000);

					Object[] arguments = new Object[] { Parser.parseProductConfiguration(productSteps, startPosition, deadline) };
					AgentController ac = cc.createNewAgent(name, ProductAgent.class.getName(), arguments);
					ac.start();

				} catch (StaleProxyException e) {
					e.printStackTrace();
				} catch (JSONException e) {
					System.err.println(gridAgent.getLocalName() + ": failed to parse product configurations: " + e.getMessage());
				}
                break;
			default:
				break;
			}
		}

	}
	/**
	 * done, default
	 * TODO implement method
	 * @return done 
	 */
	@Override
	public boolean done() {
		// TODO Auto-generated method stub
		return done;
	}

	/**
	 * Handle messages with conversationID: CONVERSATION_LISTENER_COMMAND
	 * This is used for adding basiListeners
	 * @param msg ACLMessage
	 */
	private void handleListenerCommand(ACLMessage msg) {
		if (msg != null) {
			try {
				JSONObject object = new JSONObject(msg.getContent());

				// Get the command
				String command = object.getString("command").toString();

				if (command.equals("AddBasicListener")) {
					System.out.println("addBasicListener "
							+ gridAgent.toString());
					gridAgent.addBasicListener(msg.getSender());
				} else {
					Logger.log("An error occured while deserializing the ACLMessage, missing info or command not recognized.");
				}

				// Error handling
			} catch (JSONException e) {
				Logger.log("Invalid JSON.");
			}
		}
	}

	/**
	 * Handle the message for a new Agent in the grid
	 * 
	 * @param msg contain information about the new agent (name)
	 */
	private void handleNewAgent(ACLMessage msg) {
		try {
			DFAgentDescription[] results = DFService.decodeNotification(msg
					.getContent());
			for (int i = 0; i < results.length; i++) {
				DFAgentDescription dfd = results[i];
				AID agent = dfd.getName();

				// Ask Agent for BASIC INFO
				ACLMessage message = new ACLMessage(ACLMessage.QUERY_IF);
				message.setOntology(Ontology.GRID_ONTOLOGY);
				message.setConversationId(Ontology.CONVERSATION_GET_DATA);
				JSONObject object = new JSONObject();
				object.put("command", "GET_BASIC_INFO");
				message.setContent(object.toString());
				message.addReceiver(agent);
				gridAgent.send(message);

				// Subcribe on Agent Updates.
				ACLMessage reply = new ACLMessage(ACLMessage.PROPOSE);
				reply.addReceiver(agent);
				reply.setOntology(Ontology.GRID_ONTOLOGY);
				reply.setConversationId(Ontology.CONVERSATION_LISTENER_COMMAND);
				object = new JSONObject();
				object.put("command", "ON_EQUIPLET_STATE_CHANGED");
				object.put("action", "REGISTER_LISTENER");
				reply.setContent(object.toString());
				gridAgent.send(reply);
			}
		} catch (FIPAException | JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * Create an agent in the grid
	 * @param msg contain agent information, id, type and arguments
	 */
	private void handleCreateAgent(ACLMessage msg) {
		try {
			JSONObject object = new JSONObject(msg.getContent());
			JSONObject agent = object.getJSONObject("agent");
			String id = agent.getString("id");
			String type = agent.getString("type");
			// TODO SCADA shouldn't this be a json object with something?
			String arguments = agent.getString("arguments");
			
			// select type of agent which should be spawned
			switch (type) {
			case "EquipletAgent":
				gridAgent.spawnEquipletAgent(id, arguments);
				break;
			case "ProductAgent":
				gridAgent.spawnProductAgent(id, arguments);
				break;
			// TODO SCADA not sure if these agent should be an option
			case "TrafficAgent":
				break;
			case "SupplyAgent":
				break;
			}
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * Handle the responds of the agent for the GET_BASIC_INFO message
	 * @param msg
	 */
	private void handleDataResponse(ACLMessage msg) {
		try {
			JSONObject object = new JSONObject(msg.getContent());
			switch (object.getString("command").toString()) {
			case "GET_BASIC_INFO":
				JSONObject agent = object.getJSONObject("agent");
				String type = agent.getString("type");
				String state = agent.getString("state");
				AID aid = new AID(agent.getString("id"), AID.ISGUID);
				
				// Add basic info of the agent to the list
				BasicAgentInfo bai = new BasicAgentInfo(aid, state, type);
				gridAgent.addBasicAgentInfo(bai);

				// Update all basicListeners (add agent)
				object = new JSONObject(msg.getContent());
				object.put("command", "ADD_AGENT");
				object.put("agent", bai.getJSONObject());
				gridAgent.sendAgentInfo(object.toString());
				break;
			}
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * Send the basic info gridoverview as reply to the message
	 * @param msg message containing the GET_OVERVIEW request
	 * @param client websocket identifier used in SCADAAgent
	 */
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

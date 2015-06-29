package SCADA;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;

import java.net.UnknownHostException;
import java.util.ArrayList;

import org.java_websocket.WebSocket;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import util.configuration.ServerConfigurations;
import MAS.equiplet.EquipletOnChangedHandler;
import MAS.util.Ontology;

public class SCADAAgent extends Agent implements WebSocketServerListener, SCADABasicListener, SCADADetailedListener{

	/**
	 *  Translates the client input from the WebSocketServer to messages in the JADE framework.
	 */
	private static final long serialVersionUID = 1L;
	
	private SCADAWebSocketServer webSocketServer;
	private static final int webSocketServerPort = 3529;
	
	private ArrayList<AgentConnection> agentConnections;
	private AgentConnection gridAgent;
	
	/**
	 * Basic agent setup. WebSocketServer starts and behaviours are added.
	 */
	protected void setup() {
		agentConnections = new ArrayList<AgentConnection>();
		try {
			webSocketServer = new SCADAWebSocketServer(webSocketServerPort);
			webSocketServer.start();
			webSocketServer.setListener(this);
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
		addBehaviour(new SCADAAgentListenerBehaviour(this));
	}
	
	/**
	 * This method receives the messages from the WebSocketServer and determines what should 
	 * happen and what agent in the grid should be contacted.
	 */
	@Override
	public void onWebSocketMessage(WebSocket webSocketConnection, String message) {
		// message convert to variables
		JSONObject jsonObject = null;
		try {
			jsonObject = new JSONObject(message);
			AID aid = null;
			if(!jsonObject.isNull("agent")){
				JSONObject agent = jsonObject.getJSONObject("agent");
				aid = new AID(agent.getString("id"), AID.ISLOCALNAME);
			}
			
			int index = 0;
			ACLMessage msg;
			JSONObject object = new JSONObject();
			String command = jsonObject.getString("command");
			switch(command) {
			case "GET_OVERVIEW": 
				//webSocketServer.sendMessage(webSocketConnection, "{\"type\":\"GridAgent\",\"agents\":[{\"ID\":\"EQ2\",\"type\":\"equiplet\",\"name\":\"Equiplet two\"},{\"ID\":\"EQ42\",\"type\":\"equiplet\",\"name\":\"Equiplet forty-two\"}, {\"ID\":\"PA2\",\"type\":\"product\",\"name\":\"Product two\"}]}");
				if(gridAgent != null){
					if((index = agentConnections.indexOf(gridAgent)) >= 0){
						// SCADAAgent is already connected to this agent
						agentConnections.get(index).addClient(webSocketConnection);		
					}
				}else{
					AID gridAgentAID = new AID(ServerConfigurations.GS_NAME, AID.ISGUID);
					gridAgent = new AgentConnection(gridAgentAID, webSocketConnection);
					agentConnections.add(gridAgent);
					connectToGridAgent(gridAgentAID);
				}
				msg = new ACLMessage(ACLMessage.REQUEST);
				msg.addReceiver(gridAgent.getAgent());
				object.put("command", "GET_OVERVIEW");
				object.put("client", webSocketConnection.hashCode());
				msg.setContent(object.toString());
				send(msg);
				break;
			case "GET_AGENT_INFO":
				removeClient(webSocketConnection);
				
				AgentConnection agentConn = null;
				for(int i = 0; i < agentConnections.size(); i++){
					if(agentConnections.get(i).getAgent().equals(aid)){
						agentConn = agentConnections.get(i);
						break;
					}
				}
				if(agentConn != null){
					index = agentConnections.indexOf(agentConn);
					// SCADAAgent is already connected to this agent
						
					// Remove old client connection from agent
					
					// Connect to new agent
					agentConnections.get(index).addClient(webSocketConnection);
					handleWebSocketGetAgentInfo(jsonObject, webSocketConnection);

				}else{
					agentConnections.add(new AgentConnection(aid, webSocketConnection));
					ACLMessage m = new ACLMessage(ACLMessage.QUERY_IF);
					JSONObject agent = jsonObject.getJSONObject("agent");
					AID receiver = new AID(agent.getString("id"), AID.ISLOCALNAME);
					
					JSONObject o = new JSONObject();
					o.put("command", "GET_DETAILED_INFO");
					o.put("client", webSocketConnection.hashCode());
					
					m.addReceiver(receiver);
					m.setOntology(Ontology.GRID_ONTOLOGY);
					m.setConversationId(Ontology.CONVERSATION_GET_DATA);
					m.setContent(o.toString());
					send(m);
					register(aid);
				}
				break;
			case "MODIFY_AGENT":
				handleModifyAgent(jsonObject);
				break;
			case "CREATE_AGENT":
				handleCreateAgent(jsonObject);
				break;
			case "GET_ALL_AGENTTYPES":
				handleGetAllAgentTypes(jsonObject);
			}
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * When the WebSocketServer onClose method is called this method gets called and removes 
	 * the client from the AgentConnection
	 */
	@Override
	public void onWebSocketClose(WebSocket webSocketConnection){
		removeClient(webSocketConnection);
	}
	
	/**
	 * Removes a WebSocket connection from an AgentConnection
	 * @param webSocketConnection - WebSocket to be removed form AgentConnection
	 */
	private void removeClient(WebSocket webSocketConnection){
		int index = 0;
		boolean clientFound = false;
		for(; index < agentConnections.size(); index++){
			if(agentConnections.get(index).removeClient(webSocketConnection) && agentConnections.get(index) != gridAgent){
				clientFound = true;
				break;
			}
		}

		if(clientFound){
			// Remove agent if no clients are connected
			if(agentConnections.get(index).getAmountOfClients() <= 0){
				AID aid = agentConnections.get(index).getAgent();
				deregister(aid);
				agentConnections.remove(index);
			}
		}
	}
	
	/**
	 * Remove an AgentConnection from the agentConnection list.
	 * @param msg - Based on this ACLMessage the to be removed AgentConnection get removed from the agentConnection list.
	 */
	public void removeAgentConnection(ACLMessage msg) {
		AgentConnection agentconnection = null;
		if(msg.getSender().equals(gridAgent.getAgent())){
			agentconnection = gridAgent;
		}else{
			for(AgentConnection ac : agentConnections) {
				if(ac.getAgent().equals(msg.getSender())) {
					agentconnection = ac;
				}
			}
		}
		
		if(agentconnection != null) {
			for(WebSocket ws : agentconnection.getClients()) {
				this.webSocketServer.sendMessage(ws, msg.getContent().toString());
				if(agentconnection != gridAgent){
					ws.close();
				}
			}
			if(agentconnection != gridAgent){
				agentConnections.remove(agentconnection);
			}
		}
	}
	
	/**
	 * Deregister the SCADA Agent as subscriber from an agent
	 * @param aid - The AID of the agent to be deregistered to
	 */
	private void deregister(AID aid){
		ACLMessage msg  = new ACLMessage(ACLMessage.PROPOSE);
		msg.addReceiver(aid);
		msg.setOntology(Ontology.GRID_ONTOLOGY);
		msg.setConversationId(Ontology.CONVERSATION_LISTENER_COMMAND);
		
		JSONObject object = new JSONObject();
		try {
			object.put("command",EquipletOnChangedHandler.OnChangedTypes.ALL);
			object.put("action", "DEREGISTER_LISTENER");
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		msg.setContent(object.toString());
		send(msg);
	}
	
	
	@Override
	public void onWebSocketOpen(WebSocket webSocketConnection) {

	}	
	
	/**
	 * Register the SCADA Agent as subscriber to an agent.
	 * @param agentID - The AID of the agent to be subscribed to
	 */
	private void register(AID agentID){
		ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
		JSONObject object = new JSONObject();
		try {
			object.put("command", EquipletOnChangedHandler.OnChangedTypes.ALL);
			object.put("action", "REGISTER_LISTENER");
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		message.addReceiver(agentID);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_LISTENER_COMMAND);
		message.setContent(object.toString());
		send(message);
	}
	
	/**
	 * Connect the SCADA Agent as a basicListener to the gridAgent.
	 * @param gridAgentID - AID of the gridAgent
	 */
	private void connectToGridAgent(AID gridAgentID){
		ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
		JSONObject object = new JSONObject();
		try {
			object.put("command", "AddBasicListener");
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		message.addReceiver(gridAgentID);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_LISTENER_COMMAND);
		message.setContent(object.toString());
		send(message);
	}
	
	/**
	 * Method to handle basic updates from agents
	 * @param agent - The AID of the agent. The AID will be used to look up the agents clients in the AgentConnection.
	 * @param message - The message to be send to the found clients in the AgentConnection.
	 */
	@Override
	public void onBasicUpdate(AID agent, String message) {
		
		AgentConnection agentConn = null;
		
		// Search AgentConnection for agent
		for(int i = 0; i < agentConnections.size(); i++){
			if(agentConnections.get(i).getAgent().equals(agent)){
				agentConn = agentConnections.get(i);
				break;
			}
		}
		
		// AgentConnection should be found
		if(agentConn != null){
			int index = agentConnections.indexOf(agentConn);
			ArrayList<WebSocket> clients = agentConnections.get(index).getClients();
			for(int i = 0; i < clients.size(); i++){
				webSocketServer.sendMessage(clients.get(i), message);
			}
		} else {
			System.out.println("AgentConnection is null");
		}
	}
	
	/**
	 * Method to handle the detailed update of an agent
	 * @param agent - The AID of the agent. The AID will be used to look up the agents clients in the AgentConnection.
	 * @param message - The message to be send to the found clients in the AgentConnection.
	 */
	@Override
	public void onDetailedUpdate(AID agent, String message) {
		AgentConnection agentConn = null;
		
		// Search AgentConnection for agent
		for(int i = 0; i < agentConnections.size(); i++){
			if(agentConnections.get(i).getAgent().equals(agent)){
				agentConn = agentConnections.get(i);
				break;
			}
		}
		
		// AgentConnection should be found
		if(agentConn != null){
			int index = agentConnections.indexOf(agentConn);
			ArrayList<WebSocket> clients = agentConnections.get(index).getClients();
			for(int i = 0; i < clients.size(); i++){
				webSocketServer.sendMessage(clients.get(i), message);
			}
		}
	}
	
	/**
	 * Method handles the gridAgent response on the GET_OVERVIEW command
	 * @param content - The JSONObject recevied from the gridAgent
	 */
	public void handleGetOverview(JSONObject content){
		try {
			int client = content.getInt("client");
			JSONArray agents = content.getJSONArray("agents");
			JSONObject object = new JSONObject();
			object.put("agents", agents);
			object.put("command", "GET_OVERVIEW");
			int index = 0;
			if((index = agentConnections.indexOf(gridAgent)) >= 0){
				// SCADAAgent is already connected to this agent
				WebSocket clientWS = agentConnections.get(index).getClientConnection(client);
				webSocketServer.sendMessage(clientWS, object.toString());
			}
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * Method handles the GetDetailedInfo response from an agent
	 * @param content - The JSONObject received from the agent
	 */
	public void handleGetDetailedInfo(JSONObject content){
		try {
			int client = content.getInt("client");
			content.remove("client");
			content.remove("Request");
			JSONObject agent = content.getJSONObject("agent");
			AID agentID = new AID(agent.getJSONObject("id").getString("value"), AID.ISLOCALNAME); 
			int index = 0;
			AgentConnection agentConn = null;
			for(int i = 0; i < agentConnections.size(); i++){
				if(agentConnections.get(i).getAgent().equals(agentID)){
					agentConn = agentConnections.get(i);
					break;
				}
			}
			if(agentConn != null){
				if((index = agentConnections.indexOf(agentConn)) >= 0){
					// SCADAAgent is already connected to this agent
					WebSocket clientWS = agentConnections.get(index).getClientConnection(client);
					webSocketServer.sendMessage(clientWS, content.toString());
				}
			}
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * This method handles the clients request to view an agent in detailed view.
	 * @param jsonObject - The JSONObject sent by the client containing the agents info
	 * @param webSocketConnection - The clients websocket connection.
	 */
	public void handleWebSocketGetAgentInfo(JSONObject jsonObject, WebSocket webSocketConnection){

		try {
			// Request Info
			ACLMessage msg = new ACLMessage(ACLMessage.QUERY_IF);
			JSONObject agent = jsonObject.getJSONObject("agent");
			AID receiver = new AID(agent.getString("id"), AID.ISLOCALNAME);
			
			JSONObject object = new JSONObject();
			object.put("command", "GET_DETAILED_INFO");
			object.put("client", webSocketConnection.hashCode());
			
			msg.addReceiver(receiver);
			msg.setOntology(Ontology.GRID_ONTOLOGY);
			msg.setConversationId(Ontology.CONVERSATION_GET_DATA);
			msg.setContent(object.toString());
			send(msg);
			
			//Subcribe on Agent Updates.
			register(receiver);
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	
	}
	
	/**
	 * Method handles the clients request to spawn a new agent
	 * @param object - JSONObject containing the information given by the client.
	 */
	private void handleCreateAgent(JSONObject object){
		ACLMessage msg = new ACLMessage(ACLMessage.PROPOSE);
		msg.setOntology(Ontology.GRID_ONTOLOGY);
		msg.setConversationId(Ontology.CONVERSATION_CREATE_AGENT);
		msg.addReceiver(gridAgent.getAgent());
		msg.setContent(object.toString());
		
		send(msg);
		
	}
	
	/**
	 * Handles the clients request to get all types of agents to show it on the website.
	 * @param object - JSONObject containing the infomormation given by the client
	 */
	private void handleGetAllAgentTypes(JSONObject object){
		ACLMessage msg = new ACLMessage(ACLMessage.QUERY_IF);
		msg.setOntology(Ontology.GRID_ONTOLOGY);
		msg.setConversationId(Ontology.CONVERSATION_GET_DATA);
		msg.addReceiver(gridAgent.getAgent());
		msg.setContent(object.toString());
		
		send(msg);
		
	}
	
	/**
	 * Handles the clients request to modify an agent based on the data filled in the form.
	 * @param object - JSONObject containing the filled in form.
	 */
	private void handleModifyAgent(JSONObject object){
		JSONObject agent;
		try {
			agent = object.getJSONObject("agent");
			AID aid = new AID(agent.getString("id"), AID.ISLOCALNAME);
			
			ACLMessage msg = new ACLMessage(ACLMessage.PROPOSE);
			msg.addReceiver(aid);
			msg.setOntology(Ontology.GRID_ONTOLOGY);
			msg.setConversationId(Ontology.CONVERSATION_MODIFY_AGENT);
			msg.setContent(object.toString());
			send(msg);
			
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}

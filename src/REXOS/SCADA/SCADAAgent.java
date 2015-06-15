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
import MAS.util.Ontology;

public class SCADAAgent extends Agent implements WebSocketServerListener, SCADABasicListener, SCADADetailedListener{

	/**
	 *  Default UID
	 */
	private static final long serialVersionUID = 1L;
	
	private SCADAWebSocketServer webSocketServer;
	private static final int webSocketServerPort = 3529;
	
	private ArrayList<AgentConnection> agentConnections;
	private AgentConnection gridAgent;
	
	protected void setup() {
		agentConnections = new ArrayList<AgentConnection>();
		try {
			webSocketServer = new SCADAWebSocketServer(webSocketServerPort);
			webSocketServer.start();
			webSocketServer.setListener(this);
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void onWebSocketMessage(WebSocket webSocketConnection, String message) {
		// message convert to variables
		System.out.println("SCADAAgent received: " + message);
		JSONObject jsonObject = null;
		try {
			jsonObject = new JSONObject(message);
			AID aid = new AID(jsonObject.getString("aid"), AID.ISGUID);
			int index = 0;
			switch(jsonObject.getString("command")) {
			case "GETOVERVIEW": 
				webSocketServer.sendMessage(webSocketConnection, "{\"type\":\"GridAgent\",\"agents\":[{\"Name\":\"EQ2\",\"type\":\"EquipletAgent\"},{\"Name\":\"EQ42\",\"type\":\"EquipletAgent\"}, {\"Name\":\"PA2\",\"type\":\"ProductAgent\"}]}");
//				if(gridAgent != null){
//					if((index = agentConnections.indexOf(gridAgent)) >= 0){
//						// SCADAAgent is already connected to this agent
//						agentConnections.get(index).addClient(webSocketConnection);
//					}
//				}else{
//					AID gridAgentAID = new AID(ServerConfigurations.GS_NAME, AID.ISGUID);
//					gridAgent = new AgentConnection(gridAgentAID, webSocketConnection);
//					agentConnections.add(gridAgent);
//					
//					connectToAgent(gridAgentAID);
//				}
				break;
			case "GETINFO":
				AgentConnection agent = null;
				for(int i = 0; i < agentConnections.size(); i++){
					if(agentConnections.get(i).getAgent().equals(aid)){
						agent = agentConnections.get(i);
						break;
					}
				}
				if(agent != null){
					index = agentConnections.indexOf(agent);
					// SCADAAgent is already connected to this agent
						
					// Remove old client connection from agent
					removeClient(webSocketConnection);
					// Connect to new agent
					agentConnections.get(index).addClient(webSocketConnection);
				}else{
					agentConnections.add(new AgentConnection(aid, webSocketConnection));
					connectToAgent(aid);
				}
				break;
			case "UPDATE":
				JSONArray jsonArray = jsonObject.getJSONArray("values");
				for(int i = 0; i < jsonArray.length(); i++){
					JSONObject o = jsonArray.getJSONObject(i);
					String method, param;
					method = o.getString("method");
					param = o.getString("param");
					executeMethodOnAgent(method, param);
				}
				break;
			}
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	@Override
	public void onWebSocketClose(WebSocket webSocketConnection){
		removeClient(webSocketConnection);
	}
	
	private void removeClient(WebSocket webSocketConnection){
		int index = 0;
		for(; index < agentConnections.size(); index++){
			if(agentConnections.get(index).removeClient(webSocketConnection)){
				break;
			}
		}
		
		// Remove agent if no clients are connected
		if(agentConnections.get(index).getAmountOfClients() <= 0){
			agentConnections.remove(index);
		}
	}
	@Override
	public void onWebSocketOpen(WebSocket webSocketConnection) {
		System.out.println("SCADA agent: new socket opened!");
	}	
	
	private void connectToAgent(AID agentID){
		ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
		String JSONMessage = "{\n 'requested-listener-command': 'AddDetailedListener',\n }";
		
		message.addReceiver(agentID);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_LISTENER_COMMAND);
		message.setContent(JSONMessage);
		send(message);
		System.out.println("SCADAAgent message send!");
	}
	
	
	public void executeMethodOnAgent(String method, String param) {
		switch(method) {
		case "test":
			System.out.println("Executing " + method + " with param: " + param);
			break;
			default:
		}
	}

	@Override
	public void onBasicUpdate(AID agent, String message) {
		System.out.println("SCADA: basicUpdate from: " + agent.toString() + "message: " + message);
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

	@Override
	public void onDetailedUpdate(AID agent, String message) {
		System.out.println("SCADA: detailedUpdate from: " + agent.toString() + "message: " + message);
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
}

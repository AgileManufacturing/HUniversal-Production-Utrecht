package SCADA;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;

import java.net.UnknownHostException;
import java.util.ArrayList;

import org.java_websocket.WebSocket;

import MAS.util.Ontology;

public class SCADAAgent extends Agent implements WebSocketServerListener{

	/**
	 *  Default UID
	 */
	private static final long serialVersionUID = 1L;
	
	private SCADAWebSocketServer webSocketServer;
	private static final int webSocketServerPort = 3529;
	
	private ArrayList<AgentConnection> agentConnections;
	
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
		
		convertMessage(message);
		
		// WARNING THIS DOES NOT WORK.
		AID agentAID = null;
		
		// change agent?
		int index;
		if((index = agentConnections.indexOf(agentAID)) >= 0){
			// SCADAAgent is already connected to this agent
			agentConnections.get(index).addClient(webSocketConnection);
		}else{
			agentConnections.add(new AgentConnection(agentAID, webSocketConnection));
		}
		
	}
	
	@Override
	public void onWebSocketClose(WebSocket webSocketConnection){
		// Problem which agent was the client connected to?
		
		for(int i = 0; i < agentConnections.size(); i++){
			if(agentConnections.get(i).removeClient(webSocketConnection)){
				break;
			}
		}
	}
	@Override
	public void onWebSocketOpen(WebSocket webSocketConnection) {
		int index;
		String ip = "127.0.0.1";
		AID gridAgent = new AID("Grid@"+ip+"/JADE", AID.ISGUID);
		if((index = agentConnections.indexOf(gridAgent)) >= 0){
			// SCADAAgent is already connected to this agent
			agentConnections.get(index).addClient(webSocketConnection);
		}else{
			agentConnections.add(new AgentConnection(gridAgent, webSocketConnection));
			
			connectToAgent(gridAgent);
		}
	}
	
	private void convertMessage(String message){
		
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
	
	
}

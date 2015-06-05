package SCADA;

import jade.core.AID;
import jade.core.Agent;

import java.net.UnknownHostException;
import java.util.ArrayList;

import org.java_websocket.WebSocket;

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
			agentConnections.get(i).removeClient(webSocketConnection);
		}
	}
	
	public void convertMessage(String message){
		
	}
	
	
	
	
	
}

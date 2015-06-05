package SCADA;

import jade.core.AID;
import org.java_websocket.WebSocket;
import java.util.ArrayList;

public class AgentConnection {
	private AID agent;
	private ArrayList<WebSocket> clients;
	
	public AgentConnection(AID agent, WebSocket client){
		this.agent = agent;
		clients = new ArrayList<WebSocket>();
		clients.add(client);
	}
	
	public boolean addClient(WebSocket client){
		return clients.add(client);
	}
	
	public boolean removeClient(WebSocket client){
		return clients.remove(client);
	}
	
	public int getAmountOfClients(){
		return clients.size();
	}
}

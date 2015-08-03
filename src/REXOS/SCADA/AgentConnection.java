package SCADA;

import jade.core.AID;
import org.java_websocket.WebSocket;
import java.util.ArrayList;

/**
 * Keeps track of which client is listening to the agent with the given AID.
 */

public class AgentConnection {
	private AID agent;
	private ArrayList<WebSocket> clients;
	
	public AgentConnection(AID agent, WebSocket client){
		this.agent = agent;
		clients = new ArrayList<WebSocket>();
		clients.add(client);
	}
	
	/**
	 * Add a client to the list clients who watch the agent
	 * @param client - WebSocket to be added
	 * @return boolean - if true, the client is succesfully added else not.
	 */
	public boolean addClient(WebSocket client){
		return clients.add(client);
	}
	
	/**
	 * Remove a client from the list of clients who watch the agent
	 * @param client - WebSocket to be removed
	 * @return boolean - if true, the client is succesfully removed, else not.
	 */
	public boolean removeClient(WebSocket client){
		return clients.remove(client);
	}
	
	/**
	 * Get the amount of clients in the list
	 * @return integer - The amount of clients.
	 */
	public int getAmountOfClients(){
		return clients.size();
	}
	
	/**
	 * Get the corresponsding agent from the AgentConnection
	 * @return AID - The AID of the agent
	 */
	public AID getAgent(){
		return agent;
	}
	
	/**
	 * Get the list of clients.
	 * @return ArrayList<WebSocket> - the list of websockets
	 */
	public ArrayList<WebSocket> getClients(){
		return clients;
	}
	
	/**
	 * Get a client WebSocket based on a given hashcode
	 * @param hash - The hashcode of a WebSocket
	 * @return WebSocket - The WebSocket corresponding with the hashcode
	 */
	public WebSocket getClientConnection(int hash){
		for(WebSocket w : clients){
			if(w.hashCode() == hash){
				return w;
			}
		}
		return null;
	}
}

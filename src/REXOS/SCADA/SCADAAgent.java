package SCADA;

import jade.core.AID;
import jade.core.Agent;

import java.net.UnknownHostException;
import java.util.ArrayList;

import org.java_websocket.WebSocket;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

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
		JSONObject jsonObject = null;
		try {
			jsonObject = new JSONObject(message);
			AID aid = new AID(jsonObject.getString("aid"), AID.ISGUID);
			switch(jsonObject.getString("command")) {
			case "GETINFO":
				//connecToAgent(aid);
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
		// Problem which agent was the client connected to?
		
		for(int i = 0; i < agentConnections.size(); i++){
			agentConnections.get(i).removeClient(webSocketConnection);
		}
	}	
	
	public void executeMethodOnAgent(String method, String param) {
		switch(method) {
		case "test":
			System.out.println("Executing " + method + " with param: " + param);
			break;
			default:
		}
	}
}

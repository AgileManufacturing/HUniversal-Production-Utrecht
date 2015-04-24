package SCADA;

import java.net.UnknownHostException;
import java.util.Map;

import org.java_websocket.WebSocket;

import MAS.util.Triple;

public class WebSocketContainer implements WebSocketListener {
	
	private static final int clientServerPort = 3527;
	private static final int scadaAgentServerPort = 3528;
	private Triple<Integer,WebSocket, WebSocket> connectionList;
	
	private SCADAWebSocketServer clientServer, scadaAgentServer;
	
	public WebSocketContainer() throws UnknownHostException {
		clientServer = new SCADAWebSocketServer(clientServerPort);
		clientServer.setListener(this);
		
		scadaAgentServer = new SCADAWebSocketServer(scadaAgentServerPort);
		scadaAgentServer.setListener(this);
		
		clientServer.start();
		scadaAgentServer.start();
	}
	
	public void addConnection(WebSocket conn) {
		
	}
	
	public void sendMessage(WebSocket conn, String msg) {
		
	}
	
	public void closeConnection(WebSocket conn) {
		
	}
}

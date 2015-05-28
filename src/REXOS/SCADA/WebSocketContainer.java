package SCADA;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Map;

import org.java_websocket.WebSocket;

import MAS.util.Pair;
import MAS.util.Triple;

/*
 * 
 * @author: Pim te Slaa
 * 
 */

public class WebSocketContainer implements WebSocketListener {
	
	private static final int CLIENTSERVERPORT      = 3528;
	private static final int SCADA_AGENTSERVERPORT = 3529;
	
	private ArrayList<Pair<WebSocket, WebSocket>> connectionList;
	
	private SCADAWebSocketServer clientServer, scadaAgentServer;
	
	public WebSocketContainer() throws UnknownHostException {
		connectionList = new ArrayList<Pair<WebSocket,WebSocket>>();
		
		clientServer = new SCADAWebSocketServer(CLIENTSERVERPORT);
		clientServer.setListener(this);
		
		scadaAgentServer = new SCADAWebSocketServer(SCADA_AGENTSERVERPORT);
		scadaAgentServer.setListener(this);
		
		scadaAgentServer.start();
		clientServer.start();
	}
	
	public void addConnection(WebSocket conn) {
		if(conn.getRemoteSocketAddress().getPort() == CLIENTSERVERPORT) {
			connectionList.add(new Pair<WebSocket,WebSocket>(conn, null));
			/*
			 * Send message to GridAgent to create new ScadaAgent;
			 */
		} else if(conn.getRemoteSocketAddress().getPort() == SCADA_AGENTSERVERPORT){
			for(Pair<WebSocket,WebSocket> p : connectionList) {
				if(p.second == null) {
					p.second = conn;
					break;
				}
			}
		}
	}
	
	public void sendMessage(WebSocket conn, String msg) {
		if(conn.getRemoteSocketAddress().getPort() == CLIENTSERVERPORT) {
			for(Pair<WebSocket,WebSocket> p : connectionList) {
				if(p.first.equals(conn)) {
					p.second.send(msg);
				}
			}
		} else if(conn.getRemoteSocketAddress().getPort() == SCADA_AGENTSERVERPORT) {
			for(Pair<WebSocket,WebSocket> p : connectionList) {
				if(p.second.equals(conn)) {
					p.first.send(msg);
				}
			}
		}
	}
	
	public void closeConnection(WebSocket conn) {
		if(conn.getRemoteSocketAddress().getPort() == CLIENTSERVERPORT) {
			for(Pair<WebSocket,WebSocket> p : connectionList) {
				if(p.first.equals(conn)) {
					p.second.close();
				}
			}
		} else if(conn.getRemoteSocketAddress().getPort() == SCADA_AGENTSERVERPORT) {
			for(Pair<WebSocket,WebSocket> p : connectionList) {
				if(p.second.equals(conn)) {
					p.first.close();
				}
			}
		}
	}
}

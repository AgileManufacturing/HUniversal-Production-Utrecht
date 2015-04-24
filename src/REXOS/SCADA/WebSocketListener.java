package SCADA;

import org.java_websocket.WebSocket;

public interface WebSocketListener {
	
	public void addConnection(WebSocket conn);
	public void sendMessage(WebSocket conn, String msg);
	public void closeConnection(WebSocket conn);
}

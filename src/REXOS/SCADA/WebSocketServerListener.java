package SCADA;

import org.java_websocket.WebSocket;

public interface WebSocketServerListener {
	public void onWebSocketMessage(WebSocket conn, String message);
	public void onWebSocketClose(WebSocket conn);
	public void onWebSocketOpen(WebSocket conn);
}

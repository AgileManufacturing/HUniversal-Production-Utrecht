package SCADA;

import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

/*
 * 
 * @author: Pim te Slaa
 * 
 */

public class SCADAWebSocketServer extends WebSocketServer {
	private ArrayList<WebSocket> connectionList;
	private WebSocketServerListener theListener;

	public SCADAWebSocketServer(int port) throws UnknownHostException {
		super(new InetSocketAddress(port));
		connectionList = new ArrayList<WebSocket>();
		System.out.println("WSS created");
	}
	
	@Override
	public void onOpen(WebSocket conn, ClientHandshake handshake) {
		connectionList.add(conn);
	}

	@Override
	public void onMessage(WebSocket conn, String message) {
		System.out.println("WebSocketServer received: " +  message);
		theListener.onWebSocketMessage(message);
	}
	
	@Override
	public void onClose(WebSocket conn, int code, String reason, boolean remote) {
		connectionList.remove(conn);
	}

	@Override
	public void onError(WebSocket conn, Exception ex) {
		ex.printStackTrace();
	}
	
	public void sendMessage(WebSocket conn, String message) {
		conn.send(message);
	}
	
	public void setListener(WebSocketServerListener s) {
		theListener = s;
	}
}

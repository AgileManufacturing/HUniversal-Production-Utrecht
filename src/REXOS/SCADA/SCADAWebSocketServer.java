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

/**
 * WebSocketServer used by the SCADA system.
 */
public class SCADAWebSocketServer extends WebSocketServer {
	private ArrayList<WebSocket> connectionList;
	private WebSocketServerListener theListener;

	public SCADAWebSocketServer(int port) throws UnknownHostException {
		super(new InetSocketAddress(port));
		connectionList = new ArrayList<WebSocket>();
		System.out.println("WSS created");
	}
	
	/**
	 * Method that gets called when a client connects to the SocketServer and a WebSocket gets opened.
	 */
	@Override
	public void onOpen(WebSocket conn, ClientHandshake handshake) {
		theListener.onWebSocketOpen(conn);
		connectionList.add(conn);
	}

	/**
	 * Method that gets called when a message is received on the SocketServer.
	 */
	@Override
	public void onMessage(WebSocket conn, String message) {
		System.out.println("WebSocketServer received: " +  message);
		theListener.onWebSocketMessage(conn, message);
	}
	
	/**
	 * Method that gets called when a WebSocket closes.
	 */
	@Override
	public void onClose(WebSocket conn, int code, String reason, boolean remote) {
		theListener.onWebSocketClose(conn);
		connectionList.remove(conn);
	}

	/**
	 * Method that gets called when an error has occured on a WebSocket or on the WebSocketServer.
	 */
	@Override
	public void onError(WebSocket conn, Exception ex) {
		ex.printStackTrace();
	}
	
	/**
	 * Method used to send a message to the specified WebSocket.
	 * @param conn - WebSocket to send the message to.
	 * @param message - String message to be send.
	 */
	public void sendMessage(WebSocket conn, String message) {
		conn.send(message);
	}
	
	/**
	 * Method to set a listener to listen to the WebSocketServer via the WebSocketServerListener interface
	 * @param s - WebSocketServerListener the listener to be set.
	 */
	public void setListener(WebSocketServerListener s) {
		theListener = s;
	}
}

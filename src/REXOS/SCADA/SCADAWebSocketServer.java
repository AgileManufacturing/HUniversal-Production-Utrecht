package SCADA;

import java.net.InetSocketAddress;
import java.net.UnknownHostException;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

/*
 * 
 * @author: Pim te Slaa
 * 
 */

public class SCADAWebSocketServer extends WebSocketServer {
	
	public SCADAWebSocketServer(int port) throws UnknownHostException {
		super(new InetSocketAddress(port));
	}
	
	@Override
	public void onOpen(WebSocket conn, ClientHandshake handshake) {
		
	}

	@Override
	public void onMessage(WebSocket conn, String message) {

	}
	
	@Override
	public void onClose(WebSocket conn, int code, String reason, boolean remote) {
	
	}

	@Override
	public void onError(WebSocket conn, Exception ex) {

	}
}

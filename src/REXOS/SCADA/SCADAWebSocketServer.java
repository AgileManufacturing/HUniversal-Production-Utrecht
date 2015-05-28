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
	
	private WebSocketListener theListener;
	
	public SCADAWebSocketServer(int port) throws UnknownHostException {
		super(new InetSocketAddress(port));
	}
	
	@Override
	public void onOpen(WebSocket conn, ClientHandshake handshake) {
		theListener.addConnection(conn);
	}

	@Override
	public void onMessage(WebSocket conn, String message) {
		theListener.sendMessage(conn, message);
	}
	
	@Override
	public void onClose(WebSocket conn, int code, String reason, boolean remote) {
		theListener.closeConnection(conn);
	}

	@Override
	public void onError(WebSocket conn, Exception ex) {

	}
	
	public void setListener(WebSocketListener l) {
		theListener = l;
	}
}
